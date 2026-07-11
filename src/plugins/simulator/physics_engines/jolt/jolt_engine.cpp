/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_engine.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_engine.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_model.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_plugin.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* Broad-phase layers mirror the object layers one-to-one */
   namespace JoltBroadPhaseLayers {
      static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
      static constexpr JPH::BroadPhaseLayer MOVING(1);
      static constexpr JPH::uint NUM_LAYERS = 2;
   }

   class CJoltBPLayerInterface final : public JPH::BroadPhaseLayerInterface {
   public:
      virtual JPH::uint GetNumBroadPhaseLayers() const override {
         return JoltBroadPhaseLayers::NUM_LAYERS;
      }
      virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer un_layer) const override {
         return un_layer == JoltLayers::NON_MOVING ?
            JoltBroadPhaseLayers::NON_MOVING : JoltBroadPhaseLayers::MOVING;
      }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
      virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer c_layer) const override {
         return c_layer == JoltBroadPhaseLayers::NON_MOVING ? "NON_MOVING" : "MOVING";
      }
#endif
   };

   class CJoltObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
   public:
      virtual bool ShouldCollide(JPH::ObjectLayer un_layer1,
                                 JPH::BroadPhaseLayer c_layer2) const override {
         /* Two static objects never collide */
         return un_layer1 == JoltLayers::MOVING ||
                c_layer2 == JoltBroadPhaseLayers::MOVING;
      }
   };

   class CJoltObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter {
   public:
      virtual bool ShouldCollide(JPH::ObjectLayer un_layer1,
                                 JPH::ObjectLayer un_layer2) const override {
         return un_layer1 == JoltLayers::MOVING ||
                un_layer2 == JoltLayers::MOVING;
      }
   };

   /* The physics system keeps references to these; they are stateless
    * and shared by all engine instances */
   static CJoltBPLayerInterface m_cBPLayerInterface;
   static CJoltObjectVsBroadPhaseLayerFilter m_cObjectVsBroadPhaseLayerFilter;
   static CJoltObjectLayerPairFilter m_cObjectLayerPairFilter;

   /****************************************/
   /****************************************/

   /* One-time global Jolt initialization */
   static void JoltGlobalInit() {
      static bool bInitialized = false;
      if(!bInitialized) {
         JPH::RegisterDefaultAllocator();
         JPH::Factory::sInstance = new JPH::Factory();
         JPH::RegisterTypes();
         bInitialized = true;
      }
   }

   /****************************************/
   /****************************************/

   CJoltEngine::CJoltEngine() {}

   /****************************************/
   /****************************************/

   void CJoltEngine::Init(TConfigurationNode& t_tree) {
      /* Initialize the parent (parses id, iterations, boundaries) */
      CPhysicsEngine::Init(t_tree);
      JoltGlobalInit();
      GetNodeAttributeOrDefault(t_tree, "default_friction", m_fDefaultFriction, m_fDefaultFriction);
      GetNodeAttributeOrDefault(t_tree, "threads", m_unThreads, m_unThreads);
      UInt32 unMaxBodies = 16384;
      GetNodeAttributeOrDefault(t_tree, "max_bodies", unMaxBodies, unMaxBodies);
      /* The contact constraint buffer lives in the temp allocator;
       * 32 MB fits the 10240 constraints below with ample margin */
      m_ptrTempAllocator = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);
      if(m_unThreads <= 1) {
         /* Single-threaded stepping is deterministic across runs */
         m_ptrJobSystem = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
      }
      else {
         m_ptrJobSystem = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, int(m_unThreads) - 1);
      }
      m_ptrSystem = std::make_unique<JPH::PhysicsSystem>();
      m_ptrSystem->Init(unMaxBodies, 0, unMaxBodies, 10240,
                        m_cBPLayerInterface,
                        m_cObjectVsBroadPhaseLayerFilter,
                        m_cObjectLayerPairFilter);
      /* No gravity by default; add the <gravity> plugin to enable it */
      m_ptrSystem->SetGravity(JPH::Vec3::sZero());
      /* Load the plugins */
      TConfigurationNodeIterator tPluginIterator;
      for(tPluginIterator = tPluginIterator.begin(&t_tree);
          tPluginIterator != tPluginIterator.end();
          ++tPluginIterator) {
         CJoltPlugin* pcPlugin = CFactory<CJoltPlugin>::New(tPluginIterator->Value());
         pcPlugin->SetEngine(*this);
         pcPlugin->Init(*tPluginIterator);
         AddPhysicsPlugin(tPluginIterator->Value(), *pcPlugin);
      }
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::PostSpaceInit() {
      /* All initial bodies are in the world at this point */
      m_ptrSystem->OptimizeBroadPhase();
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::Reset() {
      for(auto& tModel : m_tPhysicsModels) {
         tModel.second->Reset();
      }
      for(auto& tPlugin : m_tPhysicsPlugins) {
         tPlugin.second->Reset();
      }
      m_ptrSystem->OptimizeBroadPhase();
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::Destroy() {
      /* Destroy all physics models */
      for(auto& tModel : m_tPhysicsModels) {
         for(auto& tPlugin : m_tPhysicsPlugins) {
            tPlugin.second->UnregisterModel(*tModel.second);
         }
         delete tModel.second;
      }
      /* Destroy all plugins */
      for(auto& tPlugin : m_tPhysicsPlugins) {
         tPlugin.second->Destroy();
         delete tPlugin.second;
      }
      m_tPhysicsModels.clear();
      m_tPhysicsPlugins.clear();
      m_ptrSystem.reset();
      m_ptrJobSystem.reset();
      m_ptrTempAllocator.reset();
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::Update() {
      /* Update the physics state from the entities */
      for(auto& tModel : m_tPhysicsModels) {
         tModel.second->UpdateFromEntityStatus();
      }
      /* Step the simulation forwards */
      for(UInt32 i = 0; i < GetIterations(); ++i) {
         for(auto& tModel : m_tPhysicsModels) {
            tModel.second->UpdatePhysics();
         }
         for(auto& tPlugin : m_tPhysicsPlugins) {
            tPlugin.second->Update();
         }
         JPH::EPhysicsUpdateError eError =
            m_ptrSystem->Update(float(GetPhysicsClockTick()), 1,
                                m_ptrTempAllocator.get(),
                                m_ptrJobSystem.get());
         if(eError != JPH::EPhysicsUpdateError::None) {
            THROW_ARGOSEXCEPTION("Jolt physics update error in engine \""
                                 << GetId() << "\" (code "
                                 << static_cast<UInt32>(eError) << ")");
         }
      }
      /* Update the simulated space */
      for(auto& tModel : m_tPhysicsModels) {
         tModel.second->UpdateEntityStatus();
      }
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::CheckIntersectionWithRay(TEmbodiedEntityIntersectionData& t_data,
                                              const CRay3& c_ray) const {
      CVector3 cDirection;
      c_ray.GetDirection(cDirection);
      cDirection *= c_ray.GetLength();
      JPH::RRayCast cRayCast(ToJolt(c_ray.GetStart()), ToJolt(cDirection));
      JPH::RayCastResult sHit;
      if(m_ptrSystem->GetNarrowPhaseQuery().CastRay(cRayCast, sHit)) {
         /* Bodies without user data (e.g. the floor) are not models */
         JPH::uint64 unUserData =
            m_ptrSystem->GetBodyInterfaceNoLock().GetUserData(sHit.mBodyID);
         if(unUserData != 0) {
            auto* pcModel = reinterpret_cast<CJoltModel*>(unUserData);
            t_data.push_back(
               SEmbodiedEntityIntersectionItem(&pcModel->GetEmbodiedEntity(),
                                               sHit.mFraction));
         }
      }
   }

   /****************************************/
   /****************************************/

   size_t CJoltEngine::GetNumPhysicsModels() {
      return m_tPhysicsModels.size();
   }

   /****************************************/
   /****************************************/

   bool CJoltEngine::AddEntity(CEntity& c_entity) {
      SOperationOutcome cOutcome =
         CallEntityOperation<CJoltOperationAddEntity, CJoltEngine, SOperationOutcome>
         (*this, c_entity);
      return cOutcome.Value;
   }

   /****************************************/
   /****************************************/

   bool CJoltEngine::RemoveEntity(CEntity& c_entity) {
      SOperationOutcome cOutcome =
         CallEntityOperation<CJoltOperationRemoveEntity, CJoltEngine, SOperationOutcome>
         (*this, c_entity);
      return cOutcome.Value;
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::AddPhysicsModel(const std::string& str_id,
                                     CJoltModel& c_model) {
      for(auto& tPlugin : m_tPhysicsPlugins) {
         tPlugin.second->RegisterModel(c_model);
      }
      m_tPhysicsModels[str_id] = &c_model;
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::RemovePhysicsModel(const std::string& str_id) {
      auto itModel = m_tPhysicsModels.find(str_id);
      if(itModel != m_tPhysicsModels.end()) {
         for(auto& tPlugin : m_tPhysicsPlugins) {
            tPlugin.second->UnregisterModel(*itModel->second);
         }
         delete itModel->second;
         m_tPhysicsModels.erase(itModel);
      }
      else {
         THROW_ARGOSEXCEPTION("The model \"" << str_id <<
                              "\" was not found in the Jolt engine \"" <<
                              GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::AddPhysicsPlugin(const std::string& str_id,
                                      CJoltPlugin& c_plugin) {
      m_tPhysicsPlugins[str_id] = &c_plugin;
   }

   /****************************************/
   /****************************************/

   void CJoltEngine::RemovePhysicsPlugin(const std::string& str_id) {
      auto it = m_tPhysicsPlugins.find(str_id);
      if(it != m_tPhysicsPlugins.end()) {
         delete it->second;
         m_tPhysicsPlugins.erase(it);
      }
      else {
         THROW_ARGOSEXCEPTION("The plugin \"" << str_id <<
                              "\" was not found in the Jolt engine \"" <<
                              GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_PHYSICS_ENGINE(CJoltEngine,
                           "jolt",
                           "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                           "1.0",
                           "A 3D dynamics physics engine based on Jolt Physics",
                           "This physics engine is a 3D dynamics engine based on Jolt Physics\n"
                           "(https://github.com/jrouwe/JoltPhysics), the physics engine used by\n"
                           "Godot 4 and many commercial games.\n\n"

                           "REQUIRED XML CONFIGURATION\n\n"
                           "  <physics_engines>\n"
                           "    ...\n"
                           "    <jolt id=\"jolt\" />\n"
                           "    ...\n"
                           "  </physics_engines>\n\n"

                           "The 'id' attribute is necessary and must be unique among the physics engines.\n\n"

                           "OPTIONAL XML CONFIGURATION\n\n"

                           "The 'iterations' attribute (default 10) sets how many sub-steps are\n"
                           "performed per simulation tick. The 'threads' attribute (default 1)\n"
                           "sets the number of threads used to step the physics; with the default\n"
                           "of 1 the simulation is deterministic across runs and platforms.\n"
                           "The 'default_friction' attribute (default 1.0) sets the friction\n"
                           "coefficient assigned to created bodies.\n\n"

                           "Like the dynamics3d engine, features are added through plugins\n"
                           "declared as child tags. The <floor> plugin adds a static floor at the\n"
                           "given height (default 0); the <gravity> plugin enables gravity with\n"
                           "downward acceleration 'g' (default 9.81). Without the <gravity>\n"
                           "plugin, bodies float.\n\n"

                           "  <physics_engines>\n"
                           "    ...\n"
                           "    <jolt id=\"jolt\" iterations=\"10\" threads=\"1\">\n"
                           "      <floor height=\"0\" friction=\"1.0\" />\n"
                           "      <gravity g=\"9.81\" />\n"
                           "    </jolt>\n"
                           "    ...\n"
                           "  </physics_engines>\n\n",

                           "Usable"
      );

   /****************************************/
   /****************************************/

}
