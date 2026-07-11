/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_engine.h>
 *
 * A 3D dynamics physics engine based on Jolt Physics
 * (https://github.com/jrouwe/JoltPhysics). The engine works in the
 * ARGoS coordinate system directly (z up); no axis conversion is
 * needed between ARGoS and Jolt quantities.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_ENGINE_H
#define JOLT_ENGINE_H

namespace argos {
   class CJoltEngine;
   class CJoltModel;
   class CJoltPlugin;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_common.h>

#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

#include <map>
#include <memory>

namespace argos {

   /****************************************/
   /****************************************/

   /* ARGoS <-> Jolt math conversions (same axis conventions) */

   inline JPH::Vec3 ToJolt(const CVector3& c_vector) {
      return JPH::Vec3(float(c_vector.GetX()),
                       float(c_vector.GetY()),
                       float(c_vector.GetZ()));
   }

   inline JPH::Quat ToJolt(const CQuaternion& c_quaternion) {
      return JPH::Quat(float(c_quaternion.GetX()),
                       float(c_quaternion.GetY()),
                       float(c_quaternion.GetZ()),
                       float(c_quaternion.GetW()));
   }

   inline CVector3 ToARGoS(JPH::Vec3Arg c_vector) {
      return CVector3(c_vector.GetX(), c_vector.GetY(), c_vector.GetZ());
   }

   inline CQuaternion ToARGoS(JPH::QuatArg c_quaternion) {
      return CQuaternion(c_quaternion.GetW(),
                         c_quaternion.GetX(),
                         c_quaternion.GetY(),
                         c_quaternion.GetZ());
   }

   /****************************************/
   /****************************************/

   /* Object layers: static environment vs simulated entities */
   namespace JoltLayers {
      static constexpr JPH::ObjectLayer NON_MOVING = 0;
      static constexpr JPH::ObjectLayer MOVING = 1;
      static constexpr JPH::uint NUM_LAYERS = 2;
   }

   /****************************************/
   /****************************************/

   class CJoltEngine : public CPhysicsEngine {

   public:

      CJoltEngine();

      virtual ~CJoltEngine() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Update();

      virtual void Destroy();

      virtual void PostSpaceInit();

      virtual size_t GetNumPhysicsModels();

      virtual bool AddEntity(CEntity& c_entity);

      virtual bool RemoveEntity(CEntity& c_entity);

      virtual void CheckIntersectionWithRay(TEmbodiedEntityIntersectionData& t_data,
                                            const CRay3& c_ray) const;

      inline JPH::PhysicsSystem& GetSystem() {
         return *m_ptrSystem;
      }

      inline const JPH::PhysicsSystem& GetSystem() const {
         return *m_ptrSystem;
      }

      inline JPH::BodyInterface& GetBodyInterface() {
         return m_ptrSystem->GetBodyInterface();
      }

      inline std::map<std::string, CJoltPlugin*>& GetPhysicsPlugins() {
         return m_tPhysicsPlugins;
      }

      inline float GetDefaultFriction() const {
         return m_fDefaultFriction;
      }

      void AddPhysicsModel(const std::string& str_id,
                           CJoltModel& c_model);

      void RemovePhysicsModel(const std::string& str_id);

      void AddPhysicsPlugin(const std::string& str_id,
                            CJoltPlugin& c_plugin);

      void RemovePhysicsPlugin(const std::string& str_id);

   private:

      /* Maps of models and plugins */
      std::map<std::string, CJoltModel*> m_tPhysicsModels;
      std::map<std::string, CJoltPlugin*> m_tPhysicsPlugins;
      /* Default friction for created bodies */
      float m_fDefaultFriction = 1.0f;
      /* Number of worker threads (1 = single-threaded, deterministic) */
      UInt32 m_unThreads = 1;
      /* Jolt world data; the broad-phase interface objects are
       * defined in the .cpp and referenced by the physics system */
      std::unique_ptr<JPH::TempAllocatorImpl> m_ptrTempAllocator;
      std::unique_ptr<JPH::JobSystem> m_ptrJobSystem;
      std::unique_ptr<JPH::PhysicsSystem> m_ptrSystem;

   };

   /****************************************/
   /****************************************/

   template <typename ACTION>
   class CJoltOperation : public CEntityOperation<ACTION, CJoltEngine, SOperationOutcome> {
   public:
      virtual ~CJoltOperation() {}
   };

   class CJoltOperationAddEntity : public CJoltOperation<CJoltOperationAddEntity> {
   public:
      virtual ~CJoltOperationAddEntity() {}
   };

   class CJoltOperationRemoveEntity : public CJoltOperation<CJoltOperationRemoveEntity> {
   public:
      virtual ~CJoltOperationRemoveEntity() {}
   };

#define REGISTER_JOLT_OPERATION(ACTION, OPERATION, ENTITY)              \
   REGISTER_ENTITY_OPERATION(ACTION, CJoltEngine, OPERATION, SOperationOutcome, ENTITY);

#define REGISTER_STANDARD_JOLT_OPERATION_ADD_ENTITY(SPACE_ENTITY, JOLT_MODEL) \
   class CJoltOperationAdd ## SPACE_ENTITY : public CJoltOperationAddEntity { \
   public:                                                              \
   CJoltOperationAdd ## SPACE_ENTITY() {}                               \
   virtual ~CJoltOperationAdd ## SPACE_ENTITY() {}                      \
   SOperationOutcome ApplyTo(CJoltEngine& c_engine,                     \
                SPACE_ENTITY& c_entity) {                               \
      auto* pcPhysModel = new JOLT_MODEL(c_engine,                      \
                                         c_entity);                     \
      c_engine.AddPhysicsModel(c_entity.GetId(),                        \
                               *pcPhysModel);                           \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         AddPhysicsModel(c_engine.GetId(), *pcPhysModel);               \
      return SOperationOutcome(true);                                   \
   }                                                                    \
   };                                                                   \
   REGISTER_JOLT_OPERATION(CJoltOperationAddEntity,                     \
                           CJoltOperationAdd ## SPACE_ENTITY,           \
                           SPACE_ENTITY);

#define REGISTER_STANDARD_JOLT_OPERATION_REMOVE_ENTITY(SPACE_ENTITY)    \
   class CJoltOperationRemove ## SPACE_ENTITY : public CJoltOperationRemoveEntity { \
   public:                                                              \
   CJoltOperationRemove ## SPACE_ENTITY() {}                            \
   virtual ~CJoltOperationRemove ## SPACE_ENTITY() {}                   \
   SOperationOutcome ApplyTo(CJoltEngine& c_engine,                     \
                SPACE_ENTITY& c_entity) {                               \
      c_engine.RemovePhysicsModel(c_entity.GetId());                    \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         RemovePhysicsModel(c_engine.GetId());                          \
      return SOperationOutcome(true);                                   \
   }                                                                    \
   };                                                                   \
   REGISTER_JOLT_OPERATION(CJoltOperationRemoveEntity,                  \
                           CJoltOperationRemove ## SPACE_ENTITY,        \
                           SPACE_ENTITY);

#define REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(SPACE_ENTITY, JOLT_MODEL) \
   REGISTER_STANDARD_JOLT_OPERATION_ADD_ENTITY(SPACE_ENTITY, JOLT_MODEL) \
   REGISTER_STANDARD_JOLT_OPERATION_REMOVE_ENTITY(SPACE_ENTITY)

   /****************************************/
   /****************************************/

}

#endif
