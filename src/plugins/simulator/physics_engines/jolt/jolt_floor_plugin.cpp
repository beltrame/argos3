/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_floor_plugin.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_floor_plugin.h"

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* The floor extends well beyond any reasonable arena */
   static const float FLOOR_HALF_SIDE = 1000.0f;
   static const float FLOOR_HALF_THICKNESS = 0.5f;

   void CJoltFloorPlugin::Init(TConfigurationNode& t_tree) {
      Real fHeight = 0.0;
      GetNodeAttributeOrDefault(t_tree, "height", fHeight, fHeight);
      Real fFriction = m_pcEngine->GetDefaultFriction();
      GetNodeAttributeOrDefault(t_tree, "friction", fFriction, fFriction);
      JPH::BodyCreationSettings cSettings(
         CJoltShapeManager::RequestBox(JPH::Vec3(FLOOR_HALF_SIDE,
                                                 FLOOR_HALF_SIDE,
                                                 FLOOR_HALF_THICKNESS)),
         JPH::RVec3(0.0f, 0.0f, float(fHeight) - FLOOR_HALF_THICKNESS),
         JPH::Quat::sIdentity(),
         JPH::EMotionType::Static,
         JoltLayers::NON_MOVING);
      cSettings.mFriction = float(fFriction);
      /* No user data: the floor is not an ARGoS model */
      cSettings.mUserData = 0;
      m_cFloorId = m_pcEngine->GetBodyInterface().CreateAndAddBody(
         cSettings, JPH::EActivation::DontActivate);
      if(m_cFloorId.IsInvalid()) {
         THROW_ARGOSEXCEPTION("Could not create the Jolt floor body");
      }
   }

   /****************************************/
   /****************************************/

   void CJoltFloorPlugin::Destroy() {
      if(!m_cFloorId.IsInvalid()) {
         JPH::BodyInterface& cInterface = m_pcEngine->GetBodyInterface();
         cInterface.RemoveBody(m_cFloorId);
         cInterface.DestroyBody(m_cFloorId);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_JOLT_PLUGIN(CJoltFloorPlugin,
                        "floor",
                        "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                        "1.0",
                        "Inserts a static floor into the Jolt engine",
                        "For a description on how to use this plugin, please consult the documentation\n"
                        "for the jolt physics engine plugin",
                        "Usable");

   /****************************************/
   /****************************************/

}
