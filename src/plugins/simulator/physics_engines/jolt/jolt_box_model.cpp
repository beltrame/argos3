/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_box_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_box_model.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   CJoltBoxModel::CJoltBoxModel(CJoltEngine& c_engine,
                                CBoxEntity& c_box) :
      CJoltSingleBodyObjectModel(c_engine, c_box) {
      bool bMovable = c_box.GetEmbodiedEntity().IsMovable();
      /* The ARGoS origin anchor sits at the bottom center of the box;
       * the Jolt body at its geometric center */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(c_box.GetSize().GetZ()) * 0.5f);
      SAnchor& sAnchor = c_box.GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         CJoltShapeManager::RequestBox(ToJolt(c_box.GetSize() * 0.5)),
         cPosition, cRotation,
         bMovable ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
         bMovable ? JoltLayers::MOVING : JoltLayers::NON_MOVING);
      cSettings.mFriction = c_engine.GetDefaultFriction();
      if(bMovable) {
         cSettings.mOverrideMassProperties =
            JPH::EOverrideMassProperties::CalculateInertia;
         cSettings.mMassPropertiesOverride.mMass = float(c_box.GetMass());
      }
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      /* Finalize the model */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CBoxEntity, CJoltBoxModel);

   /****************************************/
   /****************************************/

}
