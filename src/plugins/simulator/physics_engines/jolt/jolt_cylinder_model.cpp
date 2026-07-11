/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_cylinder_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_cylinder_model.h"

#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   CJoltCylinderModel::CJoltCylinderModel(CJoltEngine& c_engine,
                                          CCylinderEntity& c_cylinder) :
      CJoltSingleBodyObjectModel(c_engine, c_cylinder) {
      bool bMovable = c_cylinder.GetEmbodiedEntity().IsMovable();
      /* The ARGoS origin anchor sits at the bottom center of the
       * cylinder; the Jolt body at its geometric center */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(c_cylinder.GetHeight()) * 0.5f);
      SAnchor& sAnchor = c_cylinder.GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         CJoltShapeManager::RequestCylinder(float(c_cylinder.GetHeight()) * 0.5f,
                                            float(c_cylinder.GetRadius())),
         cPosition, cRotation,
         bMovable ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
         bMovable ? JoltLayers::MOVING : JoltLayers::NON_MOVING);
      cSettings.mFriction = c_engine.GetDefaultFriction();
      if(bMovable) {
         cSettings.mOverrideMassProperties =
            JPH::EOverrideMassProperties::CalculateInertia;
         cSettings.mMassPropertiesOverride.mMass = float(c_cylinder.GetMass());
      }
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      /* Finalize the model */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CCylinderEntity, CJoltCylinderModel);

   /****************************************/
   /****************************************/

}
