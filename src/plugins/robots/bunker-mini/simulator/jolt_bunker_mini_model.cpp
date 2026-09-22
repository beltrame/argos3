/**
 * @file <argos3/plugins/robots/bunker-mini/simulator/jolt_bunker_mini_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_bunker_mini_model.h"
#include "bunker_mini_entity.h"

#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CJoltBunkerMiniModel::BUNKER_MINI_LENGTH      = 0.660f;
   const Real CJoltBunkerMiniModel::BUNKER_MINI_WIDTH       = 0.584f;
   const Real CJoltBunkerMiniModel::BUNKER_MINI_HEIGHT      = 0.281f;
   const Real CJoltBunkerMiniModel::BUNKER_MINI_MASS        = 55.0f;
   const Real CJoltBunkerMiniModel::BUNKER_MINI_TRACK_GAUGE = 0.412f;

   /****************************************/
   /****************************************/

   CJoltBunkerMiniModel::CJoltBunkerMiniModel(CJoltEngine& c_engine,
                                              CBunkerMiniEntity& c_entity) :
      CJoltGroundRobotModel(c_engine, c_entity),
      m_cBunkerMiniEntity(c_entity),
      m_cWheeledEntity(c_entity.GetWheeledEntity()) {
      /* Register auxiliary anchors */
      RegisterAnchorMethod<CJoltBunkerMiniModel>(
         GetEmbodiedEntity().GetAnchor("body"),
         &CJoltBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerMiniModel>(
         GetEmbodiedEntity().GetAnchor("left_track"),
         &CJoltBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerMiniModel>(
         GetEmbodiedEntity().GetAnchor("right_track"),
         &CJoltBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerMiniModel>(
         GetEmbodiedEntity().GetAnchor("lidar"),
         &CJoltBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerMiniModel>(
         GetEmbodiedEntity().GetAnchor("camera"),
         &CJoltBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerMiniModel>(
         GetEmbodiedEntity().GetAnchor("imu"),
         &CJoltBunkerMiniModel::UpdateAuxiliaryAnchor);

      /* The body is a box standing on the origin anchor */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(BUNKER_MINI_HEIGHT) * 0.5f);
      SAnchor& sAnchor = GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         new JPH::BoxShape(JPH::Vec3(float(BUNKER_MINI_LENGTH) * 0.5f,
                                     float(BUNKER_MINI_WIDTH) * 0.5f,
                                     float(BUNKER_MINI_HEIGHT) * 0.5f)),
         cPosition, cRotation,
         JPH::EMotionType::Dynamic,
         JoltLayers::MOVING);
      cSettings.mFriction = c_engine.GetDefaultFriction();
      cSettings.mEnhancedInternalEdgeRemoval = true;
      cSettings.mLinearDamping = 0.0f;
      cSettings.mAngularDamping = 0.0f;
      /* Differential/skid drive: translate in world X/Y/Z while retaining
       * contact-driven roll and pitch. */
      cSettings.mAllowedDOFs = JPH::EAllowedDOFs::TranslationX |
                               JPH::EAllowedDOFs::TranslationY |
                               JPH::EAllowedDOFs::TranslationZ |
                               JPH::EAllowedDOFs::RotationX |
                               JPH::EAllowedDOFs::RotationY |
                               JPH::EAllowedDOFs::RotationZ;
      cSettings.mMotionQuality = JPH::EMotionQuality::LinearCast;
      cSettings.mAllowSleeping = false;
      cSettings.mOverrideMassProperties =
         JPH::EOverrideMassProperties::CalculateInertia;
      cSettings.mMassPropertiesOverride.mMass = float(BUNKER_MINI_MASS);
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltBunkerMiniModel::UpdateFromEntityStatus() {
      const Real* pfWheelVelocities = m_cWheeledEntity.GetWheelVelocities();
      Real fLinear = (pfWheelVelocities[0] + pfWheelVelocities[1]) * 0.5f;
      Real fAngular = (pfWheelVelocities[1] - pfWheelVelocities[0]) / BUNKER_MINI_TRACK_GAUGE;

      SetDriveVelocity(fLinear, fAngular);
   }

   /****************************************/
   /****************************************/

   void CJoltBunkerMiniModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CBunkerMiniEntity, CJoltBunkerMiniModel);

}

