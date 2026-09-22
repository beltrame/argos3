/**
 * @file <argos3/plugins/robots/scout-mini/simulator/jolt_scout_mini_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_scout_mini_model.h"
#include "scout_mini_entity.h"

#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CJoltScoutMiniModel::SCOUT_MINI_LENGTH      = 0.612f;
   const Real CJoltScoutMiniModel::SCOUT_MINI_WIDTH       = 0.580f;
   const Real CJoltScoutMiniModel::SCOUT_MINI_HEIGHT      = 0.245f;
   const Real CJoltScoutMiniModel::SCOUT_MINI_MASS        = 25.0f;
   const Real CJoltScoutMiniModel::SCOUT_MINI_TRACK_GAUGE = 0.450f;

   /****************************************/
   /****************************************/

   CJoltScoutMiniModel::CJoltScoutMiniModel(CJoltEngine& c_engine,
                                              CScoutMiniEntity& c_entity) :
      CJoltGroundRobotModel(c_engine, c_entity),
      m_cScoutMiniEntity(c_entity),
      m_cWheeledEntity(c_entity.GetWheeledEntity()) {
      /* Register auxiliary anchors */
      RegisterAnchorMethod<CJoltScoutMiniModel>(
         GetEmbodiedEntity().GetAnchor("body"),
         &CJoltScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltScoutMiniModel>(
         GetEmbodiedEntity().GetAnchor("left_wheels"),
         &CJoltScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltScoutMiniModel>(
         GetEmbodiedEntity().GetAnchor("right_wheels"),
         &CJoltScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltScoutMiniModel>(
         GetEmbodiedEntity().GetAnchor("lidar"),
         &CJoltScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltScoutMiniModel>(
         GetEmbodiedEntity().GetAnchor("camera"),
         &CJoltScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltScoutMiniModel>(
         GetEmbodiedEntity().GetAnchor("imu"),
         &CJoltScoutMiniModel::UpdateAuxiliaryAnchor);

      /* The body is a box standing on the origin anchor */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(SCOUT_MINI_HEIGHT) * 0.5f);
      SAnchor& sAnchor = GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         new JPH::BoxShape(JPH::Vec3(float(SCOUT_MINI_LENGTH) * 0.5f,
                                     float(SCOUT_MINI_WIDTH) * 0.5f,
                                     float(SCOUT_MINI_HEIGHT) * 0.5f)),
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
      cSettings.mMassPropertiesOverride.mMass = float(SCOUT_MINI_MASS);
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltScoutMiniModel::UpdateFromEntityStatus() {
      const Real* pfWheelVelocities = m_cWheeledEntity.GetWheelVelocities();
      Real fLinear = (pfWheelVelocities[0] + pfWheelVelocities[1]) * 0.5f;
      Real fAngular = (pfWheelVelocities[1] - pfWheelVelocities[0]) / SCOUT_MINI_TRACK_GAUGE;

      SetDriveVelocity(fLinear, fAngular);
   }

   /****************************************/
   /****************************************/

   void CJoltScoutMiniModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CScoutMiniEntity, CJoltScoutMiniModel);

}

