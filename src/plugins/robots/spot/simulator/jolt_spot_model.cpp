/**
 * @file <argos3/plugins/robots/spot/simulator/jolt_spot_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_spot_model.h"
#include "spot_entity.h"

#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CJoltSpotModel::SPOT_LENGTH      = 1.100f;
   const Real CJoltSpotModel::SPOT_WIDTH       = 0.500f;
   const Real CJoltSpotModel::SPOT_HEIGHT      = 0.620f;
   const Real CJoltSpotModel::SPOT_MASS        = 32.7f;
   const Real CJoltSpotModel::SPOT_TRACK_GAUGE = 0.500f;

   /****************************************/
   /****************************************/

   CJoltSpotModel::CJoltSpotModel(CJoltEngine& c_engine,
                                              CSpotEntity& c_entity) :
      CJoltSingleBodyObjectModel(c_engine, c_entity),
      m_cSpotEntity(c_entity),
      m_cWheeledEntity(c_entity.GetWheeledEntity()) {
      /* Register auxiliary anchors */
      RegisterAnchorMethod<CJoltSpotModel>(
         GetEmbodiedEntity().GetAnchor("body"),
         &CJoltSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltSpotModel>(
         GetEmbodiedEntity().GetAnchor("left_legs"),
         &CJoltSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltSpotModel>(
         GetEmbodiedEntity().GetAnchor("right_legs"),
         &CJoltSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltSpotModel>(
         GetEmbodiedEntity().GetAnchor("lidar"),
         &CJoltSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltSpotModel>(
         GetEmbodiedEntity().GetAnchor("camera"),
         &CJoltSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltSpotModel>(
         GetEmbodiedEntity().GetAnchor("imu"),
         &CJoltSpotModel::UpdateAuxiliaryAnchor);

      /* The body is a box standing on the origin anchor */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(SPOT_HEIGHT) * 0.5f);
      SAnchor& sAnchor = GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         new JPH::BoxShape(JPH::Vec3(float(SPOT_LENGTH) * 0.5f,
                                     float(SPOT_WIDTH) * 0.5f,
                                     float(SPOT_HEIGHT) * 0.5f)),
         cPosition, cRotation,
         JPH::EMotionType::Dynamic,
         JoltLayers::MOVING);
      cSettings.mFriction = 0.0f;
      cSettings.mLinearDamping = 0.0f;
      cSettings.mAngularDamping = 0.0f;
      /* Planar differential/skid drive: translates in X/Y/Z and yaws */
      cSettings.mAllowedDOFs = JPH::EAllowedDOFs::TranslationX |
                               JPH::EAllowedDOFs::TranslationY |
                               JPH::EAllowedDOFs::TranslationZ |
                               JPH::EAllowedDOFs::RotationZ;
      cSettings.mMotionQuality = JPH::EMotionQuality::LinearCast;
      cSettings.mAllowSleeping = false;
      cSettings.mOverrideMassProperties =
         JPH::EOverrideMassProperties::CalculateInertia;
      cSettings.mMassPropertiesOverride.mMass = float(SPOT_MASS);
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltSpotModel::UpdateFromEntityStatus() {
      const Real* pfWheelVelocities = m_cWheeledEntity.GetWheelVelocities();
      Real fLinear = (pfWheelVelocities[0] + pfWheelVelocities[1]) * 0.5f;
      Real fAngular = (pfWheelVelocities[1] - pfWheelVelocities[0]) / SPOT_TRACK_GAUGE;

      JPH::BodyInterface& cInterface = GetJoltEngine().GetBodyInterface();
      const JPH::BodyID& cId = m_vecBodies[0].Id;
      JPH::RVec3 cPosition;
      JPH::Quat cRotation;
      cInterface.GetPositionAndRotation(cId, cPosition, cRotation);
      JPH::Vec3 cForward = cRotation * JPH::Vec3::sAxisX();

      /* Maintain vertical velocity from gravity */
      float fVerticalVelocity = cInterface.GetLinearVelocity(cId).GetZ();
      cInterface.SetLinearAndAngularVelocity(
         cId,
         JPH::Vec3(cForward.GetX() * float(fLinear),
                   cForward.GetY() * float(fLinear),
                   fVerticalVelocity),
         JPH::Vec3(0.0f, 0.0f, float(fAngular)));
   }

   /****************************************/
   /****************************************/

   void CJoltSpotModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CSpotEntity, CJoltSpotModel);

}

