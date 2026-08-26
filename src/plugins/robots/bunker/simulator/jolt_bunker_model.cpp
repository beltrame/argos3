/**
 * @file <argos3/plugins/robots/bunker/simulator/jolt_bunker_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_bunker_model.h"
#include "bunker_entity.h"

#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CJoltBunkerModel::BUNKER_LENGTH      = 1.023f;
   const Real CJoltBunkerModel::BUNKER_WIDTH       = 0.778f;
   const Real CJoltBunkerModel::BUNKER_HEIGHT      = 0.380f;
   const Real CJoltBunkerModel::BUNKER_MASS        = 170.0f;
   const Real CJoltBunkerModel::BUNKER_TRACK_GAUGE = 0.620f;

   /****************************************/
   /****************************************/

   CJoltBunkerModel::CJoltBunkerModel(CJoltEngine& c_engine,
                                              CBunkerEntity& c_entity) :
      CJoltSingleBodyObjectModel(c_engine, c_entity),
      m_cBunkerEntity(c_entity),
      m_cWheeledEntity(c_entity.GetWheeledEntity()) {
      /* Register auxiliary anchors */
      RegisterAnchorMethod<CJoltBunkerModel>(
         GetEmbodiedEntity().GetAnchor("body"),
         &CJoltBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerModel>(
         GetEmbodiedEntity().GetAnchor("left_tracks"),
         &CJoltBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerModel>(
         GetEmbodiedEntity().GetAnchor("right_tracks"),
         &CJoltBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerModel>(
         GetEmbodiedEntity().GetAnchor("lidar"),
         &CJoltBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerModel>(
         GetEmbodiedEntity().GetAnchor("camera"),
         &CJoltBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod<CJoltBunkerModel>(
         GetEmbodiedEntity().GetAnchor("imu"),
         &CJoltBunkerModel::UpdateAuxiliaryAnchor);

      /* The body is a box standing on the origin anchor */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(BUNKER_HEIGHT) * 0.5f);
      SAnchor& sAnchor = GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         new JPH::BoxShape(JPH::Vec3(float(BUNKER_LENGTH) * 0.5f,
                                     float(BUNKER_WIDTH) * 0.5f,
                                     float(BUNKER_HEIGHT) * 0.5f)),
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
      cSettings.mMassPropertiesOverride.mMass = float(BUNKER_MASS);
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltBunkerModel::UpdateFromEntityStatus() {
      const Real* pfWheelVelocities = m_cWheeledEntity.GetWheelVelocities();
      Real fLinear = (pfWheelVelocities[0] + pfWheelVelocities[1]) * 0.5f;
      Real fAngular = (pfWheelVelocities[1] - pfWheelVelocities[0]) / BUNKER_TRACK_GAUGE;

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

   void CJoltBunkerModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CBunkerEntity, CJoltBunkerModel);

}

