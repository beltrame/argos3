/**
 * @file <argos3/plugins/robots/foot-bot/simulator/jolt_footbot_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_footbot_model.h"
#include "footbot_entity.h"
#include "footbot_turret_entity.h"

#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* Same dimensions as the dynamics2d foot-bot model */
   static const Real FOOTBOT_RADIUS              = 0.085036758f;
   static const Real FOOTBOT_INTERWHEEL_DISTANCE = 0.14f;
   static const Real FOOTBOT_HEIGHT              = 0.146899733f;
   static const Real FOOTBOT_MASS                = 1.6f;

   enum FOOTBOT_WHEELS {
      FOOTBOT_LEFT_WHEEL = 0,
      FOOTBOT_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CJoltFootBotModel::CJoltFootBotModel(CJoltEngine& c_engine,
                                        CFootBotEntity& c_entity) :
      CJoltSingleBodyObjectModel(c_engine, c_entity),
      m_cFootBotEntity(c_entity),
      m_cWheeledEntity(c_entity.GetWheeledEntity()) {
      RegisterAnchorMethod<CJoltFootBotModel>(
         GetEmbodiedEntity().GetAnchor("turret"),
         &CJoltFootBotModel::UpdateTurretAnchor);
      RegisterAnchorMethod<CJoltFootBotModel>(
         GetEmbodiedEntity().GetAnchor("perspective_camera"),
         &CJoltFootBotModel::UpdatePerspectiveCameraAnchor);
      /* The body is a cylinder standing on the origin anchor */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(FOOTBOT_HEIGHT) * 0.5f);
      SAnchor& sAnchor = GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         CJoltShapeManager::RequestCylinder(float(FOOTBOT_HEIGHT) * 0.5f,
                                            float(FOOTBOT_RADIUS)),
         cPosition, cRotation,
         JPH::EMotionType::Dynamic,
         JoltLayers::MOVING);
      /* The wheel velocities are applied as body velocities every
       * tick; ground friction and damping would fight them */
      cSettings.mFriction = 0.0f;
      cSettings.mLinearDamping = 0.0f;
      cSettings.mAngularDamping = 0.0f;
      /* A differential-drive robot only translates and yaws */
      cSettings.mAllowedDOFs = JPH::EAllowedDOFs::TranslationX |
                               JPH::EAllowedDOFs::TranslationY |
                               JPH::EAllowedDOFs::TranslationZ |
                               JPH::EAllowedDOFs::RotationZ;
      /* No tunneling through thin walls */
      cSettings.mMotionQuality = JPH::EMotionQuality::LinearCast;
      /* Velocities are set every tick; never sleep */
      cSettings.mAllowSleeping = false;
      cSettings.mOverrideMassProperties =
         JPH::EOverrideMassProperties::CalculateInertia;
      cSettings.mMassPropertiesOverride.mMass = float(FOOTBOT_MASS);
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      /* Finalize the model */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltFootBotModel::UpdateFromEntityStatus() {
      const Real* pfWheelVelocities = m_cWheeledEntity.GetWheelVelocities();
      Real fLinear = (pfWheelVelocities[FOOTBOT_LEFT_WHEEL] +
                      pfWheelVelocities[FOOTBOT_RIGHT_WHEEL]) * 0.5;
      Real fAngular = (pfWheelVelocities[FOOTBOT_RIGHT_WHEEL] -
                       pfWheelVelocities[FOOTBOT_LEFT_WHEEL]) /
                      FOOTBOT_INTERWHEEL_DISTANCE;
      JPH::BodyInterface& cInterface = GetJoltEngine().GetBodyInterface();
      const JPH::BodyID& cId = m_vecBodies[0].Id;
      JPH::RVec3 cPosition;
      JPH::Quat cRotation;
      cInterface.GetPositionAndRotation(cId, cPosition, cRotation);
      JPH::Vec3 cForward = cRotation * JPH::Vec3::sAxisX();
      /* Keep the vertical velocity: gravity holds the robot down */
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

   void CJoltFootBotModel::UpdateTurretAnchor(SAnchor& s_anchor) {
      const SAnchor& sOrigin = GetEmbodiedEntity().GetOriginAnchor();
      CVector3 cOffset = s_anchor.OffsetPosition;
      cOffset.Rotate(sOrigin.Orientation);
      s_anchor.Position = sOrigin.Position + cOffset;
      CQuaternion cTurretRotation(
         m_cFootBotEntity.GetTurretEntity().GetRotation(), CVector3::Z);
      s_anchor.Orientation = sOrigin.Orientation * cTurretRotation;
      s_anchor.OffsetOrientation = cTurretRotation;
   }

   /****************************************/
   /****************************************/

   void CJoltFootBotModel::UpdatePerspectiveCameraAnchor(SAnchor& s_anchor) {
      const SAnchor& sOrigin = GetEmbodiedEntity().GetOriginAnchor();
      CVector3 cOffset = s_anchor.OffsetPosition;
      cOffset.Rotate(sOrigin.Orientation);
      s_anchor.Position = sOrigin.Position + cOffset;
      s_anchor.Orientation = sOrigin.Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CFootBotEntity, CJoltFootBotModel);

   /****************************************/
   /****************************************/

}
