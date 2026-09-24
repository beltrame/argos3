/**
 * @file <argos3/plugins/robots/spot/simulator/jolt_spot_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_common.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
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
      CJoltGroundRobotModel(c_engine, c_entity),
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
      cSettings.mMassPropertiesOverride.mMass = float(SPOT_MASS);
      const JPH::BodyID cId = CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      /* Active balance approximation: constrain swing only, never position.
       * Constraint X is world/body up, so its free twist axis is chassis yaw.
       * All translation axes remain free: unsupported bodies still fall. */
      JPH::SixDOFConstraintSettings cBalance;
      cBalance.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
      cBalance.mSwingType = JPH::ESwingType::Cone;
      cBalance.mAxisX1 = cBalance.mAxisX2 = JPH::Vec3::sAxisZ();
      cBalance.mAxisY1 = cBalance.mAxisY2 = JPH::Vec3::sAxisX();
      cBalance.SetLimitedAxis(JPH::SixDOFConstraintSettings::RotationY, -0.523598776f, 0.523598776f);
      cBalance.SetLimitedAxis(JPH::SixDOFConstraintSettings::RotationZ, -0.523598776f, 0.523598776f);
      cBalance.mNumPositionStepsOverride = 12;
      m_pcBalance = c_engine.GetBodyInterface().CreateConstraint(&cBalance, JPH::BodyID(), cId);
      c_engine.GetSystem().AddConstraint(m_pcBalance);
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   CJoltSpotModel::~CJoltSpotModel() {
      GetJoltEngine().GetSystem().RemoveConstraint(m_pcBalance);
   }

   void CJoltSpotModel::Reset() {
      CJoltGroundRobotModel::Reset();
      ResetMotionState();
   }

   void CJoltSpotModel::MoveTo(const CVector3& c_position, const CQuaternion& c_orientation) {
      CJoltGroundRobotModel::MoveTo(c_position, c_orientation);
      ResetMotionState();
   }

   void CJoltSpotModel::ResetMotionState() {
      m_pcBalance->ResetWarmStart();
      SetDriveVelocity(0.0, 0.0);
      m_eStepPhase = EStepPhase::NONE;
      m_cStepTarget = JPH::RVec3::sZero();
      m_cStepDirection = JPH::Vec3::sZero();
      m_bStepPaused = false;
      m_fStepHoldHeight = 0.0f;
      m_fStepTimeLeft = m_fStepPauseTimeLeft = m_fStepCooldown = 0.0f;
      m_fCommandLinear = m_fCommandAngular = 0.0f;
   }

   /****************************************/
   /****************************************/

   void CJoltSpotModel::UpdateFromEntityStatus() {
      const Real* pfWheelVelocities = m_cWheeledEntity.GetWheelVelocities();
      Real fLinear = (pfWheelVelocities[0] + pfWheelVelocities[1]) * 0.5f;
      Real fAngular = (pfWheelVelocities[1] - pfWheelVelocities[0]) / SPOT_TRACK_GAUGE;

      m_fCommandLinear = float(fLinear);
      m_fCommandAngular = float(fAngular);
      SetDriveVelocity(fLinear, fAngular);
      if(m_eStepPhase == EStepPhase::NONE && m_fStepCooldown <= 0.0f) TryStartStep();
   }

   /****************************************/
   /****************************************/

   CJoltModel::SContactSurfaceVelocity CJoltSpotModel::GetContactSurfaceVelocity(
      const JPH::Body& c_body,
      JPH::Vec3Arg c_support_normal,
      JPH::RVec3Arg c_contact_offset,
      const JPH::ContactPoints& c_contact_points) const {
      const JPH::Quat cRotation = c_body.GetRotation();
      /* The lower part of Spot's box approximates feet and lower legs, not
       * articulated limbs. Keep rounded/edge support on rough terrain: an
       * angle cutoff pins this solid leg-volume proxy against small rocks. */
      if(c_support_normal.Dot(cRotation * JPH::Vec3::sAxisZ()) <= 0.0f ||
         c_contact_points.empty()) return {};
      const JPH::Quat cInverse = cRotation.Conjugated();
      for(const JPH::Vec3& cPoint : c_contact_points) {
         const JPH::Vec3 cLocal = cInverse * JPH::Vec3(
            c_contact_offset + cPoint - c_body.GetCenterOfMassPosition());
         /* Only the bottom 12 cm acts as the foot/lower-leg drive envelope;
          * torso contacts stay passive. A manifold shares one surface velocity,
          * so every point must qualify (never drive a whole side from one toe). */
         if(std::abs(cLocal.GetZ() + float(SPOT_HEIGHT) * 0.5f) > 0.12f) return {};
      }
      auto sVelocity = CJoltGroundRobotModel::GetContactSurfaceVelocity(
         c_body, c_support_normal, c_contact_offset, c_contact_points);
      /* Keep traction on steep lower-leg contacts, but never motor up a wall.
       * Classify in world axes: a wall must stay steep even as the body pitches.
       * 40 degrees separates the measured 16/18-degree ramps from rock edges.
       * World-Z yaw also keeps angular surface motion horizontal at every point. */
      if(c_support_normal.GetZ() < STEEP_CONTACT_NORMAL_Z) {
         sVelocity.Linear.SetZ(0.0f);
         sVelocity.Angular = JPH::Vec3(0.0f, 0.0f, sVelocity.Angular.GetZ());
      }
      return sVelocity;
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

