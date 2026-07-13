/**
 * @file <argos3/plugins/robots/drone/simulator/jolt_drone_model.cpp>
 *
 * Flight controller math ported from pointmass3d_drone_model.cpp
 * (Sinan Oguz, Michael Allwright); instead of integrating the
 * dynamics manually, the thrust and torques are applied to a Jolt
 * body.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_drone_model.h"
#include "drone_entity.h"
#include "drone_flight_system_entity.h"

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>

#include <array>
#include <cmath>

namespace argos {

   /****************************************/
   /****************************************/

   CJoltDroneModel::CJoltDroneModel(CJoltEngine& c_engine,
                                    CDroneEntity& c_drone) :
      CJoltSingleBodyObjectModel(c_engine, c_drone),
      m_cFlightSystemEntity(c_drone.GetFlightSystemEntity()) {
      /* the body is a cylinder covering the rotor circle, standing on
       * the origin anchor */
      JPH::Vec3 cAnchorOffset(0.0f, 0.0f, float(HEIGHT) * 0.5f);
      SAnchor& sAnchor = GetEmbodiedEntity().GetOriginAnchor();
      JPH::Quat cRotation = ToJolt(sAnchor.Orientation);
      JPH::RVec3 cPosition = ToJolt(sAnchor.Position) + cRotation * cAnchorOffset;
      JPH::BodyCreationSettings cSettings(
         CJoltShapeManager::RequestCylinder(float(HEIGHT) * 0.5f,
                                            float(ARM_LENGTH)),
         cPosition, cRotation,
         JPH::EMotionType::Dynamic,
         JoltLayers::MOVING);
      cSettings.mFriction = c_engine.GetDefaultFriction();
      cSettings.mLinearDamping = 0.0f;
      /* mild rotational aero damping */
      cSettings.mAngularDamping = 0.5f;
      cSettings.mMotionQuality = JPH::EMotionQuality::LinearCast;
      /* forces are applied every sub-step; never sleep */
      cSettings.mAllowSleeping = false;
      cSettings.mOverrideMassProperties =
         JPH::EOverrideMassProperties::MassAndInertiaProvided;
      cSettings.mMassPropertiesOverride.mMass = float(MASS);
      cSettings.mMassPropertiesOverride.mInertia =
         JPH::Mat44::sScale(ToJolt(INERTIA));
      CreateBody(cSettings, &sAnchor, cAnchorOffset, JPH::Quat::sIdentity());
      /* initialize the home pose and the controller state */
      Reset();
   }

   /****************************************/
   /****************************************/

   void CJoltDroneModel::ResetControllerState() {
      m_cOrientationTargetPrev.Set(0.0, 0.0, 0.0);
      m_cAngularVelocityCumulativeError.Set(0.0, 0.0, 0.0);
      m_fAltitudeCumulativeError = 0.0;
      m_fTargetPositionZPrev = 0.0;
      m_cInputPosition = m_cHomePosition;
      m_fInputYawAngle = m_fHomeYawAngle;
   }

   /****************************************/
   /****************************************/

   void CJoltDroneModel::Reset() {
      /* restore the start pose and zero the velocities */
      CJoltModel::Reset();
      ReadBodyState();
      /* reset the home position and yaw angle */
      m_cHomePosition = m_cPosition;
      m_fHomeYawAngle = m_cOrientation.GetZ();
      ResetControllerState();
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltDroneModel::MoveTo(const CVector3& c_position,
                                const CQuaternion& c_orientation) {
      /* update the home pose like the pointmass3d model, so the
       * flight-system readings stay consistent */
      CRadians cYaw, cPitch, cRoll;
      c_orientation.ToEulerAngles(cYaw, cPitch, cRoll);
      Real fDeltaYaw = cYaw.GetValue() - m_cOrientation.GetZ();
      m_fHomeYawAngle += fDeltaYaw;
      CVector3 cOffsetPosition(m_cHomePosition - m_cPosition);
      m_cHomePosition = c_position + cOffsetPosition.RotateZ(CRadians(fDeltaYaw));
      /* move the body (also updates the entity status) */
      CJoltSingleBodyObjectModel::MoveTo(c_position, c_orientation);
   }

   /****************************************/
   /****************************************/

   void CJoltDroneModel::ReadBodyState() {
      JPH::BodyInterface& cInterface = GetJoltEngine().GetBodyInterface();
      const SBody& sBody = m_vecBodies[0];
      JPH::RVec3 cBodyPosition;
      JPH::Quat cBodyRotation;
      cInterface.GetPositionAndRotation(sBody.Id, cBodyPosition, cBodyRotation);
      /* origin anchor frame (body frame shifted to the bottom) */
      m_cPosition =
         ToARGoS(JPH::Vec3(cBodyPosition) -
                 cBodyRotation * sBody.AnchorOffsetPosition);
      CRadians cYaw, cPitch, cRoll;
      ToARGoS(cBodyRotation).ToEulerAngles(cYaw, cPitch, cRoll);
      m_cOrientation.Set(cRoll.GetValue(), cPitch.GetValue(), cYaw.GetValue());
      m_cVelocity = ToARGoS(cInterface.GetLinearVelocity(sBody.Id));
      /* angular velocity in the world frame: the ARGoS Euler angles
       * are extrinsic (R = Rx*Ry*Rz), so roll and pitch are tilts
       * about the WORLD axes and the attitude PIDs must see (and
       * apply torques in, see UpdatePhysics) the world frame */
      m_cAngularVelocity = ToARGoS(cInterface.GetAngularVelocity(sBody.Id));
   }

   /****************************************/
   /****************************************/

   void CJoltDroneModel::UpdateEntityStatus() {
      ReadBodyState();
      /* update the flight system entity sensor readings */
      CVector3 cPositionReading(m_cPosition - m_cHomePosition);
      cPositionReading.RotateZ(CRadians(-m_fHomeYawAngle));
      CVector3 cOrientationReading(m_cOrientation -
                                   (CVector3::Z * m_fHomeYawAngle));
      m_cFlightSystemEntity.SetPositionReading(cPositionReading);
      m_cFlightSystemEntity.SetOrientationReading(cOrientationReading);
      m_cFlightSystemEntity.SetVelocityReading(m_cVelocity);
      m_cFlightSystemEntity.SetAngularVelocityReading(m_cAngularVelocity);
      /* update the anchors, AABB, and entity components */
      CJoltModel::UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltDroneModel::UpdateFromEntityStatus() {
      /* pull actuator data from the control interface;
       * CDroneFlightSystemEntity works in the drone's home frame */
      CVector3 cTargetPosition(m_cFlightSystemEntity.GetTargetPosition());
      m_cInputPosition =
         m_cHomePosition + cTargetPosition.RotateZ(CRadians(m_fHomeYawAngle));
      m_fInputYawAngle =
         m_fHomeYawAngle + m_cFlightSystemEntity.GetTargetYawAngle().GetValue();
   }

   /****************************************/
   /****************************************/

   void CJoltDroneModel::UpdatePhysics() {
      ReadBodyState();
      Real fGravity = 9.81;
      Real fClockTick = GetJoltEngine().GetPhysicsClockTick();
      /* update the position (XY) and altitude (Z) controller */
      CVector3 cPositionError(m_cInputPosition - m_cPosition);
      /* accumulate the altitude error */
      m_fAltitudeCumulativeError += cPositionError.GetZ() * fClockTick;
      /* calculate the azimuth contribution to the navigation of the
       * drone on the XY plane */
      Real fAzimuth = std::atan2(std::abs(cPositionError.GetY()),
                                 std::abs(cPositionError.GetX()));
      /* calculate velocity limits */
      CRange<Real> cVelocityLimitX(-XY_VEL_MAX * std::cos(fAzimuth),
                                   XY_VEL_MAX * std::cos(fAzimuth));
      CRange<Real> cVelocityLimitY(-XY_VEL_MAX * std::sin(fAzimuth),
                                   XY_VEL_MAX * std::sin(fAzimuth));
      CRange<Real> cVelocityLimitZ(-Z_VEL_MAX, Z_VEL_MAX);
      /* calculate desired XYZ velocities */
      Real fTargetTransVelX = cPositionError.GetX() * XY_POS_KP;
      Real fTargetTransVelY = cPositionError.GetY() * XY_POS_KP;
      Real fTargetTransVelZ =
         (m_cInputPosition.GetZ() - m_fTargetPositionZPrev) / fClockTick;
      /* saturate velocities */
      cVelocityLimitX.TruncValue(fTargetTransVelX);
      cVelocityLimitY.TruncValue(fTargetTransVelY);
      cVelocityLimitZ.TruncValue(fTargetTransVelZ);
      m_fTargetPositionZPrev = m_cInputPosition.GetZ();
      /* XYZ velocity error */
      CVector3 cTransVelocityError(fTargetTransVelX, fTargetTransVelY, fTargetTransVelZ);
      cTransVelocityError -= m_cVelocity;
      /* desired XY accelerations */
      CVector3 cTargetTransAcc = cTransVelocityError * XY_VEL_KP;
      /* outputs of the position controller. The ARGoS Euler angles
       * are EXTRINSIC (R = Rx*Ry*Rz): roll and pitch are tilts about
       * the world axes, so the thrust direction is
       * (sin(pitch), -sin(roll)cos(pitch), cos(roll)cos(pitch)),
       * independent of yaw, and the tilt mapping needs no yaw terms
       * (pointmass3d's yaw+pi trigonometry expresses the same mapping
       * only after its sign conventions cancel; wired to a real rigid
       * body it cross-couples the loops as yaw grows) */
      Real fDesiredRollAngle = -std::cos(m_cOrientation.GetY()) * std::cos(m_cOrientation.GetX()) *
         cTargetTransAcc.GetY() / fGravity;
      Real fDesiredPitchAngle = std::cos(m_cOrientation.GetY()) * std::cos(m_cOrientation.GetX()) *
         cTargetTransAcc.GetX() / fGravity;
      Real fDesiredYawAngle = m_fInputYawAngle;
      ROLL_PITCH_LIMIT.TruncValue(fDesiredRollAngle);
      ROLL_PITCH_LIMIT.TruncValue(fDesiredPitchAngle);
      CVector3 cOrientationTarget(fDesiredRollAngle, fDesiredPitchAngle, fDesiredYawAngle);
      /* output of the altitude controller */
      Real fAltitudeControlSignal = MASS * fGravity +
         CalculatePIDResponse(cPositionError.GetZ(),
                              m_fAltitudeCumulativeError,
                              cTransVelocityError.GetZ(),
                              ALTITUDE_KP,
                              ALTITUDE_KI,
                              ALTITUDE_KD) /
         (std::cos(m_cOrientation.GetX()) * std::cos(m_cOrientation.GetY()));
      /* attitude (roll, pitch, yaw) control. This inner loop departs
       * from pointmass3d in two ways required by exact rigid-body
       * integration:
       * - the yaw read from the Jolt body wraps at +/- pi (pointmass3d
       *   integrates an unwrapped yaw state), so the yaw error must be
       *   normalized or a wrap crossing looks like a full-turn error;
       * - pointmass3d's angular-velocity target differentiates the
       *   commanded attitude, which spikes when the position
       *   controller moves its output between sub-steps; its half-step
       *   integration absorbs the kicks, an exact integrator turns
       *   them into a sustained attitude limit cycle; the feedforward
       *   is therefore clamped to a physical slew rate */
      CVector3 cOrientationError(cOrientationTarget - m_cOrientation);
      cOrientationError.SetZ(
         NormalizedDifference(CRadians(cOrientationTarget.GetZ()),
                              CRadians(m_cOrientation.GetZ())).GetValue());
      /* Yaw-rate limit, implemented as a clamp on the yaw error fed
       * to the PID. On a real rigid body a fast yaw rate r transports
       * tilt between the roll and pitch channels (dphi/dt ~ wx +
       * theta*r), a coupling absent from pointmass3d's per-axis
       * integration; an unrestricted yaw slew destabilizes the tilt
       * loops into a rotating +/- 50 deg cone */
      Real fYawError = cOrientationError.GetZ();
      YAW_ERROR_LIMIT.TruncValue(fYawError);
      cOrientationError.SetZ(fYawError);
      CVector3 cAngularVelocityTarget =
         (cOrientationTarget - m_cOrientationTargetPrev) / fClockTick;
      m_cOrientationTargetPrev = cOrientationTarget;
      Real fRateX = cAngularVelocityTarget.GetX();
      Real fRateY = cAngularVelocityTarget.GetY();
      Real fRateZ = cAngularVelocityTarget.GetZ();
      ANGULAR_RATE_LIMIT.TruncValue(fRateX);
      ANGULAR_RATE_LIMIT.TruncValue(fRateY);
      ANGULAR_RATE_LIMIT.TruncValue(fRateZ);
      cAngularVelocityTarget.Set(fRateX, fRateY, fRateZ);
      CVector3 cAngularVelocityError(cAngularVelocityTarget - m_cAngularVelocity);
      m_cAngularVelocityCumulativeError += cAngularVelocityError * fClockTick;
      Real fAttitudeControlSignalX = INERTIA.GetX() *
         CalculatePIDResponse(cOrientationError.GetX(),
                              m_cAngularVelocityCumulativeError.GetX(),
                              cAngularVelocityError.GetX(),
                              ROLL_PITCH_KP, ROLL_PITCH_KI, ROLL_PITCH_KD);
      Real fAttitudeControlSignalY = INERTIA.GetY() *
         CalculatePIDResponse(cOrientationError.GetY(),
                              m_cAngularVelocityCumulativeError.GetY(),
                              cAngularVelocityError.GetY(),
                              ROLL_PITCH_KP, ROLL_PITCH_KI, ROLL_PITCH_KD);
      Real fAttitudeControlSignalZ = INERTIA.GetZ() *
         CalculatePIDResponse(cOrientationError.GetZ(),
                              m_cAngularVelocityCumulativeError.GetZ(),
                              cAngularVelocityError.GetZ(),
                              YAW_KP, YAW_KI, YAW_KD);
      TORQUE_LIMIT.TruncValue(fAttitudeControlSignalX);
      TORQUE_LIMIT.TruncValue(fAttitudeControlSignalY);
      TORQUE_LIMIT.TruncValue(fAttitudeControlSignalZ);
      /* calculate the rotor speeds from the control signals */
      std::array<Real, 4> arrSquaredRotorSpeeds = {
         fAltitudeControlSignal / (4 * B) -
            fAttitudeControlSignalX * (ROOT_TWO / (4 * B * ARM_LENGTH)) -
            fAttitudeControlSignalY * (ROOT_TWO / (4 * B * ARM_LENGTH)) -
            fAttitudeControlSignalZ / (4 * D),
         fAltitudeControlSignal / (4 * B) -
            fAttitudeControlSignalX * (ROOT_TWO / (4 * B * ARM_LENGTH)) +
            fAttitudeControlSignalY * (ROOT_TWO / (4 * B * ARM_LENGTH)) +
            fAttitudeControlSignalZ / (4 * D),
         fAltitudeControlSignal / (4 * B) +
            fAttitudeControlSignalX * (ROOT_TWO / (4 * B * ARM_LENGTH)) +
            fAttitudeControlSignalY * (ROOT_TWO / (4 * B * ARM_LENGTH)) -
            fAttitudeControlSignalZ / (4 * D),
         fAltitudeControlSignal / (4 * B) +
            fAttitudeControlSignalX * (ROOT_TWO / (4 * B * ARM_LENGTH)) -
            fAttitudeControlSignalY * (ROOT_TWO / (4 * B * ARM_LENGTH)) +
            fAttitudeControlSignalZ / (4 * D),
      };
      /* the gyroscopic effect of the rotor speeds */
      Real fOmegaR = -std::sqrt(std::abs(arrSquaredRotorSpeeds[0])) +
                      std::sqrt(std::abs(arrSquaredRotorSpeeds[1])) +
                     -std::sqrt(std::abs(arrSquaredRotorSpeeds[2])) +
                      std::sqrt(std::abs(arrSquaredRotorSpeeds[3]));
      /* calculate the drone thrust and torques */
      Real fThrust = B *
         (arrSquaredRotorSpeeds[0] + arrSquaredRotorSpeeds[1] +
          arrSquaredRotorSpeeds[2] + arrSquaredRotorSpeeds[3]);
      Real fTorqueX = (ARM_LENGTH / ROOT_TWO) * B *
         (-arrSquaredRotorSpeeds[0] - arrSquaredRotorSpeeds[1] +
           arrSquaredRotorSpeeds[2] + arrSquaredRotorSpeeds[3]);
      Real fTorqueY = (ARM_LENGTH / ROOT_TWO) * B *
         (-arrSquaredRotorSpeeds[0] + arrSquaredRotorSpeeds[1] +
           arrSquaredRotorSpeeds[2] - arrSquaredRotorSpeeds[3]);
      Real fTorqueZ = D *
         (-arrSquaredRotorSpeeds[0] + arrSquaredRotorSpeeds[1] -
           arrSquaredRotorSpeeds[2] + arrSquaredRotorSpeeds[3]);
      THRUST_LIMIT.TruncValue(fThrust);
      /* rotor gyroscopic torques (the Euler coupling terms of the
       * original model are handled by Jolt's rigid-body integrator) */
      fTorqueX -= JR * m_cAngularVelocity.GetY() * fOmegaR;
      fTorqueY += JR * m_cAngularVelocity.GetX() * fOmegaR;
      /* apply the thrust along the true body z axis and the torques
       * in the body frame (gravity itself is applied by Jolt) */
      JPH::BodyInterface& cInterface = GetJoltEngine().GetBodyInterface();
      const JPH::BodyID& cId = m_vecBodies[0].Id;
      JPH::Quat cBodyRotation = cInterface.GetRotation(cId);
      cInterface.AddForce(
         cId, cBodyRotation * JPH::Vec3(0.0f, 0.0f, float(fThrust)));
      /* world-frame torques: the roll/pitch/yaw PIDs work on the
       * extrinsic (world-axis) Euler angles */
      cInterface.AddTorque(
         cId, JPH::Vec3(float(fTorqueX),
                        float(fTorqueY),
                        float(fTorqueZ)));
   }

   /****************************************/
   /****************************************/

   Real CJoltDroneModel::CalculatePIDResponse(Real f_cur_error,
                                              Real f_sum_error,
                                              Real f_vel_error,
                                              Real f_k_p,
                                              Real f_k_i,
                                              Real f_k_d) {
      return f_k_p * f_cur_error + f_k_i * f_sum_error + f_k_d * f_vel_error;
   }

   /****************************************/
   /****************************************/

   /* Parameters (same as the pointmass3d drone model) */
   const Real CJoltDroneModel::HEIGHT = 0.25;
   const Real CJoltDroneModel::ARM_LENGTH = 0.22;
   const Real CJoltDroneModel::MASS = 0.812;
   const CVector3 CJoltDroneModel::INERTIA = {0.01085, 0.01092, 0.02121};
   const Real CJoltDroneModel::B = 1.1236e-5;
   const Real CJoltDroneModel::D = 1.4088e-7;
   const Real CJoltDroneModel::JR = 5.225e-5;
   const CRange<Real> CJoltDroneModel::TORQUE_LIMIT = CRange<Real>(-0.5721, 0.5721);
   const CRange<Real> CJoltDroneModel::YAW_ERROR_LIMIT = CRange<Real>(-0.3, 0.3);
   const CRange<Real> CJoltDroneModel::ANGULAR_RATE_LIMIT = CRange<Real>(-3.0, 3.0);
   const CRange<Real> CJoltDroneModel::THRUST_LIMIT = CRange<Real>(-15, 15);
   const CRange<Real> CJoltDroneModel::ROLL_PITCH_LIMIT = CRange<Real>(-0.5, 0.5);
   const Real CJoltDroneModel::XY_VEL_MAX = 1;
   const Real CJoltDroneModel::Z_VEL_MAX = 0.05;
   const Real CJoltDroneModel::XY_POS_KP = 1;
   const Real CJoltDroneModel::XY_VEL_KP = 3;
   const Real CJoltDroneModel::YAW_KP = 13;
   const Real CJoltDroneModel::YAW_KI = 0;
   const Real CJoltDroneModel::YAW_KD = 8;
   const Real CJoltDroneModel::ALTITUDE_KP = 5;
   const Real CJoltDroneModel::ALTITUDE_KI = 0;
   const Real CJoltDroneModel::ALTITUDE_KD = 6;
   /* Stiffer than pointmass3d (12/6): its half-step integration slows
    * the simulated plant, so its attitude loop (wn ~3.5 rad/s) stays
    * clear of the velocity loop pole (XY_VEL_KP = 3 rad/s). Under
    * exact rigid-body integration those bandwidths collide and the
    * drone circles in a +/- 60 deg coning limit cycle; wn ~7 rad/s
    * with damping ratio ~0.7 restores the cascade separation */
   const Real CJoltDroneModel::ROLL_PITCH_KP = 12;
   const Real CJoltDroneModel::ROLL_PITCH_KI = 0;
   const Real CJoltDroneModel::ROLL_PITCH_KD = 6;
   const Real CJoltDroneModel::ROOT_TWO = std::sqrt(2.0);

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CDroneEntity, CJoltDroneModel);

   /****************************************/
   /****************************************/

}
