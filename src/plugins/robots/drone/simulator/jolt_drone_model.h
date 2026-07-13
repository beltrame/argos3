/**
 * @file <argos3/plugins/robots/drone/simulator/jolt_drone_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_DRONE_MODEL_H
#define JOLT_DRONE_MODEL_H

namespace argos {
   class CJoltDroneModel;
   class CDroneEntity;
   class CDroneFlightSystemEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

#include <argos3/core/utility/math/range.h>

namespace argos {

   /**
    * The drone in the Jolt engine: a dynamic body driven by the
    * cascaded PID flight controller ported from the pointmass3d drone
    * model. The controller outputs (rotor thrust and torques) are
    * applied as forces on the body every physics sub-step and Jolt
    * integrates the rigid-body dynamics; the MEMS sensor noise of the
    * original model is not simulated.
    */
   class CJoltDroneModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltDroneModel(CJoltEngine& c_engine,
                      CDroneEntity& c_drone);

      virtual ~CJoltDroneModel() {}

      virtual void Reset();

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation);

      virtual void UpdateEntityStatus();

      virtual void UpdateFromEntityStatus();

      virtual void UpdatePhysics();

   private:

      /** Reads pose and velocities of the origin anchor frame from
       *  the Jolt body into the m_c* state below */
      void ReadBodyState();

      void ResetControllerState();

      static Real CalculatePIDResponse(Real f_cur_error,
                                       Real f_sum_error,
                                       Real f_vel_error,
                                       Real f_k_p,
                                       Real f_k_i,
                                       Real f_k_d);

   private:

      CDroneFlightSystemEntity& m_cFlightSystemEntity;
      /* position and yaw input from the controller */
      Real m_fInputYawAngle;
      CVector3 m_cInputPosition;
      /* home position and yaw angle */
      CVector3 m_cHomePosition;
      Real m_fHomeYawAngle;
      /* drone state read from the Jolt body */
      CVector3 m_cPosition;
      CVector3 m_cOrientation; /* (roll, pitch, yaw) */
      CVector3 m_cVelocity;
      CVector3 m_cAngularVelocity; /* world frame */
      /* PID controller state */
      CVector3 m_cOrientationTargetPrev;
      CVector3 m_cAngularVelocityCumulativeError;
      Real m_fAltitudeCumulativeError;
      Real m_fTargetPositionZPrev;

      const static Real ROOT_TWO;
      const static Real HEIGHT;
      const static Real ARM_LENGTH;
      const static Real MASS;
      const static CVector3 INERTIA;
      const static Real B;
      const static Real D;
      const static Real JR;
      const static CRange<Real> ROLL_PITCH_LIMIT;
      const static CRange<Real> THRUST_LIMIT;
      const static CRange<Real> TORQUE_LIMIT;
      /* Yaw-rate limit (clamp on the yaw error fed to the PID) */
      const static CRange<Real> YAW_ERROR_LIMIT;
      /* Slew limit on the attitude-target feedforward */
      const static CRange<Real> ANGULAR_RATE_LIMIT;
      const static Real XY_VEL_MAX;
      const static Real Z_VEL_MAX;
      const static Real XY_POS_KP;
      const static Real XY_VEL_KP;
      const static Real ALTITUDE_KP;
      const static Real ALTITUDE_KI;
      const static Real ALTITUDE_KD;
      const static Real ROLL_PITCH_KP;
      const static Real ROLL_PITCH_KI;
      const static Real ROLL_PITCH_KD;
      const static Real YAW_KP;
      const static Real YAW_KI;
      const static Real YAW_KD;

   };

}

#endif
