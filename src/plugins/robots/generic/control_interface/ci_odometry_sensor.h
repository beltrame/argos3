/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */
#ifndef CCI_ODOMETRY_SENSOR_H
#define CCI_ODOMETRY_SENSOR_H

namespace argos {
   class CCI_OdometrySensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

namespace argos {

   /**
    * A generic odometry sensor.
    *
    * Unlike CCI_PositioningSensor (ground truth), this sensor reports a
    * pose estimate that drifts away from ground truth as a real
    * odometry pipeline would. Two implementations exist:
    *
    * - "drift" models the drift statistically, perturbing ground-truth
    *   relative motion, so no odometry algorithm runs at all;
    * - "external" carries the pose computed by a real estimator running
    *   outside ARGoS (see the external_estimator plugin), so the drift
    *   is whatever that estimator actually produces.
    *
    * Both feed SLAM/localization stacks that expect a continuous but
    * imperfect pose estimate.
    *
    * FRAME. Both report START-RELATIVE poses: identity on the first tick,
    * whatever the robot's pose in the arena, and relative motion accumulated
    * from there. That is what real hardware does, since neither an encoder
    * nor a SLAM front end knows where in the world it was switched on, and it
    * is the premise a collaborative stack is written against: recovering the
    * transform between robots is the job, not an input to it. The one
    * exception is deliberate and explicit, the external_estimator medium's
    * alignment="ground_truth", which hands out the world frame for tests that
    * need a known answer.
    *
    * A consumer wanting arena coordinates composes the robot's known start
    * pose onto the reading. Composing it onto a reading that already carries
    * the start pose places the robot at twice its spawn offset and rotates it
    * twice as far, which is a wrong answer that looks plausible.
    */
   class CCI_OdometrySensor : public CCI_Sensor {

   public:

      struct SReading {
         CVector3 Position;
         CQuaternion Orientation;
         /** Linear velocity in the body frame, m/s. Left at zero by
          *  implementations that do not estimate it. */
         CVector3 LinearVelocity;
         /** Angular velocity in the body frame, rad/s. Left at zero by
          *  implementations that do not estimate it. */
         CVector3 AngularVelocity;
         /** The simulation tick this estimate refers to. */
         UInt32 Tick = 0;
         /** False until a pose is available. The "drift" implementation
          *  has one from the first tick; an external estimator needs
          *  time to initialize, and reports nothing until it has. */
         bool Valid = false;
      };

   public:

      virtual ~CCI_OdometrySensor() {}

      const SReading& GetReading() const;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      SReading m_sReading;

   };

}

#endif
