/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */
#ifndef CCI_IMU_SENSOR_H
#define CCI_IMU_SENSOR_H

namespace argos {
   class CCI_IMUSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   /**
    * A generic inertial measurement unit (gyroscope + accelerometer).
    *
    * Both readings are expressed in the sensor body frame:
    * - AngularVelocity is the true rotational rate (rad/s).
    * - LinearAcceleration is the specific force (m/s^2), i.e. the
    *   gravity-compensated proper acceleration a real accelerometer
    *   reports. A robot at rest on the ground therefore reads
    *   (0,0,+g) rather than (0,0,0), matching real hardware.
    */
   class CCI_IMUSensor : public CCI_Sensor {

   public:

      struct SReading {
         CVector3 AngularVelocity;
         CVector3 LinearAcceleration;
      };

   public:

      virtual ~CCI_IMUSensor() {}

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
