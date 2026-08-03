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
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

namespace argos {

   /**
    * A generic odometry sensor.
    *
    * Unlike CCI_PositioningSensor (ground truth), this sensor models a
    * dead-reckoning pose estimate that drifts away from ground truth
    * over distance travelled, as a real wheel/visual odometry pipeline
    * would. It exists to feed SLAM/localization stacks that expect a
    * continuous but imperfect pose estimate, without requiring an
    * actual odometry algorithm to run inside ARGoS.
    */
   class CCI_OdometrySensor : public CCI_Sensor {

   public:

      struct SReading {
         CVector3 Position;
         CQuaternion Orientation;
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
