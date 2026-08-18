/**
 * @file <argos3/plugins/robots/bunker-mini/control_interface/ci_bunker_mini_track_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef CCI_BUNKER_MINI_TRACK_SENSOR_H
#define CCI_BUNKER_MINI_TRACK_SENSOR_H

namespace argos {
   class CCI_BunkerMiniTrackSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_BunkerMiniTrackSensor : public CCI_Sensor {

   public:

      struct SReading {
         Real LeftVelocity;
         Real RightVelocity;
         Real LeftDistance;
         Real RightDistance;

         SReading() :
            LeftVelocity(0.0f),
            RightVelocity(0.0f),
            LeftDistance(0.0f),
            RightDistance(0.0f) {}

         SReading(Real f_left_velocity,
                  Real f_right_velocity,
                  Real f_left_distance,
                  Real f_right_distance) :
            LeftVelocity(f_left_velocity),
            RightVelocity(f_right_velocity),
            LeftDistance(f_left_distance),
            RightDistance(f_right_distance) {}
      };

      virtual ~CCI_BunkerMiniTrackSensor() {}

      const SReading& GetReading() const {
         return m_sReading;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      SReading m_sReading;

   };

}

#endif
