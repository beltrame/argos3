/**
 * @file <argos3/plugins/robots/bunker-mini/control_interface/ci_bunker_mini_track_actuator.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef CCI_BUNKER_MINI_TRACK_ACTUATOR_H
#define CCI_BUNKER_MINI_TRACK_ACTUATOR_H

namespace argos {
   class CCI_BunkerMiniTrackActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>

namespace argos {

   class CCI_BunkerMiniTrackActuator : public CCI_Actuator {

   public:

      virtual ~CCI_BunkerMiniTrackActuator() {}

      /**
       * Sets the linear velocity of the left and right tracks in cm/s.
       */
      virtual void SetLinearVelocity(Real f_left_velocity,
                                     Real f_right_velocity) = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
#endif

   };

}

#endif
