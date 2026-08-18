/**
 * @file <argos3/plugins/robots/bunker-mini/control_interface/ci_bunker_mini_track_actuator.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "ci_bunker_mini_track_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

#ifdef ARGOS_WITH_LUA
   /*
    * Function to set track linear velocities from Lua
    */
   int LuaBunkerMiniTrackSetLinearVelocity(lua_State* pt_lua_state) {
      if(lua_gettop(pt_lua_state) != 2) {
         return luaL_error(pt_lua_state, "robot.tracks.set_linear_velocity() expects 2 arguments");
      }
      Real fLeft = luaL_checknumber(pt_lua_state, 1);
      Real fRight = luaL_checknumber(pt_lua_state, 2);
      CLuaUtility::GetDeviceInstance<CCI_BunkerMiniTrackActuator>(pt_lua_state, "tracks")->SetLinearVelocity(fLeft, fRight);
      return 0;
   }
#endif

#ifdef ARGOS_WITH_LUA
   void CCI_BunkerMiniTrackActuator::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "tracks");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "set_linear_velocity", &LuaBunkerMiniTrackSetLinearVelocity);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

}
