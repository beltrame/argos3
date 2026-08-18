/**
 * @file <argos3/plugins/robots/bunker-mini/control_interface/ci_bunker_mini_track_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "ci_bunker_mini_track_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

#ifdef ARGOS_WITH_LUA
   void CCI_BunkerMiniTrackSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "track_sensor");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "left_velocity", m_sReading.LeftVelocity);
      CLuaUtility::AddToTable(pt_lua_state, "right_velocity", m_sReading.RightVelocity);
      CLuaUtility::AddToTable(pt_lua_state, "left_distance", m_sReading.LeftDistance);
      CLuaUtility::AddToTable(pt_lua_state, "right_distance", m_sReading.RightDistance);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }

   void CCI_BunkerMiniTrackSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "track_sensor");
      CLuaUtility::AddToTable(pt_lua_state, "left_velocity", m_sReading.LeftVelocity);
      CLuaUtility::AddToTable(pt_lua_state, "right_velocity", m_sReading.RightVelocity);
      CLuaUtility::AddToTable(pt_lua_state, "left_distance", m_sReading.LeftDistance);
      CLuaUtility::AddToTable(pt_lua_state, "right_distance", m_sReading.RightDistance);
      lua_pop(pt_lua_state, 1);
   }
#endif

}
