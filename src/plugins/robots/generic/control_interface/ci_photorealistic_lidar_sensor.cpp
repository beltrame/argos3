/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "ci_photorealistic_lidar_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * Only the scan metadata is exposed to Lua, as for the camera. A
    * default scan is 16 channels over 1800 azimuths; pushing 28800
    * readings into a Lua table every control step would cost more than
    * the render that produced them. Controllers that consume scans
    * should be written in C++.
    */
   void CCI_PhotorealisticLidarSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "photorealistic_lidar");
      CLuaUtility::AddToTable(pt_lua_state, "rings", Real(0));
      CLuaUtility::AddToTable(pt_lua_state, "azimuths", Real(0));
      CLuaUtility::AddToTable(pt_lua_state, "max_range", Real(0));
      CLuaUtility::AddToTable(pt_lua_state, "tick", Real(0));
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }

   /****************************************/
   /****************************************/

   void CCI_PhotorealisticLidarSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      const SScan& sScan = GetScan();
      lua_getfield(pt_lua_state, -1, "photorealistic_lidar");
      CLuaUtility::AddToTable(pt_lua_state, "rings", Real(sScan.NumRings));
      CLuaUtility::AddToTable(pt_lua_state, "azimuths", Real(sScan.NumAzimuths));
      CLuaUtility::AddToTable(pt_lua_state, "max_range", sScan.MaxRange);
      CLuaUtility::AddToTable(pt_lua_state, "tick", Real(sScan.Tick));
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/

}
