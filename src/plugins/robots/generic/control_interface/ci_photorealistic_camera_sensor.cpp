/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "ci_photorealistic_camera_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * Only the frame metadata is exposed to Lua; per-pixel access from
    * Lua would be too slow to be useful. Vision controllers needing
    * pixel data should be written in C++.
    */
   void CCI_PhotorealisticCameraSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "photorealistic_camera");
      CLuaUtility::AddToTable(pt_lua_state, "width", Real(0));
      CLuaUtility::AddToTable(pt_lua_state, "height", Real(0));
      CLuaUtility::AddToTable(pt_lua_state, "tick", Real(0));
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }

   /****************************************/
   /****************************************/

   void CCI_PhotorealisticCameraSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      const SFrame& sFrame = GetFrame();
      lua_getfield(pt_lua_state, -1, "photorealistic_camera");
      CLuaUtility::AddToTable(pt_lua_state, "width", Real(sFrame.Width));
      CLuaUtility::AddToTable(pt_lua_state, "height", Real(sFrame.Height));
      CLuaUtility::AddToTable(pt_lua_state, "tick", Real(sFrame.Tick));
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/

}
