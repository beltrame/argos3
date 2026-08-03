/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "ci_imu_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   const CCI_IMUSensor::SReading& CCI_IMUSensor::GetReading() const {
      return m_sReading;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_IMUSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::StartTable(pt_lua_state, "imu");
      CLuaUtility::AddToTable(pt_lua_state, "angular_velocity",    m_sReading.AngularVelocity);
      CLuaUtility::AddToTable(pt_lua_state, "linear_acceleration", m_sReading.LinearAcceleration);
      CLuaUtility::EndTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_IMUSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "imu");
      CLuaUtility::AddToTable(pt_lua_state, "angular_velocity",    m_sReading.AngularVelocity);
      CLuaUtility::AddToTable(pt_lua_state, "linear_acceleration", m_sReading.LinearAcceleration);
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/

}
