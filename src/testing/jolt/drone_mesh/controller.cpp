#include "controller.h"

#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_actuator.h>

#include <sstream>

/****************************************/
/****************************************/

void CDroneMeshController::Init(TConfigurationNode& t_tree) {
   m_pcFlight = GetActuator<CCI_DroneFlightSystemActuator>("drone_flight_system");
   std::string strSchedule;
   GetNodeAttribute(t_tree, "waypoints", strSchedule);
   std::stringstream cSchedule(strSchedule);
   std::string strItem;
   while(std::getline(cSchedule, strItem, ';')) {
      if(strItem.empty()) continue;
      size_t unColon = strItem.find(':');
      if(unColon == std::string::npos) {
         THROW_ARGOSEXCEPTION("waypoint \"" << strItem << "\" is not tick:x,y,z");
      }
      SWaypoint sWaypoint;
      sWaypoint.Tick = FromString<UInt32>(strItem.substr(0, unColon));
      std::istringstream cTarget(strItem.substr(unColon + 1));
      cTarget >> sWaypoint.Target;
      m_vecWaypoints.push_back(sWaypoint);
   }
   if(m_vecWaypoints.empty()) {
      THROW_ARGOSEXCEPTION("drone_mesh_controller needs at least one waypoint");
   }
   Reset();
}

/****************************************/
/****************************************/

void CDroneMeshController::ControlStep() {
   CVector3 cTarget = m_vecWaypoints.front().Target;
   for(const SWaypoint& sWaypoint : m_vecWaypoints) {
      if(sWaypoint.Tick <= m_unTick) cTarget = sWaypoint.Target;
   }
   m_pcFlight->SetTargetPosition(cTarget);
   ++m_unTick;
}

/****************************************/
/****************************************/

void CDroneMeshController::Reset() {
   m_unTick = 0;
   m_pcFlight->SetTargetPosition(m_vecWaypoints.front().Target);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CDroneMeshController, "drone_mesh_controller");
