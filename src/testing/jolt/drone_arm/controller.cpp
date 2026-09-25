#include "controller.h"

#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_actuator.h>

void CDroneArmController::Init(TConfigurationNode& t_tree) {
   m_pcFlight = GetActuator<CCI_DroneFlightSystemActuator>("drone_flight_system");
   GetNodeAttribute(t_tree, "arm_tick", m_unArmTick);
   GetNodeAttribute(t_tree, "disarm_tick", m_unDisarmTick);
   m_pcFlight->SetTargetPosition(CVector3(0.0, 0.0, 1.0));
}

void CDroneArmController::ControlStep() {
   ++m_unTick;
   if(m_unTick == m_unArmTick) m_pcFlight->Arm(true, false);
   if(m_unTick == m_unDisarmTick) m_pcFlight->Arm(false, false);
}

REGISTER_CONTROLLER(CDroneArmController, "drone_arm_controller");
