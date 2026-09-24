/**
 * @file <argos3/testing/jolt/mesh/controller.cpp>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#include "controller.h"

#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/****************************************/
/****************************************/

void CMeshDriveController::Init(TConfigurationNode& t_tree) {
   m_pcWheels =
      GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   GetNodeAttributeOrDefault(t_tree, "left", m_fLeft, m_fLeft);
   GetNodeAttributeOrDefault(t_tree, "right", m_fRight, m_fRight);
   GetNodeAttributeOrDefault(t_tree, "interrupt_tick", m_unInterruptTick, m_unInterruptTick);
   GetNodeAttributeOrDefault(t_tree, "interrupt_ticks", m_unInterruptTicks, m_unInterruptTicks);
   GetNodeAttributeOrDefault(t_tree, "interrupt_left", m_fInterruptLeft, m_fInterruptLeft);
   GetNodeAttributeOrDefault(t_tree, "interrupt_right", m_fInterruptRight, m_fInterruptRight);
}

/****************************************/
/****************************************/

void CMeshDriveController::ControlStep() {
   ++m_unTick;
   if(m_unInterruptTick && m_unTick >= m_unInterruptTick &&
      m_unTick < m_unInterruptTick + m_unInterruptTicks)
      m_pcWheels->SetLinearVelocity(m_fInterruptLeft, m_fInterruptRight);
   else
      m_pcWheels->SetLinearVelocity(m_fLeft, m_fRight);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CMeshDriveController, "mesh_drive_controller");
