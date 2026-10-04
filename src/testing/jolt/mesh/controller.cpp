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
   TConfigurationNodeIterator it;
   for(it = it.begin(&t_tree); it != it.end(); ++it) {
      SCommand sCommand;
      GetNodeAttribute(*it, "tick", sCommand.Tick);
      GetNodeAttribute(*it, "left", sCommand.Left);
      GetNodeAttribute(*it, "right", sCommand.Right);
      if(!m_vecCommands.empty() && sCommand.Tick <= m_vecCommands.back().Tick)
         THROW_ARGOSEXCEPTION("Test commands must have increasing ticks");
      m_vecCommands.push_back(sCommand);
   }
}

/****************************************/
/****************************************/

void CMeshDriveController::ControlStep() {
   ++m_unTick;
   Real fLeft = m_fLeft, fRight = m_fRight;
   if(m_unInterruptTick && m_unTick >= m_unInterruptTick &&
      m_unTick < m_unInterruptTick + m_unInterruptTicks) {
      fLeft = m_fInterruptLeft;
      fRight = m_fInterruptRight;
   }
   for(const auto& sCommand : m_vecCommands) {
      if(m_unTick < sCommand.Tick) break;
      fLeft = sCommand.Left;
      fRight = sCommand.Right;
   }
   m_pcWheels->SetLinearVelocity(fLeft, fRight);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CMeshDriveController, "mesh_drive_controller");
