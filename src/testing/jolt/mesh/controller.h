/**
 * @file <argos3/testing/jolt/mesh/controller.h>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#ifndef JOLT_MESH_DRIVE_CONTROLLER_H
#define JOLT_MESH_DRIVE_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <vector>

namespace argos {
   class CCI_DifferentialSteeringActuator;
}

using namespace argos;

/**
 * Holds the XML wheel velocities, optionally interrupting them for a fixed
 * tick interval or overriding them with ordered <command tick="..." left="..."
 * right="..." /> entries. All motion remains the physics engine's doing.
 *
 *   <params left="50" right="50" />
 *
 * The units are those of CCI_DifferentialSteeringActuator, i.e. cm/s.
 */
class CMeshDriveController : public CCI_Controller {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void ControlStep();

private:

   CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
   Real m_fLeft = 0.0;
   Real m_fRight = 0.0;
   UInt32 m_unTick = 0;
   UInt32 m_unInterruptTick = 0;
   UInt32 m_unInterruptTicks = 20;
   Real m_fInterruptLeft = 0.0;
   Real m_fInterruptRight = 0.0;
   struct SCommand { UInt32 Tick; Real Left; Real Right; };
   std::vector<SCommand> m_vecCommands;

};

#endif
