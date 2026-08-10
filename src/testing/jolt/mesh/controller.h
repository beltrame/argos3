/**
 * @file <argos3/testing/jolt/mesh/controller.h>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#ifndef JOLT_MESH_DRIVE_CONTROLLER_H
#define JOLT_MESH_DRIVE_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

namespace argos {
   class CCI_DifferentialSteeringActuator;
}

using namespace argos;

/**
 * Holds both wheels at the constant velocity given in the XML and does
 * nothing else, so that whatever happens to the robot is the physics
 * engine's doing.
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

};

#endif
