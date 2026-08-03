#ifndef JOLT_DRONE_ODOMETRY_CONTROLLER_H
#define JOLT_DRONE_ODOMETRY_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

using namespace argos;

/**
 * Flies towards (4, 0, 1) in the drone's home frame, exactly like the
 * drone_fly test. The odometry sensor is listed in the XML purely so
 * the loop functions can read it off the controller (via GetSensor());
 * this controller does not need to touch it itself.
 */
class CDroneOdometryController : public CCI_Controller {

public:

   virtual void Init(TConfigurationNode& t_tree);

};

#endif
