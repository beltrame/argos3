#include "controller.h"

#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_actuator.h>

/****************************************/
/****************************************/

void CDroneOdometryController::Init(TConfigurationNode& t_tree) {
   CCI_DroneFlightSystemActuator* pcActuator =
      GetActuator<CCI_DroneFlightSystemActuator>("drone_flight_system");
   pcActuator->SetTargetPosition(CVector3(4.0, 0.0, 1.0));
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CDroneOdometryController, "drone_odometry_controller");
