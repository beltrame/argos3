#ifndef JOLT_FOOTBOT_DRIVE_CONTROLLER_H
#define JOLT_FOOTBOT_DRIVE_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

using namespace argos;

/** Drives forward at 10 cm/s */
class CFootBotDriveController : public CCI_Controller {

public:

   virtual void Init(TConfigurationNode& t_tree);

};

#endif
