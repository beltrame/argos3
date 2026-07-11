#ifndef JOLT_DRONE_FLY_CONTROLLER_H
#define JOLT_DRONE_FLY_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

using namespace argos;

/** Flies towards (4, 0, 1) in the drone's home frame */
class CDroneFlyController : public CCI_Controller {

public:

   virtual void Init(TConfigurationNode& t_tree);

};

#endif
