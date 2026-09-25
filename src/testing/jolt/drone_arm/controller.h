#ifndef JOLT_DRONE_ARM_CONTROLLER_H
#define JOLT_DRONE_ARM_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

namespace argos {
   class CCI_DroneFlightSystemActuator;
}

using namespace argos;

/** Targets (0, 0, 1) throughout; arms at `arm_tick`, disarms at `disarm_tick` */
class CDroneArmController : public CCI_Controller {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void ControlStep();

private:

   CCI_DroneFlightSystemActuator* m_pcFlight = nullptr;
   UInt32 m_unTick = 0;
   UInt32 m_unArmTick = 0;
   UInt32 m_unDisarmTick = 0;

};

#endif
