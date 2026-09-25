#ifndef JOLT_DRONE_MESH_CONTROLLER_H
#define JOLT_DRONE_MESH_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector3.h>

#include <vector>

namespace argos {
   class CCI_DroneFlightSystemActuator;
}

using namespace argos;

/**
 * Flies a schedule of home-frame targets: from each waypoint's tick on,
 * the drone is sent to that waypoint's position.
 */
class CDroneMeshController : public CCI_Controller {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void ControlStep();
   virtual void Reset();

private:

   struct SWaypoint {
      UInt32 Tick;
      CVector3 Target;
   };

   std::vector<SWaypoint> m_vecWaypoints;
   CCI_DroneFlightSystemActuator* m_pcFlight = nullptr;
   UInt32 m_unTick = 0;

};

#endif
