#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>

#include <cmath>

void CDroneArmLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcDrone = &dynamic_cast<CDroneEntity&>(GetSpace().GetEntity("drone"));
}

void CDroneArmLoopFunctions::PostStep() {
   UInt32 unTick = GetSpace().GetSimulationClock();
   Real fZ = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
   if(unTick == 19 && fZ > 0.02) {
      THROW_ARGOSEXCEPTION("Disarmed drone left the floor (z = " << fZ << ")");
   }
   if(unTick == 100 && std::fabs(fZ - 1.0) > 0.1) {
      THROW_ARGOSEXCEPTION("Armed drone at z = " << fZ << " after 8 s, expected ~1");
   }
}

bool CDroneArmLoopFunctions::IsExperimentFinished() {
   if(GetSpace().GetSimulationClock() < 200) return false;
   Real fZ = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
   if(fZ > 0.05) {
      THROW_ARGOSEXCEPTION("Drone disarmed in the air is still at z = " << fZ);
   }
   return true;
}

REGISTER_LOOP_FUNCTIONS(CDroneArmLoopFunctions, "drone_arm_loop_functions");
