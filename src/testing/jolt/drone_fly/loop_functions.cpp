#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>

#include <cmath>

/****************************************/
/****************************************/

void CDroneFlyLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcDrone = &dynamic_cast<CDroneEntity&>(GetSpace().GetEntity("drone"));
}

/****************************************/
/****************************************/

void CDroneFlyLoopFunctions::PostStep() {
   /* The drone must stay upright the whole flight */
   const CQuaternion& cOrientation =
      m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Orientation;
   CRadians cYaw, cPitch, cRoll;
   cOrientation.ToEulerAngles(cYaw, cPitch, cRoll);
   if(std::fabs(cRoll.GetValue()) > 0.6 || std::fabs(cPitch.GetValue()) > 0.6) {
      THROW_ARGOSEXCEPTION("Drone lost attitude stability at tick "
                           << GetSpace().GetSimulationClock()
                           << " (roll " << cRoll.GetValue()
                           << ", pitch " << cPitch.GetValue() << ")");
   }
   /* The 1 m/s XY velocity limit must be respected: after 2 s the
    * drone cannot have covered much more than 2 m */
   if(GetSpace().GetSimulationClock() == 20) {
      const CVector3& cPosition =
         m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
      if(cPosition.GetX() > 2.2) {
         THROW_ARGOSEXCEPTION("Drone at x = " << cPosition.GetX()
                              << " after 2 s; the XY velocity limit is broken");
      }
   }
   /* After 10 s the drone must hover at the target */
   if(GetSpace().GetSimulationClock() == 100) {
      const CVector3& cPosition =
         m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
      if(Distance(cPosition, CVector3(4.0, 0.0, 1.0)) > 0.15) {
         THROW_ARGOSEXCEPTION("Drone at " << cPosition
                              << " after 10 s, expected ~(4, 0, 1)");
      }
   }
}

/****************************************/
/****************************************/

bool CDroneFlyLoopFunctions::IsExperimentFinished() {
   if(GetSpace().GetSimulationClock() < 300) {
      return false;
   }
   /* After 30 s the drone must hover at the target */
   const CVector3& cPosition =
      m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
   if(Distance(cPosition, CVector3(4.0, 0.0, 1.0)) > 0.1) {
      THROW_ARGOSEXCEPTION("Drone at " << cPosition
                           << ", expected hover at ~(4, 0, 1)");
   }
   return true;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDroneFlyLoopFunctions, "drone_fly_loop_functions");
