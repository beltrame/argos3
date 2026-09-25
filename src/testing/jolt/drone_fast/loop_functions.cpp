#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>

#include <cmath>
#include <iostream>

void CDroneFastLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcDrone = &dynamic_cast<CDroneEntity&>(GetSpace().GetEntity("drone"));
   m_cPrevious = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
}

void CDroneFastLoopFunctions::PostStep() {
   UInt32 unTick = GetSpace().GetSimulationClock();
   const SAnchor& sOrigin = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor();
   CRadians cYaw, cPitch, cRoll;
   sOrigin.Orientation.ToEulerAngles(cYaw, cPitch, cRoll);
   if(std::fabs(cRoll.GetValue()) > 0.6 || std::fabs(cPitch.GetValue()) > 0.6) {
      THROW_ARGOSEXCEPTION("Drone lost attitude at tick " << unTick);
   }
   /* 100 ticks per second */
   Real fSpeed = std::hypot(sOrigin.Position.GetX() - m_cPrevious.GetX(),
                            sOrigin.Position.GetY() - m_cPrevious.GetY()) * 100.0;
   m_fPeakSpeed = std::max(m_fPeakSpeed, fSpeed);
   m_cPrevious = sOrigin.Position;
   if(unTick == 200 && sOrigin.Position.GetX() < 3.0) {
      THROW_ARGOSEXCEPTION("Drone at x = " << sOrigin.Position.GetX()
                           << " after 2 s; max_xy_velocity was not applied");
   }
}

bool CDroneFastLoopFunctions::IsExperimentFinished() {
   if(GetSpace().GetSimulationClock() < 1500) return false;
   const CVector3& cPosition = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
   std::cout << "[drone_fast] peak speed " << m_fPeakSpeed << " m/s, final "
             << cPosition << std::endl;
   if(m_fPeakSpeed < 4.0 || m_fPeakSpeed > 5.6) {
      THROW_ARGOSEXCEPTION("Peak speed " << m_fPeakSpeed << " m/s, expected ~5");
   }
   if(Distance(cPosition, CVector3(20.0, 0.0, 1.0)) > 0.15) {
      THROW_ARGOSEXCEPTION("Drone at " << cPosition << ", expected hover at ~(20, 0, 1)");
   }
   return true;
}

REGISTER_LOOP_FUNCTIONS(CDroneFastLoopFunctions, "drone_fast_loop_functions");
