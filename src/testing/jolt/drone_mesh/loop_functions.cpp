#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>
#include <argos3/plugins/robots/drone/simulator/drone_flight_system_entity.h>

#include <cmath>
#include <iostream>

/****************************************/
/****************************************/

void CDroneMeshLoopFunctions::Init(TConfigurationNode& t_tree) {
   std::string strDrone = "drone";
   GetNodeAttributeOrDefault(t_tree, "drone", strDrone, strDrone);
   m_pcDrone = &dynamic_cast<CDroneEntity&>(GetSpace().GetEntity(strDrone));
   GetNodeAttribute(t_tree, "wall_from", m_unWallFrom);
   GetNodeAttribute(t_tree, "wall_to", m_unWallTo);
   GetNodeAttribute(t_tree, "wall_limit_y", m_fWallLimitY);
   GetNodeAttribute(t_tree, "wall_reach_y", m_fWallReachY);
   GetNodeAttribute(t_tree, "min_z", m_fMinZ);
   GetNodeAttribute(t_tree, "end_tick", m_unEndTick);
   GetNodeAttribute(t_tree, "expect_final", m_cExpectFinal);
   GetNodeAttributeOrDefault(t_tree, "final_tolerance", m_fFinalTolerance, m_fFinalTolerance);
   const SAnchor& sOrigin = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor();
   m_cHomePosition = sOrigin.Position;
   CRadians cPitch, cRoll;
   sOrigin.Orientation.ToEulerAngles(m_cHomeYaw, cPitch, cRoll);
}

/****************************************/
/****************************************/

void CDroneMeshLoopFunctions::PostStep() {
   UInt32 unTick = GetSpace().GetSimulationClock();
   CEmbodiedEntity& cBody = m_pcDrone->GetEmbodiedEntity();
   const CVector3& cPosition = cBody.GetOriginAnchor().Position;
   if(cPosition.GetZ() < m_fMinZ) {
      THROW_ARGOSEXCEPTION("Drone sank to z = " << cPosition.GetZ() << " at tick "
                           << unTick << "; the mesh floor did not carry it");
   }
   if(unTick >= m_unWallFrom && unTick < m_unWallTo) {
      m_fMaxWallY = std::max(m_fMaxWallY, cPosition.GetY());
      /* IsCollidingWithSomething reports penetration only (no separation
       * tolerance), but Jolt's speculative contacts hold a drone resting
       * on the wall at separation ~0, so a contact shows on a tick only
       * through rounding. A stall at the wall, with the target beyond it,
       * counts as the hit too: in free flight the drone cannot stop there */
      if(cBody.IsCollidingWithSomething()) m_bContact = true;
      const CDroneFlightSystemEntity& cFlight = m_pcDrone->GetFlightSystemEntity();
      CVector3 cTarget = m_cHomePosition +
         CVector3(cFlight.GetTargetPosition()).RotateZ(m_cHomeYaw);
      if(cTarget.GetY() > m_fWallLimitY &&
         cPosition.GetY() >= m_fWallReachY &&
         std::fabs(cFlight.GetVelocityReading().GetY()) < STALL_VELOCITY) {
         if(++m_unStallTicks >= STALL_TICKS) m_bStalled = true;
      }
      else {
         m_unStallTicks = 0;
      }
   }
   if(unTick == m_unWallTo) {
      std::cout << "[drone_mesh] wall: max y " << m_fMaxWallY
                << ", collided " << (m_bContact || m_bStalled)
                << " (contact " << m_bContact
                << ", stalled " << m_bStalled << ")" << std::endl;
      if(!m_bContact && !m_bStalled) {
         THROW_ARGOSEXCEPTION("Drone was sent through the wall but neither a collision "
                              "nor a stall against it was seen");
      }
      if(m_fMaxWallY > m_fWallLimitY) {
         THROW_ARGOSEXCEPTION("Drone reached y = " << m_fMaxWallY
                              << ", past the wall limit " << m_fWallLimitY);
      }
      if(m_fMaxWallY < m_fWallReachY) {
         THROW_ARGOSEXCEPTION("Drone stopped at y = " << m_fMaxWallY
                              << ", short of the wall (expected at least "
                              << m_fWallReachY << ")");
      }
   }
}

/****************************************/
/****************************************/

bool CDroneMeshLoopFunctions::IsExperimentFinished() {
   if(GetSpace().GetSimulationClock() < m_unEndTick) return false;
   const SAnchor& sOrigin = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor();
   CRadians cYaw, cPitch, cRoll;
   sOrigin.Orientation.ToEulerAngles(cYaw, cPitch, cRoll);
   std::cout << "[drone_mesh] final " << sOrigin.Position << " roll "
             << cRoll.GetValue() << " pitch " << cPitch.GetValue() << std::endl;
   CVector3 cError = sOrigin.Position - m_cExpectFinal;
   if(std::hypot(cError.GetX(), cError.GetY()) > m_fFinalTolerance ||
      std::fabs(cError.GetZ()) > m_fFinalTolerance) {
      THROW_ARGOSEXCEPTION("Drone ended at " << sOrigin.Position << ", expected ~"
                           << m_cExpectFinal << " +/- " << m_fFinalTolerance);
   }
   if(std::fabs(cRoll.GetValue()) > 0.2 || std::fabs(cPitch.GetValue()) > 0.2) {
      THROW_ARGOSEXCEPTION("Drone ended tilted (roll " << cRoll.GetValue()
                           << ", pitch " << cPitch.GetValue() << ")");
   }
   return true;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDroneMeshLoopFunctions, "drone_mesh_loop_functions");
