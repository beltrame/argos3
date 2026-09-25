#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>

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
      if(cBody.IsCollidingWithSomething()) m_bCollided = true;
   }
   if(unTick == m_unWallTo) {
      std::cout << "[drone_mesh] wall: max y " << m_fMaxWallY
                << ", collided " << m_bCollided << std::endl;
      if(!m_bCollided) {
         THROW_ARGOSEXCEPTION("Drone was sent through the wall but no collision was reported");
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
