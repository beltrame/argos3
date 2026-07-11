#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <cmath>

/****************************************/
/****************************************/

static const Real FOOTBOT_HEIGHT = 0.146899733;

void CFootBotDriveLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcFootBot = &dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("fb"));
}

/****************************************/
/****************************************/

void CFootBotDriveLoopFunctions::PostStep() {
   const CVector3& cPosition =
      m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
   /* Gravity must keep the robot on the floor */
   if(std::fabs(cPosition.GetZ()) > 0.005) {
      THROW_ARGOSEXCEPTION("Foot-bot left the floor: z = " << cPosition.GetZ());
   }
   /* A vertical ray through the robot must hit it (exercises the
    * Jolt ray-cast path used by the proximity sensors) */
   if(GetSpace().GetSimulationClock() == 50) {
      CRay3 cRay(CVector3(cPosition.GetX(), cPosition.GetY(), 1.0),
                 CVector3(cPosition.GetX(), cPosition.GetY(), 0.0));
      SEmbodiedEntityIntersectionItem sItem;
      if(!GetClosestEmbodiedEntityIntersectedByRay(sItem, cRay)) {
         THROW_ARGOSEXCEPTION("Ray cast did not hit the foot-bot");
      }
      if(sItem.IntersectedEntity != &m_pcFootBot->GetEmbodiedEntity()) {
         THROW_ARGOSEXCEPTION("Ray cast hit \""
                              << sItem.IntersectedEntity->GetId()
                              << "\" instead of the foot-bot");
      }
      Real fExpectedT = 1.0 - FOOTBOT_HEIGHT;
      if(std::fabs(sItem.TOnRay - fExpectedT) > 0.01) {
         THROW_ARGOSEXCEPTION("Ray hit the foot-bot at t = " << sItem.TOnRay
                              << ", expected ~" << fExpectedT);
      }
   }
}

/****************************************/
/****************************************/

bool CFootBotDriveLoopFunctions::IsExperimentFinished() {
   if(GetSpace().GetSimulationClock() < 100) {
      return false;
   }
   /* 10 s at 10 cm/s: from (-0.5, 0) to (0.5, 0) */
   const CVector3& cPosition =
      m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
   if(Distance(cPosition, CVector3(0.5, 0.0, 0.0)) > 0.02) {
      THROW_ARGOSEXCEPTION("Foot-bot at " << cPosition
                           << ", expected ~(0.5, 0, 0)");
   }
   return true;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CFootBotDriveLoopFunctions,
                        "footbot_drive_loop_functions");
