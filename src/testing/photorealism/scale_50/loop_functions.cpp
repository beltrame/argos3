#include "loop_functions.h"
#include "../camera_sensor/controller.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/****************************************/
/****************************************/

void CScale50LoopFunctions::Init(TConfigurationNode& t_tree) {
   CSpace::TMapPerType& tFootBots = GetSpace().GetEntitiesByType("foot-bot");
   for(auto& tFootBot : tFootBots) {
      auto& cFootBot = *any_cast<CFootBotEntity*>(tFootBot.second);
      m_vecControllers.push_back(
         &dynamic_cast<CCameraTestController&>(
            cFootBot.GetControllableEntity().GetController()));
   }
   if(m_vecControllers.size() != 50) {
      THROW_ARGOSEXCEPTION("Expected 50 foot-bots, found "
                           << m_vecControllers.size());
   }
}

/****************************************/
/****************************************/

void CScale50LoopFunctions::PostExperiment() {
   /* Pipelined delivery: the frame rendered at tick N is sensed at
    * tick N+1 and read by the controller at tick N+2, so over T ticks
    * every camera must have received at least T-2 frames */
   UInt32 unTicks = CSimulator::GetInstance().GetMaxSimulationClock();
   for(size_t i = 0; i < m_vecControllers.size(); ++i) {
      UInt32 unFrames = m_vecControllers[i]->m_unFrameCount;
      if(unFrames + 2 < unTicks) {
         THROW_ARGOSEXCEPTION("Camera " << i << " received only "
                              << unFrames << " frames over "
                              << unTicks << " ticks");
      }
   }
   LOG << "[INFO] All " << m_vecControllers.size()
       << " cameras delivered " << (unTicks - 2)
       << "+ frames" << std::endl;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CScale50LoopFunctions, "scale_50_loop_functions");
