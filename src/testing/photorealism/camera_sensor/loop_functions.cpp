#include "loop_functions.h"
#include "controller.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/photorealism/photorealism_medium.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <cmath>

/* Camera mount x offset (0.1) subtracted from the wall face at x=1.0 */
static const Real EXPECTED_WALL_DEPTH = 0.9;

/****************************************/
/****************************************/

void CCameraTestLoopFunctions::Init(TConfigurationNode& t_tree) {
   GetNodeAttributeOrDefault(t_tree, "expected_latency",
                             m_unExpectedLatency, m_unExpectedLatency);
   auto& cFootBot = dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("fb"));
   m_pcController = &dynamic_cast<CCameraTestController&>(
      cFootBot.GetControllableEntity().GetController());
   m_pcWall = &dynamic_cast<CBoxEntity&>(GetSpace().GetEntity("wall"));
   m_pcMedium = &CSimulator::GetInstance().GetMedium<CPhotorealismMedium>("pr");
}

/****************************************/
/****************************************/

void CCameraTestLoopFunctions::PostStep() {
   UInt32 unClock = GetSpace().GetSimulationClock();
   /* Give the pipeline time to warm up */
   if(unClock < 3) {
      return;
   }
   if(!m_pcController->m_bHasFrame) {
      THROW_ARGOSEXCEPTION("No camera frame delivered by tick " << unClock);
   }
   const CCI_PhotorealisticCameraSensor::SFrame& sFrame =
      m_pcController->m_sLastFrame;
   /* Latency */
   if(unClock - sFrame.Tick != m_unExpectedLatency) {
      THROW_ARGOSEXCEPTION("Latency check failed: clock=" << unClock
                           << " frame tick=" << sFrame.Tick
                           << " expected latency=" << m_unExpectedLatency);
   }
   /* Depth of the wall at the image center */
   size_t unCenter = size_t(sFrame.Height / 2) * sFrame.Width + sFrame.Width / 2;
   if(std::abs(sFrame.Depth[unCenter] - EXPECTED_WALL_DEPTH) > 0.01) {
      THROW_ARGOSEXCEPTION("Depth check failed: center depth = "
                           << sFrame.Depth[unCenter]
                           << ", expected " << EXPECTED_WALL_DEPTH);
   }
   /* Segmentation ids of the wall at the image center */
   UInt16 unWallId = m_pcMedium->GetSceneSync().GetEntityId(
      m_pcWall->GetEmbodiedEntity());
   if(unWallId == 0) {
      THROW_ARGOSEXCEPTION("The wall has no segmentation id");
   }
   if(sFrame.EntityId[unCenter] != unWallId) {
      THROW_ARGOSEXCEPTION("Segmentation check failed: center entity id = "
                           << sFrame.EntityId[unCenter]
                           << ", expected " << unWallId);
   }
   if(sFrame.ClassId[unCenter] != UInt8(EPRClass::Box)) {
      THROW_ARGOSEXCEPTION("Segmentation check failed: center class id = "
                           << UInt32(sFrame.ClassId[unCenter])
                           << ", expected " << UInt32(EPRClass::Box));
   }
   /* The RGB image must not be empty black (the lit wall is visible) */
   UInt32 unSum = 0;
   for(size_t i = 0; i < sFrame.RGB.size(); ++i) {
      unSum += sFrame.RGB[i];
   }
   if(unSum == 0) {
      THROW_ARGOSEXCEPTION("RGB image is fully black");
   }
   m_bChecked = true;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CCameraTestLoopFunctions,
                        "camera_test_loop_functions");
