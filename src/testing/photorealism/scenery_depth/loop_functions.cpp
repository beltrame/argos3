#include "loop_functions.h"
#include "../camera_sensor/controller.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_id_scene.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <cmath>

/****************************************/
/****************************************/

void CSceneryDepthLoopFunctions::Init(TConfigurationNode& t_tree) {
   auto& cObserver = dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("observer"));
   m_pcController = &dynamic_cast<CCameraTestController&>(
      cObserver.GetControllableEntity().GetController());
   GetNodeAttribute(t_tree, "expected_depth", m_fExpectedDepth);
   GetNodeAttribute(t_tree, "depth_tolerance", m_fDepthTolerance);
   GetNodeAttribute(t_tree, "far_plane", m_fFarPlane);
}

/****************************************/
/****************************************/

void CSceneryDepthLoopFunctions::PostStep() {
   if(GetSpace().GetSimulationClock() < 3 || !m_pcController->m_bHasFrame) {
      return;
   }
   const CCI_PhotorealisticCameraSensor::SFrame& sFrame =
      m_pcController->m_sLastFrame;
   /* Collect the pixels the scenery prop covers */
   size_t unSceneryPixels = 0;
   size_t unZeroIdPixels = 0;
   UInt16 unSceneryId = 0;
   Real fDepthSum = 0.0;
   Real fDepthMin = m_fFarPlane;
   for(size_t i = 0; i < sFrame.EntityId.size(); ++i) {
      if(sFrame.ClassId[i] == UInt8(EPRClass::Scenery)) {
         if(sFrame.EntityId[i] == 0) {
            /* Would mean the prop reached the aux buffer without an
             * identity, which is exactly the state that made its depth
             * unreadable */
            ++unZeroIdPixels;
            continue;
         }
         unSceneryId = sFrame.EntityId[i];
         ++unSceneryPixels;
         fDepthSum += sFrame.Depth[i];
         fDepthMin = std::min(fDepthMin, sFrame.Depth[i]);
      }
   }
   if(unZeroIdPixels > 0) {
      THROW_ARGOSEXCEPTION("Scenery is drawn with entity id 0 in "
                           << unZeroIdPixels << " pixels; the camera sensor "
                           "reads id 0 as empty space and replaces its depth "
                           "with the far plane");
   }
   /* The prop is a wall filling the middle of the view */
   if(unSceneryPixels < 200) {
      THROW_ARGOSEXCEPTION("Expected the scenery prop to cover the view, but "
                           "only " << unSceneryPixels << " pixels carry the "
                           "scenery class; scenery is missing from the "
                           "segmentation mask");
   }
   if(unSceneryId == 0) {
      THROW_ARGOSEXCEPTION("The scenery prop has no entity id");
   }
   /* Its depth must be the real distance, not the far plane */
   Real fMeanDepth = fDepthSum / Real(unSceneryPixels);
   if(fMeanDepth >= m_fFarPlane - 1e-3) {
      THROW_ARGOSEXCEPTION("Scenery depth reads the far plane ("
                           << m_fFarPlane << " m): the geometry was rendered "
                           "but its depth was discarded");
   }
   if(std::abs(fDepthMin - m_fExpectedDepth) > m_fDepthTolerance) {
      THROW_ARGOSEXCEPTION("Scenery depth is metrically wrong: nearest sample "
                           << fDepthMin << " m, expected "
                           << m_fExpectedDepth << " +/- "
                           << m_fDepthTolerance << " m");
   }
   /* Depth must stay inside the documented [near, far] range */
   for(size_t i = 0; i < sFrame.Depth.size(); ++i) {
      if(sFrame.Depth[i] > m_fFarPlane) {
         THROW_ARGOSEXCEPTION("Depth sample " << sFrame.Depth[i]
                              << " m exceeds the far plane " << m_fFarPlane);
      }
   }
   m_bChecked = true;
}

/****************************************/
/****************************************/

void CSceneryDepthLoopFunctions::PostExperiment() {
   if(!m_bChecked) {
      THROW_ARGOSEXCEPTION("The scenery depth check never ran: no camera "
                           "frame was delivered");
   }
   LOG << "[TEST] Scenery is visible to depth and segmentation" << std::endl;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CSceneryDepthLoopFunctions, "scenery_depth_loop_functions");
