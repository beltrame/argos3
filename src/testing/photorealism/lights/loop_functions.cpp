#include "loop_functions.h"
#include "../camera_sensor/controller.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/photorealism/photorealism_medium.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/****************************************/
/****************************************/

void CLightsLoopFunctions::Init(TConfigurationNode& t_tree) {
   auto& cObserver = dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("observer"));
   m_pcController = &dynamic_cast<CCameraTestController&>(
      cObserver.GetControllableEntity().GetController());
   GetNodeAttribute(t_tree, "near_lamp_box", m_strNearBox);
   GetNodeAttribute(t_tree, "far_box", m_strFarBox);
   GetNodeAttribute(t_tree, "min_near_luminance", m_fMinNear);
   GetNodeAttribute(t_tree, "max_near_luminance", m_fMaxNear);
   GetNodeAttribute(t_tree, "max_far_luminance", m_fMaxFar);
}

/****************************************/
/****************************************/

void CLightsLoopFunctions::ResolveBoxIds() {
   /* Not in Init(): loop functions are initialized before the media,
    * so the render scene (and with it the segmentation ids) does not
    * exist yet at that point */
   auto& cMedium = CSimulator::GetInstance().GetMedium<CPhotorealismMedium>("pr");
   CPRSceneSync& cSync = cMedium.GetSceneSync();
   m_unNearId = cSync.GetEntityId(
      dynamic_cast<CBoxEntity&>(GetSpace().GetEntity(m_strNearBox)).GetEmbodiedEntity());
   m_unFarId = cSync.GetEntityId(
      dynamic_cast<CBoxEntity&>(GetSpace().GetEntity(m_strFarBox)).GetEmbodiedEntity());
   if(m_unNearId == 0 || m_unFarId == 0) {
      THROW_ARGOSEXCEPTION("The test boxes have no segmentation id, so their "
                           "pixels cannot be located");
   }
}

/****************************************/
/****************************************/

Real CLightsLoopFunctions::MeanLuminance(UInt16 un_entity_id,
                                         size_t& un_pixels) const {
   const CCI_PhotorealisticCameraSensor::SFrame& sFrame =
      m_pcController->m_sLastFrame;
   Real fSum = 0.0;
   un_pixels = 0;
   for(size_t i = 0; i < sFrame.EntityId.size(); ++i) {
      if(sFrame.EntityId[i] != un_entity_id) {
         continue;
      }
      /* Rec. 601 luma over the 8-bit sRGB the sensor delivers */
      fSum += (0.299 * Real(sFrame.RGB[3 * i]) +
               0.587 * Real(sFrame.RGB[3 * i + 1]) +
               0.114 * Real(sFrame.RGB[3 * i + 2])) / 255.0;
      ++un_pixels;
   }
   return un_pixels > 0 ? fSum / Real(un_pixels) : 0.0;
}

/****************************************/
/****************************************/

void CLightsLoopFunctions::PostStep() {
   if(GetSpace().GetSimulationClock() < 3 || !m_pcController->m_bHasFrame) {
      return;
   }
   if(m_unNearId == 0) {
      ResolveBoxIds();
   }
   size_t unNearPixels = 0, unFarPixels = 0;
   Real fNear = MeanLuminance(m_unNearId, unNearPixels);
   Real fFar = MeanLuminance(m_unFarId, unFarPixels);
   /* Both boxes must be in view, or the comparison means nothing */
   if(unNearPixels < 50 || unFarPixels < 50) {
      THROW_ARGOSEXCEPTION("The two test boxes are not both in view ("
                           << unNearPixels << " and " << unFarPixels
                           << " pixels); the brightness comparison needs both");
   }
   if(fNear < m_fMinNear) {
      THROW_ARGOSEXCEPTION("The box under the lamp has mean luminance "
                           << fNear << ", below the expected " << m_fMinNear
                           << ": the <point> light is not lighting it");
   }
   if(fNear > m_fMaxNear) {
      THROW_ARGOSEXCEPTION("The box under the lamp has mean luminance "
                           << fNear << ", above the allowed " << m_fMaxNear
                           << ": at this exposure the lamp should not register");
   }
   if(fFar > m_fMaxFar) {
      THROW_ARGOSEXCEPTION("The box outside the lamp's falloff has mean "
                           "luminance " << fFar << ", above the allowed "
                           << m_fMaxFar << ": the light is not local");
   }
   LOG << "[TEST] mean luminance: under the lamp " << fNear
       << ", outside its reach " << fFar << std::endl;
   m_bChecked = true;
}

/****************************************/
/****************************************/

void CLightsLoopFunctions::PostExperiment() {
   if(!m_bChecked) {
      THROW_ARGOSEXCEPTION("The lighting check never ran: no camera frame "
                           "was delivered");
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CLightsLoopFunctions, "lights_loop_functions");
