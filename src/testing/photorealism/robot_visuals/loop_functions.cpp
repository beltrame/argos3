#include "loop_functions.h"
#include "../camera_sensor/controller.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/photorealism/photorealism_medium.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/****************************************/
/****************************************/

void CRobotVisualsLoopFunctions::Init(TConfigurationNode& t_tree) {
   auto& cObserver = dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("observer"));
   m_pcController = &dynamic_cast<CCameraTestController&>(
      cObserver.GetControllableEntity().GetController());
   m_pcTarget = &dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("target"));
   m_pcMedium = &CSimulator::GetInstance().GetMedium<CPhotorealismMedium>("pr");
}

/****************************************/
/****************************************/

void CRobotVisualsLoopFunctions::PreStep() {
   /* Keep the target's LED ring red every tick */
   m_pcTarget->GetLEDEquippedEntity().SetAllLEDsColors(CColor::RED);
}

/****************************************/
/****************************************/

void CRobotVisualsLoopFunctions::PostStep() {
   UInt32 unClock = GetSpace().GetSimulationClock();
   if(unClock < 3) {
      return;
   }
   if(!m_pcController->m_bHasFrame) {
      THROW_ARGOSEXCEPTION("No camera frame delivered by tick " << unClock);
   }
   const CCI_PhotorealisticCameraSensor::SFrame& sFrame =
      m_pcController->m_sLastFrame;
   /* The target foot-bot must appear in the segmentation with the
    * foot-bot class and its own entity id */
   UInt16 unTargetId = m_pcMedium->GetSceneSync().GetEntityId(
      m_pcTarget->GetEmbodiedEntity());
   if(unTargetId == 0) {
      THROW_ARGOSEXCEPTION("The target foot-bot has no segmentation id");
   }
   size_t unTargetPixels = 0;
   size_t unClassPixels = 0;
   for(size_t i = 0; i < sFrame.EntityId.size(); ++i) {
      if(sFrame.EntityId[i] == unTargetId) {
         ++unTargetPixels;
         if(sFrame.ClassId[i] == UInt8(EPRClass::FootBot)) {
            ++unClassPixels;
         }
      }
   }
   if(unTargetPixels < 50) {
      THROW_ARGOSEXCEPTION("Target foot-bot covers only " << unTargetPixels
                           << " pixels in the segmentation image");
   }
   if(unClassPixels != unTargetPixels) {
      THROW_ARGOSEXCEPTION("Target pixels carry the wrong class id");
   }
   /* Red LED pixels must be present in the RGB image: strongly red
    * pixels only exist thanks to the emissive LED cubes */
   size_t unRedPixels = 0;
   for(size_t i = 0; i < sFrame.RGB.size(); i += 3) {
      if(sFrame.RGB[i] > 150 &&
         sFrame.RGB[i] > sFrame.RGB[i + 1] + 60 &&
         sFrame.RGB[i] > sFrame.RGB[i + 2] + 60) {
         ++unRedPixels;
      }
   }
   if(unRedPixels < 3) {
      CLEDEquippedEntity& cLEDs = m_pcTarget->GetLEDEquippedEntity();
      std::ostringstream ossLEDs;
      ossLEDs << "n=" << cLEDs.GetLEDs().size();
      if(!cLEDs.GetLEDs().empty()) {
         ossLEDs << " led0 pos=" << cLEDs.GetLED(0).GetPosition()
                 << " color=" << cLEDs.GetLED(0).GetColor()
                 << " enabled=" << cLEDs.GetLED(0).IsEnabled();
      }
      THROW_ARGOSEXCEPTION("Expected red LED pixels in the RGB image, found "
                           << unRedPixels << " (" << ossLEDs.str() << ")");
   }
}

/****************************************/
/****************************************/

CColor CRobotVisualsLoopFunctions::GetFloorColor(const CVector2& c_position) {
   /* 0.25 m checkerboard */
   SInt32 nX = SInt32(std::floor(c_position.GetX() * 4.0));
   SInt32 nY = SInt32(std::floor(c_position.GetY() * 4.0));
   return ((nX + nY) & 1) != 0 ? CColor::WHITE : CColor::GRAY30;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CRobotVisualsLoopFunctions,
                        "robot_visuals_loop_functions");
