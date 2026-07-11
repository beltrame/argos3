#ifndef PHOTOREALISM_ROBOT_VISUALS_LOOP_FUNCTIONS_H
#define PHOTOREALISM_ROBOT_VISUALS_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CFootBotEntity;
   class CPhotorealismMedium;
}

class CCameraTestController;

using namespace argos;

/**
 * An observer foot-bot looks at a target foot-bot whose LEDs are set
 * to red. Checks that the target is visible in the segmentation with
 * the foot-bot class, and that red emissive LED pixels appear in the
 * RGB image.
 */
class CRobotVisualsLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PreStep();
   virtual void PostStep();
   virtual CColor GetFloorColor(const CVector2& c_position);

private:

   CCameraTestController* m_pcController = nullptr;
   CFootBotEntity* m_pcTarget = nullptr;
   CPhotorealismMedium* m_pcMedium = nullptr;

};

#endif
