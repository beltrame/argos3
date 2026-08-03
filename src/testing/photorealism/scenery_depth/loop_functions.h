#ifndef PHOTOREALISM_SCENERY_DEPTH_LOOP_FUNCTIONS_H
#define PHOTOREALISM_SCENERY_DEPTH_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

class CCameraTestController;

using namespace argos;

/**
 * Checks that a <scenery> glTF prop is visible to the depth and
 * segmentation modalities, not just to the colour image.
 *
 * Scenery used to be registered with entity id 0. The camera sensor
 * reads id 0 as "no geometry here" and substitutes the far-plane
 * distance, so the props were rendered into the auxiliary buffer and
 * then discarded: every experiment whose world is a glTF environment
 * got a blank depth image and an empty segmentation mask, while colour
 * looked perfect. Nothing caught it because every other camera test
 * puts robots and boxes in front of the lens, and those carry real ids.
 */
class CSceneryDepthLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();

private:

   CCameraTestController* m_pcController = nullptr;
   /* Distance from the camera to the prop's facing surface, and the
    * tolerance to accept, both from the XML */
   Real m_fExpectedDepth = 0.0;
   Real m_fDepthTolerance = 0.0;
   Real m_fFarPlane = 0.0;
   bool m_bChecked = false;

};

#endif
