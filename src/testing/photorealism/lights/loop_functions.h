#ifndef PHOTOREALISM_LIGHTS_LOOP_FUNCTIONS_H
#define PHOTOREALISM_LIGHTS_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

#include <string>

class CCameraTestController;

using namespace argos;

/**
 * Checks the two halves of local lighting: that a <point> light lights
 * the geometry near it, and that it lights only the geometry near it.
 *
 * The scene has no sun and no ambient light at all, and two identical
 * boxes the same distance from the camera. One stands under a lamp,
 * the other is beyond the lamp's falloff radius. The first must come
 * out bright and the second must stay black: a light that leaks
 * everywhere (or an ambient term standing in for it) would light both,
 * and would look perfectly plausible in a single screenshot.
 *
 * The same scene is also run at the default sunny-16 exposure, where
 * the box under the lamp must NOT register either. That is what pins
 * the <exposure> node down: a node that is parsed and then dropped
 * would leave the cameras at sunny-16, and the resulting black frames
 * are easy to misread as broken lights rather than as an exposure
 * that was never applied.
 *
 * Box pixels are located through the segmentation entity ids rather
 * than by guessing at image coordinates, so the check cannot quietly
 * start measuring the background if the framing changes.
 */
class CLightsLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();

private:

   /** Looks up the boxes' segmentation ids; only valid once the
    *  medium has built the render scene */
   void ResolveBoxIds();

   /** Mean luminance in [0,1] over the pixels carrying un_entity_id */
   Real MeanLuminance(UInt16 un_entity_id, size_t& un_pixels) const;

   CCameraTestController* m_pcController = nullptr;
   std::string m_strNearBox;
   std::string m_strFarBox;
   UInt16 m_unNearId = 0;
   UInt16 m_unFarId = 0;
   /* Acceptance window for the box under the lamp, and the ceiling
    * for the box outside the lamp's reach; all from the XML */
   Real m_fMinNear = 0.0;
   Real m_fMaxNear = 1.0;
   Real m_fMaxFar = 1.0;
   bool m_bChecked = false;

};

#endif
