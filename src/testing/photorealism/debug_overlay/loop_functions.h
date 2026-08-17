/*
 * Checks that debug overlays are drawn but never sensed.
 *
 * The overlay API exists so a human watching a run can see what a planner is
 * thinking. The one thing that must not happen is a robot sensing its own
 * annotation: a path drawn in front of a robot would come back as an obstacle
 * in its depth image, and the planner would route around a wall it had itself
 * just drawn.
 *
 * So this draws a dense wall of lines right across the camera's view, nearer
 * than the wall behind it, and requires every depth pixel to stay bit-identical
 * to what it was before the overlay existed. Nothing else in the scene moves,
 * so any difference at all is the overlay leaking.
 *
 * Both modalities are compared, and they are protected by different things.
 * Depth and segmentation come from a separate id scene that overlays are never
 * added to, so they are safe by construction. RGB renders the main scene, which
 * the overlays are in, and is kept clean only by the visibility layer. Checking
 * depth alone would pass whether or not the layer mask worked.
 *
 * Comparing against a baseline rather than an expected distance matters: the
 * nearest real surface is the floor, which a camera 0.15 m up with a 60 degree
 * field of view sees from 0.26 m, nearer than the overlay itself. A test
 * phrased as "nothing may be nearer than the overlay" fails on the floor and
 * says nothing about overlays.
 */

#ifndef DEBUG_OVERLAY_LOOP_FUNCTIONS_H
#define DEBUG_OVERLAY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

#include <vector>

using namespace argos;

namespace argos {
   class CPhotorealismMedium;
}

class CDebugOverlayLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void Destroy();

private:

   CPhotorealismMedium* m_pcMedium = nullptr;
   /** Distance in front of the camera to draw the overlay wall */
   Real m_fOverlayDistance = 0.4;
   /** Tick the overlay appears on; frames before it are the baseline */
   UInt32 m_unDrawAtTick = 5;
   /** Control condition: run the same comparison but draw nothing. Any
    *  difference reported then is the renderer, not the overlay. */
   bool m_bEnableOverlay = true;
   /** Per-subpixel RGB change attributable to temporal antialiasing */
   int m_nRGBJitterTolerance = 12;
   std::vector<Real> m_vecBaselineDepth;
   std::vector<UInt8> m_vecBaselineRGB;
   bool m_bHaveBaseline = false;
   UInt32 m_unChecks = 0;

};

#endif
