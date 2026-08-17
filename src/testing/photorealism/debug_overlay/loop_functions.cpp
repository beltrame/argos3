#include "loop_functions.h"
#include "../camera_sensor/controller.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/photorealism/photorealism_medium.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_debug_draw.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <algorithm>
#include <cmath>

/****************************************/
/****************************************/

void CDebugOverlayLoopFunctions::Init(TConfigurationNode& t_tree) {
   std::string strMedium("pr");
   GetNodeAttributeOrDefault(t_tree, "medium", strMedium, strMedium);
   m_pcMedium = &CSimulator::GetInstance().GetMedium<CPhotorealismMedium>(strMedium);
   GetNodeAttributeOrDefault(t_tree, "overlay_distance",
                             m_fOverlayDistance, m_fOverlayDistance);
   GetNodeAttributeOrDefault(t_tree, "draw_at_tick", m_unDrawAtTick, m_unDrawAtTick);
   GetNodeAttributeOrDefault(t_tree, "enable_overlay",
                             m_bEnableOverlay, m_bEnableOverlay);
}

/****************************************/
/****************************************/

void CDebugOverlayLoopFunctions::PostStep() {
   const UInt32 unTick = UInt32(GetSpace().GetSimulationClock());
   CPRDebugDraw& cDraw = m_pcMedium->GetDebugDraw();

   /* From draw_at_tick onwards, a dense wall of overlay lines sits between
    * the camera and the real wall. Nothing else in the scene changes: the
    * robot never moves and the light is fixed. So every depth pixel must
    * stay exactly what it was before the overlay existed - not close, the
    * same bits - and any difference at all is the overlay leaking into the
    * sensor. Comparing against a baseline rather than against an expected
    * distance avoids having to reason about which real surface happens to
    * be nearest, which is the floor and is nearer than the overlay. */
   if(unTick == m_unDrawAtTick && m_bEnableOverlay) {
      const CPRDebugDraw::SColor sRed{1.0f, 0.0f, 0.0f, 1.0f};
      for(int i = -20; i <= 20; ++i) {
         const Real fOffset = Real(i) * 0.02;
         cDraw.AddLine(CVector3(m_fOverlayDistance, fOffset, 0.0),
                       CVector3(m_fOverlayDistance, fOffset, 1.0), sRed);
         cDraw.AddLine(CVector3(m_fOverlayDistance, -0.4, 0.25 + fOffset),
                       CVector3(m_fOverlayDistance, 0.4, 0.25 + fOffset), sRed);
      }
      if(cDraw.GetNumLines() != 82) {
         THROW_ARGOSEXCEPTION("Expected 82 overlay lines, the draw API holds "
                              << cDraw.GetNumLines());
      }
      LOG << "[OVERLAY] drew " << cDraw.GetNumLines() << " lines at "
          << m_fOverlayDistance << " m, in front of the wall" << std::endl;
      LOG.Flush();
   }

   auto& cObserver = dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("fb"));
   auto& cController = dynamic_cast<CCameraTestController&>(
      cObserver.GetControllableEntity().GetController());
   if(!cController.m_bHasFrame) {
      return;
   }
   const CCI_PhotorealisticCameraSensor::SFrame& sFrame = cController.m_sLastFrame;
   if(sFrame.Depth.empty()) {
      THROW_ARGOSEXCEPTION("The camera produced no depth");
   }

   /* Give the overlay two ticks to be committed and rendered before trusting
    * a frame to be "after": the medium commits geometry at the start of the
    * next Update, and the pool may pipeline a frame on top of that. */
   const bool bAfter = unTick > m_unDrawAtTick + 1;
   if(!bAfter) {
      m_vecBaselineDepth = sFrame.Depth;
      m_vecBaselineRGB = sFrame.RGB;
      m_bHaveBaseline = true;
      return;
   }
   if(!m_bHaveBaseline) {
      THROW_ARGOSEXCEPTION("No baseline frame was captured before the overlay "
                           "was drawn");
   }
   if(sFrame.Depth.size() != m_vecBaselineDepth.size()) {
      THROW_ARGOSEXCEPTION("Frame size changed mid-run");
   }
   size_t unDiffering = 0;
   Real fWorst = 0.0;
   for(size_t i = 0; i < sFrame.Depth.size(); ++i) {
      const Real fDelta = std::abs(sFrame.Depth[i] - m_vecBaselineDepth[i]);
      if(fDelta > 0.0) {
         ++unDiffering;
         fWorst = std::max(fWorst, fDelta);
      }
   }
   ++m_unChecks;
   if(unDiffering > 0) {
      THROW_ARGOSEXCEPTION(
         unDiffering << " of " << sFrame.Depth.size() << " depth pixels changed "
         "(worst " << fWorst << " m) once the overlay was drawn at "
         << m_fOverlayDistance << " m. Overlays must be visible only to the "
         "interactive viewer, never to a sensor.");
   }
   /* RGB is the modality the visibility layer actually protects. Depth and
    * segmentation come from the id scene, which overlays are never added to,
    * so they would stay clean even with the layer mask removed; only this
    * comparison can tell whether the mask works. */
   if(sFrame.RGB.size() != m_vecBaselineRGB.size()) {
      THROW_ARGOSEXCEPTION("RGB buffer size changed mid-run");
   }
   /* RGB is not bit-reproducible frame to frame: the colour pass runs
    * temporal antialiasing, so a completely static scene still jitters by a
    * few levels per subpixel. Measured on this scene with nothing drawn at
    * all, about 5300 of 12288 subpixels move and none moves by more than a
    * handful. A leaking overlay looks nothing like that - these lines are
    * saturated red over a grey wall - so the test is on the magnitude of the
    * change, not on whether there is any. */
   size_t unLargeChanges = 0;
   int nWorstDelta = 0;
   for(size_t i = 0; i < sFrame.RGB.size(); ++i) {
      const int nDelta = std::abs(int(sFrame.RGB[i]) - int(m_vecBaselineRGB[i]));
      nWorstDelta = std::max(nWorstDelta, nDelta);
      if(nDelta > m_nRGBJitterTolerance) ++unLargeChanges;
   }
   if(unLargeChanges > 0) {
      THROW_ARGOSEXCEPTION(
         unLargeChanges << " RGB subpixels moved by more than "
         << m_nRGBJitterTolerance << " levels (worst " << nWorstDelta
         << ") once the overlay was drawn; antialiasing jitter alone stays "
         "well inside that. The overlay is being photographed: check the "
         "visibility layer on the camera views.");
   }
   const size_t unCentre = size_t(sFrame.Height / 2) * sFrame.Width + sFrame.Width / 2;
   LOG << "[OVERLAY] tick " << unTick << ": depth identical to the pre-overlay "
       << "baseline (centre " << sFrame.Depth[unCentre] << " m), RGB within "
       << "antialiasing jitter (worst " << nWorstDelta << " levels)"
       << std::endl;
   LOG.Flush();
}

/****************************************/
/****************************************/

void CDebugOverlayLoopFunctions::Destroy() {
   if(m_unChecks == 0) {
      THROW_ARGOSEXCEPTION("The overlay comparison never ran; no frame arrived "
                           "after the overlay was drawn");
   }
   LOG << "[OVERLAY] " << m_unChecks << " frames compared after drawing, none "
       << "contaminated" << std::endl;
   LOG.Flush();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDebugOverlayLoopFunctions, "debug_overlay_loop_functions");
