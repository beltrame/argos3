/**
 * @file <argos3/plugins/simulator/photorealism/pr_overlay.h>
 *
 * Debug line overlays, for code outside the photorealism plugin.
 *
 * This header deliberately mentions no Filament type, because the installed
 * ARGoS ships its own headers but not Filament's: anything that includes
 * photorealism_medium.h from outside the ARGoS source tree fails on a missing
 * filament/Box.h, pulled in transitively by the render-core headers. A loop
 * function in another project therefore has no way to reach the overlay API,
 * which is exactly the audience it exists for.
 *
 * So the surface here is ARGoS types only. Include this, link
 * argos3plugin_simulator_photorealism, and draw.
 *
 * Overlays are annotation, not scenery. They are visible only in the
 * interactive viewer, never to a camera or a lidar: see pr_debug_draw.h.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_OVERLAY_H
#define PR_OVERLAY_H

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/vector3.h>

#include <string>
#include <vector>

namespace argos {

   /** What a loop function may do to the overlay. */
   class CPROverlay {

   public:

      virtual ~CPROverlay() {}

      /** Drops every overlay. Call before redrawing: overlays are meant to
       *  be rebuilt wholesale, so that geometry which is no longer redrawn
       *  stops being shown rather than lingering. */
      virtual void Clear() = 0;

      virtual void AddLine(const CVector3& c_from, const CVector3& c_to,
                           const CColor& c_color) = 0;

      /**
       * A line with real width, in metres.
       *
       * Filament draws the LINES primitive one pixel wide and offers no line
       * width, so anything that has to be legible over a photorealistic scene
       * needs actual geometry. This builds each segment as two thin quads
       * crossed at right angles, which keeps a similar apparent thickness from
       * any viewing angle without having to be rebuilt when the camera moves.
       *
       * Costs 12 vertices a segment against 2, so it is for the handful of
       * things being watched - a path, a frontier - and not for a graph with
       * thousands of edges.
       */
      virtual void AddThickLine(const CVector3& c_from, const CVector3& c_to,
                                Real f_width, const CColor& c_color) = 0;

      /** A polyline of AddThickLine segments. */
      virtual void AddThickPolyline(const std::vector<CVector3>& vec_points,
                                    Real f_width, const CColor& c_color) = 0;

      /** A polyline through the points; nothing for fewer than two. */
      virtual void AddPolyline(const std::vector<CVector3>& vec_points,
                               const CColor& c_color) = 0;

      /** An axis-aligned cross, for marking a position. */
      virtual void AddMarker(const CVector3& c_at, Real f_size,
                             const CColor& c_color) = 0;

      /** Line segments currently held. */
      virtual size_t GetNumLines() const = 0;

   };

   /**
    * The overlay of the <photorealism> medium with the given id.
    *
    * Throws if no such medium exists. The medium owns the overlay; the
    * reference stays valid for the run.
    */
   CPROverlay& GetPhotorealismOverlay(const std::string& str_medium_id);

}

#endif
