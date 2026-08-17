/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_debug_draw.h>
 *
 * Line and point overlays on the photorealistic scene: planner paths,
 * graphs, frontiers, anything a loop function wants to annotate the
 * world with.
 *
 * Overlays are annotation, not scenery, and the distinction is
 * enforced rather than left to discipline. They live on their own
 * Filament visibility layer, which only the interactive viewer
 * enables, so a camera sensor cannot photograph them and a lidar
 * cannot range to them. Drawing a robot's planned path into the world
 * it is sensing would otherwise feed the planner its own output as an
 * obstacle.
 *
 * Geometry is rebuilt from scratch whenever it changes: overlays are
 * small (thousands of lines, against a scene of millions of
 * triangles), they change wholesale every planning cycle rather than
 * incrementally, and rebuilding avoids keeping a parallel structure
 * consistent with the Filament buffers. Nothing is uploaded on ticks
 * where nothing was drawn.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_DEBUG_DRAW_H
#define PR_DEBUG_DRAW_H

namespace argos {
   class CPRRenderEngine;
}

namespace filament {
   class Material;
   class MaterialInstance;
   class VertexBuffer;
   class IndexBuffer;
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector3.h>

#include <utils/Entity.h>

#include <vector>

namespace argos {

   /** Filament layer bit the overlays live on. Layer 0 is everything
    *  else; a view has to opt in to see this one. */
   inline constexpr UInt8 PR_OVERLAY_LAYER = 0x02;

   class CPRDebugDraw {

   public:

      /** r, g, b, a in [0, 1] */
      struct SColor {
         float R = 1.0f, G = 1.0f, B = 1.0f, A = 1.0f;
      };

      void Init(CPRRenderEngine& c_engine);

      void Destroy();

      /** Drops every overlay. Called by a loop function before it
       *  redraws, and by Reset(). */
      void Clear();

      /** One line segment. */
      void AddLine(const CVector3& c_from, const CVector3& c_to,
                   const SColor& s_color);

      /** A polyline through the given points; nothing is drawn for
       *  fewer than two. */
      void AddPolyline(const std::vector<CVector3>& vec_points,
                       const SColor& s_color);

      /** An axis-aligned cross, for marking a position. Points rather
       *  than lines would need a material with point size and would
       *  not scale with the scene. */
      void AddMarker(const CVector3& c_at, Real f_size,
                     const SColor& s_color);

      /** Uploads whatever has been added since the last call. Must run
       *  on the render thread, before the frame. */
      void Commit();

      /** Number of line segments currently held. */
      inline size_t GetNumLines() const {
         return m_vecVertices.size() / 2;
      }

   private:

      void ReleaseGeometry();

      struct SVertex {
         float Position[3];
         /** Linear RGBA, as the material's COLOR attribute */
         float Color[4];
      };

      CPRRenderEngine* m_pcEngine = nullptr;
      filament::Material* m_pcMaterial = nullptr;
      filament::MaterialInstance* m_pcMaterialInstance = nullptr;
      utils::Entity m_cRenderable;
      filament::VertexBuffer* m_pcVertices = nullptr;
      filament::IndexBuffer* m_pcIndices = nullptr;
      bool m_bHasGeometry = false;
      /** Set when the vertex list changed and Commit() has work */
      bool m_bDirty = false;
      std::vector<SVertex> m_vecVertices;

   };

}

#endif
