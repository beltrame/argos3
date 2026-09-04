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
#include <argos3/plugins/simulator/photorealism/pr_overlay.h>

#include <utils/Entity.h>

#include <mutex>
#include <vector>

namespace argos {

   /** Filament layer bit the overlays live on. Layer 0 is everything
    *  else; a view has to opt in to see this one. */
   inline constexpr UInt8 PR_OVERLAY_LAYER = 0x02;

   class CPRDebugDraw : public CPROverlay {

   public:

      void Init(CPRRenderEngine& c_engine);

      void Destroy();

      /** Drops every overlay. Called by a loop function before it
       *  redraws, and by Reset(). */
      virtual void Clear();

      /** One line segment. */
      virtual void AddLine(const CVector3& c_from, const CVector3& c_to,
                           const CColor& c_color);

      /** A polyline through the given points; nothing is drawn for
       *  fewer than two. */
      virtual void AddPolyline(const std::vector<CVector3>& vec_points,
                               const CColor& c_color);

      virtual void AddThickLine(const CVector3& c_from, const CVector3& c_to,
                                Real f_width, const CColor& c_color);

      virtual void AddThickPolyline(const std::vector<CVector3>& vec_points,
                                    Real f_width, const CColor& c_color);

      /** An axis-aligned cross, for marking a position. Points rather
       *  than lines would need a material with point size and would
       *  not scale with the scene. */
      virtual void AddMarker(const CVector3& c_at, Real f_size,
                             const CColor& c_color);

      /** Uploads whatever has been added since the last call. Must run
       *  on the render thread, before the frame. */
      void Commit();

      /** Number of line segments currently held. Thick segments are made of
       *  triangles and are not counted here. */
      virtual size_t GetNumLines() const {
         std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
         return m_vecVertices.size() / 2;
      }

   private:

      struct SVertex {
         float Position[3];
         /** Linear RGBA, as the material's COLOR attribute */
         float Color[4];

         bool operator==(const SVertex& s_other) const {
            return !(*this != s_other);
         }

         bool operator!=(const SVertex& s_other) const {
            for(UInt32 i = 0; i < 3; ++i) {
               if(Position[i] != s_other.Position[i]) return true;
            }
            for(UInt32 i = 0; i < 4; ++i) {
               if(Color[i] != s_other.Color[i]) return true;
            }
            return false;
         }
      };

      void ReleaseGeometry();
      void ReleaseLines();
      void ReleaseTriangles();
      /** Builds a renderable from a vertex list, as lines or triangles */
      void Upload(const std::vector<SVertex>& vec_vertices, bool b_triangles);

      /** Appends one triangle to the solid overlay geometry */
      void AddTriangle(const CVector3& c_a, const CVector3& c_b,
                       const CVector3& c_c, const CColor& c_color);

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
      /** What was last uploaded, so an unchanged redraw can be skipped.
       *
       *  A caller that rebuilds its overlay every tick - which is the intended
       *  usage, since anything not redrawn should disappear - would otherwise
       *  destroy and recreate the Filament buffers 10 times a second even when
       *  the geometry is identical. Uploads are asynchronous, so some frames
       *  land while the new buffers are not ready yet and the overlay flickers.
       */
      std::vector<SVertex> m_vecCommitted;
      /** Thick lines, as triangles. Kept in a second renderable because a
       *  Filament renderable draws one primitive type, and mixing the two
       *  would mean either drawing the graph as triangles (12 vertices an edge
       *  for thousands of edges) or the path as hairlines. */
      std::vector<SVertex> m_vecTriangles;
      std::vector<SVertex> m_vecTrianglesCommitted;
      utils::Entity m_cTriangleRenderable;
      filament::VertexBuffer* m_pcTriangleVertices = nullptr;
      filament::IndexBuffer* m_pcTriangleIndices = nullptr;
      bool m_bHasTriangles = false;
      mutable std::recursive_mutex m_cMutex;

   };

}

#endif
