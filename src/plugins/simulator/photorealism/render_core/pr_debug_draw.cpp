#include "pr_debug_draw.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_render_engine.h>

#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>
#include <utils/EntityManager.h>

#include <cstring>
#include <limits>

namespace argos {

   /* Compiled from materials/pr_overlay.mat and embedded by the build.
    * ARGoSHexifyFile.cmake emits these inside namespace argos, so the
    * declaration has to sit here rather than at global scope. */
   extern const unsigned char PR_OVERLAY_FILAMAT[];
   extern const size_t PR_OVERLAY_FILAMAT_SIZE;

   /****************************************/
   /****************************************/

   void CPRDebugDraw::Init(CPRRenderEngine& c_engine) {
      m_pcEngine = &c_engine;
      m_pcMaterial = filament::Material::Builder()
         .package(PR_OVERLAY_FILAMAT, PR_OVERLAY_FILAMAT_SIZE)
         .build(c_engine.GetEngine());
      if(m_pcMaterial == nullptr) {
         THROW_ARGOSEXCEPTION("Failed to load the embedded pr_overlay material");
      }
      m_pcMaterialInstance = m_pcMaterial->createInstance();
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::ReleaseLines() {
      if(m_pcEngine == nullptr) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      if(m_bHasGeometry) {
         m_pcEngine->GetScene().remove(m_cRenderable);
         cEngine.getRenderableManager().destroy(m_cRenderable);
         m_pcEngine->DestroyEntity(m_cRenderable);
         m_cRenderable = utils::Entity();
         m_bHasGeometry = false;
      }
      if(m_pcVertices != nullptr) {
         cEngine.destroy(m_pcVertices);
         m_pcVertices = nullptr;
      }
      if(m_pcIndices != nullptr) {
         cEngine.destroy(m_pcIndices);
         m_pcIndices = nullptr;
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::ReleaseTriangles() {
      if(m_pcEngine == nullptr) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      if(m_bHasTriangles) {
         m_pcEngine->GetScene().remove(m_cTriangleRenderable);
         cEngine.getRenderableManager().destroy(m_cTriangleRenderable);
         m_pcEngine->DestroyEntity(m_cTriangleRenderable);
         m_cTriangleRenderable = utils::Entity();
         m_bHasTriangles = false;
      }
      if(m_pcTriangleVertices != nullptr) {
         cEngine.destroy(m_pcTriangleVertices);
         m_pcTriangleVertices = nullptr;
      }
      if(m_pcTriangleIndices != nullptr) {
         cEngine.destroy(m_pcTriangleIndices);
         m_pcTriangleIndices = nullptr;
      }
   }

   void CPRDebugDraw::ReleaseGeometry() {
      ReleaseLines();
      ReleaseTriangles();
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::Destroy() {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      ReleaseGeometry();
      if(m_pcEngine != nullptr) {
         filament::Engine& cEngine = m_pcEngine->GetEngine();
         if(m_pcMaterialInstance != nullptr) {
            cEngine.destroy(m_pcMaterialInstance);
            m_pcMaterialInstance = nullptr;
         }
         if(m_pcMaterial != nullptr) {
            cEngine.destroy(m_pcMaterial);
            m_pcMaterial = nullptr;
         }
      }
      m_vecVertices.clear();
      m_vecCommitted.clear();
      m_vecTriangles.clear();
      m_vecTrianglesCommitted.clear();
      m_pcEngine = nullptr;
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::Clear() {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      if(!m_vecVertices.empty() || !m_vecTriangles.empty()) {
         m_vecVertices.clear();
         m_vecTriangles.clear();
         m_bDirty = true;
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddLine(const CVector3& c_from, const CVector3& c_to,
                              const CColor& c_color) {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      SVertex sVertex;
      /* CColor is 8-bit sRGB-ish; the material is unlit and writes baseColor
       * straight out, so a plain 0-255 scale is what comes back on screen */
      sVertex.Color[0] = float(c_color.GetRed()) / 255.0f;
      sVertex.Color[1] = float(c_color.GetGreen()) / 255.0f;
      sVertex.Color[2] = float(c_color.GetBlue()) / 255.0f;
      sVertex.Color[3] = float(c_color.GetAlpha()) / 255.0f;
      sVertex.Position[0] = float(c_from.GetX());
      sVertex.Position[1] = float(c_from.GetY());
      sVertex.Position[2] = float(c_from.GetZ());
      m_vecVertices.push_back(sVertex);
      sVertex.Position[0] = float(c_to.GetX());
      sVertex.Position[1] = float(c_to.GetY());
      sVertex.Position[2] = float(c_to.GetZ());
      m_vecVertices.push_back(sVertex);
      m_bDirty = true;
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddPolyline(const std::vector<CVector3>& vec_points,
                                  const CColor& c_color) {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      for(size_t i = 1; i < vec_points.size(); ++i) {
         AddLine(vec_points[i - 1], vec_points[i], c_color);
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddTriangle(const CVector3& c_a, const CVector3& c_b,
                                  const CVector3& c_c, const CColor& c_color) {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      SVertex sVertex;
      sVertex.Color[0] = float(c_color.GetRed()) / 255.0f;
      sVertex.Color[1] = float(c_color.GetGreen()) / 255.0f;
      sVertex.Color[2] = float(c_color.GetBlue()) / 255.0f;
      sVertex.Color[3] = float(c_color.GetAlpha()) / 255.0f;
      const CVector3* pcCorners[3] = {&c_a, &c_b, &c_c};
      for(const CVector3* pcCorner : pcCorners) {
         sVertex.Position[0] = float(pcCorner->GetX());
         sVertex.Position[1] = float(pcCorner->GetY());
         sVertex.Position[2] = float(pcCorner->GetZ());
         m_vecTriangles.push_back(sVertex);
      }
      m_bDirty = true;
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddThickLine(const CVector3& c_from, const CVector3& c_to,
                                   Real f_width, const CColor& c_color) {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      CVector3 cAlong = c_to - c_from;
      const Real fLength = cAlong.Length();
      if(fLength < 1e-9) {
         return;
      }
      cAlong /= fLength;
      /* Two perpendiculars to the segment. The first is taken against
       * whichever world axis the segment is least aligned with, so the cross
       * product never degenerates on a vertical or axis-aligned line - which
       * is most of them, since these paths run along a lattice. */
      CVector3 cReference =
         std::abs(cAlong.GetZ()) < 0.9 ? CVector3::Z : CVector3::X;
      CVector3 cSide = cAlong;
      cSide.CrossProduct(cReference);
      cSide.Normalize();
      CVector3 cUp = cAlong;
      cUp.CrossProduct(cSide);
      cUp.Normalize();

      const Real fHalf = f_width * 0.5;
      /* Two quads crossed at right angles: a flat ribbon alone vanishes when
       * seen edge-on, which for a path lying on the ground is exactly the
       * viewpoint someone watching from above has. */
      const CVector3 pcOffsets[2] = {cSide * fHalf, cUp * fHalf};
      for(const CVector3& cOffset : pcOffsets) {
         const CVector3 cA = c_from - cOffset;
         const CVector3 cB = c_from + cOffset;
         const CVector3 cC = c_to + cOffset;
         const CVector3 cD = c_to - cOffset;
         AddTriangle(cA, cB, cC, c_color);
         AddTriangle(cA, cC, cD, c_color);
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddThickPolyline(const std::vector<CVector3>& vec_points,
                                       Real f_width, const CColor& c_color) {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      for(size_t i = 1; i < vec_points.size(); ++i) {
         AddThickLine(vec_points[i - 1], vec_points[i], f_width, c_color);
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddMarker(const CVector3& c_at, Real f_size,
                                const CColor& c_color) {
      std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
      const Real fHalf = f_size * 0.5;
      AddLine(c_at - CVector3(fHalf, 0.0, 0.0),
              c_at + CVector3(fHalf, 0.0, 0.0), c_color);
      AddLine(c_at - CVector3(0.0, fHalf, 0.0),
              c_at + CVector3(0.0, fHalf, 0.0), c_color);
      AddLine(c_at - CVector3(0.0, 0.0, fHalf),
              c_at + CVector3(0.0, 0.0, fHalf), c_color);
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::Upload(const std::vector<SVertex>& vec_vertices,
                             bool b_triangles) {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      const auto unVertices = UInt32(vec_vertices.size());

      /* Buffers handed to Filament must outlive the async upload, so
       * these heap copies are freed in the descriptor callbacks */
      auto* pfPositions = new float[size_t(unVertices) * 3];
      auto* pfColors = new float[size_t(unVertices) * 4];
      float fMin[3] = {std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max()};
      float fMax[3] = {std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest()};
      for(UInt32 i = 0; i < unVertices; ++i) {
         const SVertex& sVertex = vec_vertices[i];
         for(UInt32 a = 0; a < 3; ++a) {
            const float fValue = sVertex.Position[a];
            pfPositions[size_t(i) * 3 + a] = fValue;
            if(fValue < fMin[a]) fMin[a] = fValue;
            if(fValue > fMax[a]) fMax[a] = fValue;
         }
         ::memcpy(&pfColors[size_t(i) * 4], sVertex.Color, 4 * sizeof(float));
      }
      /* 32-bit indices: a graph of a few thousand vertices already
       * exceeds the 65535 a UInt16 buffer would allow */
      auto* punIndices = new UInt32[unVertices];
      for(UInt32 i = 0; i < unVertices; ++i) {
         punIndices[i] = i;
      }

      filament::VertexBuffer* pcVertices = filament::VertexBuffer::Builder()
         .vertexCount(unVertices)
         .bufferCount(2)
         .attribute(filament::VertexAttribute::POSITION, 0,
                    filament::VertexBuffer::AttributeType::FLOAT3)
         .attribute(filament::VertexAttribute::COLOR, 1,
                    filament::VertexBuffer::AttributeType::FLOAT4)
         .build(cEngine);
      pcVertices->setBufferAt(
         cEngine, 0,
         filament::VertexBuffer::BufferDescriptor(
            pfPositions, size_t(unVertices) * 3 * sizeof(float),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<float*>(pt_buffer);
            }));
      pcVertices->setBufferAt(
         cEngine, 1,
         filament::VertexBuffer::BufferDescriptor(
            pfColors, size_t(unVertices) * 4 * sizeof(float),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<float*>(pt_buffer);
            }));
      filament::IndexBuffer* pcIndices = filament::IndexBuffer::Builder()
         .indexCount(unVertices)
         .bufferType(filament::IndexBuffer::IndexType::UINT)
         .build(cEngine);
      pcIndices->setBuffer(
         cEngine,
         filament::IndexBuffer::BufferDescriptor(
            punIndices, size_t(unVertices) * sizeof(UInt32),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<UInt32*>(pt_buffer);
            }));

      utils::Entity cRenderable = m_pcEngine->CreateEntity();
      filament::RenderableManager::Builder(1)
         .boundingBox({{fMin[0], fMin[1], fMin[2]}, {fMax[0], fMax[1], fMax[2]}})
         .material(0, m_pcMaterialInstance)
         .geometry(0, b_triangles
                      ? filament::RenderableManager::PrimitiveType::TRIANGLES
                      : filament::RenderableManager::PrimitiveType::LINES,
                   pcVertices, pcIndices)
         /* The point of the exercise: only a view that opts into this
          * layer sees the overlays, so sensors never do.
          *
          * The select mask is 0xFF, not PR_OVERLAY_LAYER. layerMask only
          * touches the bits named in select, and a renderable defaults to
          * layer 0, which every view shows: selecting just the overlay bit
          * would leave the geometry on both layers and it would be drawn
          * everywhere. Selecting all bits clears layer 0 as it sets this
          * one. */
         .layerMask(0xFF, PR_OVERLAY_LAYER)
         .culling(false)
         .receiveShadows(false)
         .castShadows(false)
         .build(cEngine, cRenderable);
      /* Overlays are given in world coordinates and never move as a
       * body, so the transform is identity and never updated */
      filament::TransformManager& cTransforms = cEngine.getTransformManager();
      cTransforms.create(cRenderable);
      m_pcEngine->GetScene().addEntity(cRenderable);

      if(b_triangles) {
         m_cTriangleRenderable = cRenderable;
         m_pcTriangleVertices = pcVertices;
         m_pcTriangleIndices = pcIndices;
         m_bHasTriangles = true;
      }
      else {
         m_cRenderable = cRenderable;
         m_pcVertices = pcVertices;
         m_pcIndices = pcIndices;
         m_bHasGeometry = true;
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::Commit() {
      std::vector<SVertex> vecVertices;
      std::vector<SVertex> vecTriangles;
      {
         std::lock_guard<std::recursive_mutex> cLock(m_cMutex);
         if(m_pcEngine == nullptr || !m_bDirty) {
            return;
         }
         m_bDirty = false;
         vecVertices = m_vecVertices;
         vecTriangles = m_vecTriangles;
      }
      /* Nothing actually changed: keep the buffers that are already on the
       * GPU. The usual case by far, because a caller redraws the same overlay
       * every tick so that stale geometry disappears on its own, and tearing
       * the buffers down and back up for that is both wasted work and visible
       * as flicker while the asynchronous uploads catch up. The two lists are
       * compared separately so a moving path does not force the graph, which
       * is far larger and changes far less often, to be re-uploaded with it. */
      const bool bLinesSame = (vecVertices == m_vecCommitted);
      const bool bTrianglesSame = (vecTriangles == m_vecTrianglesCommitted);
      if(bLinesSame && bTrianglesSame) {
         return;
      }
      if(!bLinesSame) {
         ReleaseLines();
         if(!vecVertices.empty()) Upload(vecVertices, false);
         m_vecCommitted = std::move(vecVertices);
      }
      if(!bTrianglesSame) {
         ReleaseTriangles();
         if(!vecTriangles.empty()) Upload(vecTriangles, true);
         m_vecTrianglesCommitted = std::move(vecTriangles);
      }
   }

}
