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

   void CPRDebugDraw::ReleaseGeometry() {
      if(m_pcEngine == nullptr) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      if(m_bHasGeometry) {
         m_pcEngine->GetScene().remove(m_cRenderable);
         cEngine.getRenderableManager().destroy(m_cRenderable);
         utils::EntityManager::get().destroy(m_cRenderable);
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

   void CPRDebugDraw::Destroy() {
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
      m_pcEngine = nullptr;
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::Clear() {
      if(!m_vecVertices.empty()) {
         m_vecVertices.clear();
         m_bDirty = true;
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddLine(const CVector3& c_from, const CVector3& c_to,
                              const CColor& c_color) {
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
      for(size_t i = 1; i < vec_points.size(); ++i) {
         AddLine(vec_points[i - 1], vec_points[i], c_color);
      }
   }

   /****************************************/
   /****************************************/

   void CPRDebugDraw::AddMarker(const CVector3& c_at, Real f_size,
                                const CColor& c_color) {
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

   void CPRDebugDraw::Commit() {
      if(m_pcEngine == nullptr || !m_bDirty) {
         return;
      }
      m_bDirty = false;
      ReleaseGeometry();
      if(m_vecVertices.empty()) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      const auto unVertices = UInt32(m_vecVertices.size());

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
         const SVertex& sVertex = m_vecVertices[i];
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

      m_pcVertices = filament::VertexBuffer::Builder()
         .vertexCount(unVertices)
         .bufferCount(2)
         .attribute(filament::VertexAttribute::POSITION, 0,
                    filament::VertexBuffer::AttributeType::FLOAT3)
         .attribute(filament::VertexAttribute::COLOR, 1,
                    filament::VertexBuffer::AttributeType::FLOAT4)
         .build(cEngine);
      m_pcVertices->setBufferAt(
         cEngine, 0,
         filament::VertexBuffer::BufferDescriptor(
            pfPositions, size_t(unVertices) * 3 * sizeof(float),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<float*>(pt_buffer);
            }));
      m_pcVertices->setBufferAt(
         cEngine, 1,
         filament::VertexBuffer::BufferDescriptor(
            pfColors, size_t(unVertices) * 4 * sizeof(float),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<float*>(pt_buffer);
            }));
      m_pcIndices = filament::IndexBuffer::Builder()
         .indexCount(unVertices)
         .bufferType(filament::IndexBuffer::IndexType::UINT)
         .build(cEngine);
      m_pcIndices->setBuffer(
         cEngine,
         filament::IndexBuffer::BufferDescriptor(
            punIndices, size_t(unVertices) * sizeof(UInt32),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<UInt32*>(pt_buffer);
            }));

      m_cRenderable = utils::EntityManager::get().create();
      filament::RenderableManager::Builder(1)
         .boundingBox({{fMin[0], fMin[1], fMin[2]}, {fMax[0], fMax[1], fMax[2]}})
         .material(0, m_pcMaterialInstance)
         .geometry(0, filament::RenderableManager::PrimitiveType::LINES,
                   m_pcVertices, m_pcIndices)
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
         .build(cEngine, m_cRenderable);
      /* Overlays are given in world coordinates and never move as a
       * body, so the transform is identity and never updated */
      filament::TransformManager& cTransforms = cEngine.getTransformManager();
      cTransforms.create(m_cRenderable);
      m_pcEngine->GetScene().addEntity(m_cRenderable);
      m_bHasGeometry = true;
   }

}
