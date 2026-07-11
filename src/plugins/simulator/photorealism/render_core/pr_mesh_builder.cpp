/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_mesh_builder.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_mesh_builder.h"

#include <argos3/core/utility/configuration/argos_exception.h>

#include <filament/Engine.h>
#include <filament/VertexBuffer.h>
#include <filament/IndexBuffer.h>
#include <geometry/SurfaceOrientation.h>
#include <math/quat.h>
#include <math/vec3.h>

#include <cmath>
#include <cstring>
#include <vector>

namespace argos {

   using filament::math::float3;
   using filament::math::quatf;

   /****************************************/
   /****************************************/

   void SPRMesh::Release(filament::Engine& c_engine) {
      if(Vertices != nullptr) {
         c_engine.destroy(Vertices);
         Vertices = nullptr;
      }
      if(Indices != nullptr) {
         c_engine.destroy(Indices);
         Indices = nullptr;
      }
   }

   /****************************************/
   /****************************************/

   SPRMesh CPRMeshBuilder::Upload(filament::Engine& c_engine,
                                  const float* pf_positions,
                                  const float* pf_normals,
                                  UInt32 un_vertices,
                                  const UInt16* pun_indices,
                                  UInt32 un_indices,
                                  const float* pf_uvs) {
      /* Compute tangent-frame quaternions from the normals, as
       * required by lit materials */
      std::vector<float3> vecNormals(un_vertices);
      ::memcpy(vecNormals.data(), pf_normals, size_t(un_vertices) * sizeof(float3));
      filament::geometry::SurfaceOrientation* pcOrientation =
         filament::geometry::SurfaceOrientation::Builder()
            .vertexCount(un_vertices)
            .normals(vecNormals.data())
            .build();
      if(pcOrientation == nullptr) {
         THROW_ARGOSEXCEPTION("Failed to compute surface orientation for procedural mesh");
      }
      /* Buffers handed to Filament must outlive the async upload; heap
       * copies freed in the buffer-descriptor callbacks */
      auto* pfPositions = new float[size_t(un_vertices) * 3];
      ::memcpy(pfPositions, pf_positions, size_t(un_vertices) * 3 * sizeof(float));
      auto* pcQuats = new quatf[un_vertices];
      pcOrientation->getQuats(pcQuats, un_vertices);
      delete pcOrientation;
      auto* punIndices = new UInt16[un_indices];
      ::memcpy(punIndices, pun_indices, size_t(un_indices) * sizeof(UInt16));

      SPRMesh sMesh;
      filament::VertexBuffer::Builder cVertexBuilder;
      cVertexBuilder
         .vertexCount(un_vertices)
         .bufferCount(pf_uvs != nullptr ? 3 : 2)
         .attribute(filament::VertexAttribute::POSITION, 0,
                    filament::VertexBuffer::AttributeType::FLOAT3)
         .attribute(filament::VertexAttribute::TANGENTS, 1,
                    filament::VertexBuffer::AttributeType::FLOAT4);
      if(pf_uvs != nullptr) {
         cVertexBuilder.attribute(filament::VertexAttribute::UV0, 2,
                                  filament::VertexBuffer::AttributeType::FLOAT2);
      }
      sMesh.Vertices = cVertexBuilder.build(c_engine);
      sMesh.Vertices->setBufferAt(
         c_engine, 0,
         filament::VertexBuffer::BufferDescriptor(
            pfPositions, size_t(un_vertices) * 3 * sizeof(float),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<float*>(pt_buffer);
            }));
      sMesh.Vertices->setBufferAt(
         c_engine, 1,
         filament::VertexBuffer::BufferDescriptor(
            pcQuats, size_t(un_vertices) * sizeof(quatf),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<quatf*>(pt_buffer);
            }));
      if(pf_uvs != nullptr) {
         auto* pfUVs = new float[size_t(un_vertices) * 2];
         ::memcpy(pfUVs, pf_uvs, size_t(un_vertices) * 2 * sizeof(float));
         sMesh.Vertices->setBufferAt(
            c_engine, 2,
            filament::VertexBuffer::BufferDescriptor(
               pfUVs, size_t(un_vertices) * 2 * sizeof(float),
               [](void* pt_buffer, size_t, void*) {
                  delete[] static_cast<float*>(pt_buffer);
               }));
      }
      sMesh.Indices = filament::IndexBuffer::Builder()
         .indexCount(un_indices)
         .bufferType(filament::IndexBuffer::IndexType::USHORT)
         .build(c_engine);
      sMesh.Indices->setBuffer(
         c_engine,
         filament::IndexBuffer::BufferDescriptor(
            punIndices, size_t(un_indices) * sizeof(UInt16),
            [](void* pt_buffer, size_t, void*) {
               delete[] static_cast<UInt16*>(pt_buffer);
            }));
      /* Axis-aligned bounding box from the positions */
      float3 cMin(pf_positions[0], pf_positions[1], pf_positions[2]);
      float3 cMax = cMin;
      for(UInt32 i = 1; i < un_vertices; ++i) {
         const float* pfVertex = pf_positions + size_t(i) * 3;
         cMin = min(cMin, float3(pfVertex[0], pfVertex[1], pfVertex[2]));
         cMax = max(cMax, float3(pfVertex[0], pfVertex[1], pfVertex[2]));
      }
      sMesh.Aabb = filament::Box().set(cMin, cMax);
      return sMesh;
   }

   /****************************************/
   /****************************************/

   SPRMesh CPRMeshBuilder::BuildBox(filament::Engine& c_engine) {
      /* 6 faces x 4 vertices, flat normals; x,y in [-0.5,0.5], z in [0,1] */
      static const float pfP[] = {
         /* +x */  0.5f,-0.5f,0.f,  0.5f, 0.5f,0.f,  0.5f, 0.5f,1.f,  0.5f,-0.5f,1.f,
         /* -x */ -0.5f, 0.5f,0.f, -0.5f,-0.5f,0.f, -0.5f,-0.5f,1.f, -0.5f, 0.5f,1.f,
         /* +y */  0.5f, 0.5f,0.f, -0.5f, 0.5f,0.f, -0.5f, 0.5f,1.f,  0.5f, 0.5f,1.f,
         /* -y */ -0.5f,-0.5f,0.f,  0.5f,-0.5f,0.f,  0.5f,-0.5f,1.f, -0.5f,-0.5f,1.f,
         /* +z */ -0.5f,-0.5f,1.f,  0.5f,-0.5f,1.f,  0.5f, 0.5f,1.f, -0.5f, 0.5f,1.f,
         /* -z */ -0.5f, 0.5f,0.f,  0.5f, 0.5f,0.f,  0.5f,-0.5f,0.f, -0.5f,-0.5f,0.f
      };
      static const float pfN[] = {
          1,0,0,  1,0,0,  1,0,0,  1,0,0,
         -1,0,0, -1,0,0, -1,0,0, -1,0,0,
          0,1,0,  0,1,0,  0,1,0,  0,1,0,
          0,-1,0, 0,-1,0, 0,-1,0, 0,-1,0,
          0,0,1,  0,0,1,  0,0,1,  0,0,1,
          0,0,-1, 0,0,-1, 0,0,-1, 0,0,-1
      };
      std::vector<UInt16> vecIndices;
      for(UInt16 f = 0; f < 6; ++f) {
         UInt16 unBase = f * 4;
         const UInt16 punFace[] = {
            unBase, UInt16(unBase + 1), UInt16(unBase + 2),
            unBase, UInt16(unBase + 2), UInt16(unBase + 3)
         };
         vecIndices.insert(vecIndices.end(), punFace, punFace + 6);
      }
      return Upload(c_engine, pfP, pfN, 24, vecIndices.data(), vecIndices.size());
   }

   /****************************************/
   /****************************************/

   SPRMesh CPRMeshBuilder::BuildCylinder(filament::Engine& c_engine,
                                         UInt32 un_segments) {
      std::vector<float> vecPositions;
      std::vector<float> vecNormals;
      std::vector<UInt16> vecIndices;
      auto AddVertex = [&](float f_x, float f_y, float f_z,
                           float f_nx, float f_ny, float f_nz) {
         vecPositions.insert(vecPositions.end(), { f_x, f_y, f_z });
         vecNormals.insert(vecNormals.end(), { f_nx, f_ny, f_nz });
         return UInt16(vecPositions.size() / 3 - 1);
      };
      /* Side: smooth normals, two rings */
      for(UInt32 i = 0; i <= un_segments; ++i) {
         float fAngle = 2.0f * float(M_PI) * i / un_segments;
         float fCos = std::cos(fAngle);
         float fSin = std::sin(fAngle);
         AddVertex(fCos, fSin, 0.0f, fCos, fSin, 0.0f);
         AddVertex(fCos, fSin, 1.0f, fCos, fSin, 0.0f);
      }
      for(UInt32 i = 0; i < un_segments; ++i) {
         UInt16 unBase = i * 2;
         const UInt16 punQuad[] = {
            unBase, UInt16(unBase + 2), UInt16(unBase + 3),
            unBase, UInt16(unBase + 3), UInt16(unBase + 1)
         };
         vecIndices.insert(vecIndices.end(), punQuad, punQuad + 6);
      }
      /* Caps: flat normals, fan around a center vertex */
      for(UInt32 unCap = 0; unCap < 2; ++unCap) {
         float fZ = float(unCap);
         float fNz = (unCap == 0) ? -1.0f : 1.0f;
         UInt16 unCenter = AddVertex(0.0f, 0.0f, fZ, 0.0f, 0.0f, fNz);
         UInt16 unFirst = 0;
         for(UInt32 i = 0; i <= un_segments; ++i) {
            float fAngle = 2.0f * float(M_PI) * i / un_segments;
            UInt16 unVertex = AddVertex(std::cos(fAngle), std::sin(fAngle), fZ,
                                        0.0f, 0.0f, fNz);
            if(i == 0) {
               unFirst = unVertex;
            }
            else {
               if(unCap == 0) {
                  const UInt16 punTri[] = { unCenter, unVertex, UInt16(unVertex - 1) };
                  vecIndices.insert(vecIndices.end(), punTri, punTri + 3);
               }
               else {
                  const UInt16 punTri[] = { unCenter, UInt16(unVertex - 1), unVertex };
                  vecIndices.insert(vecIndices.end(), punTri, punTri + 3);
               }
            }
         }
      }
      return Upload(c_engine,
                    vecPositions.data(), vecNormals.data(),
                    vecPositions.size() / 3,
                    vecIndices.data(), vecIndices.size());
   }

   /****************************************/
   /****************************************/

   SPRMesh CPRMeshBuilder::BuildPlane(filament::Engine& c_engine) {
      static const float pfP[] = {
         -0.5f,-0.5f,0.f,  0.5f,-0.5f,0.f,  0.5f, 0.5f,0.f, -0.5f, 0.5f,0.f
      };
      static const float pfN[] = {
         0,0,1, 0,0,1, 0,0,1, 0,0,1
      };
      static const float pfUV[] = {
         0,0, 1,0, 1,1, 0,1
      };
      static const UInt16 punI[] = { 0, 1, 2, 0, 2, 3 };
      return Upload(c_engine, pfP, pfN, 4, punI, 6, pfUV);
   }

   /****************************************/
   /****************************************/

}
