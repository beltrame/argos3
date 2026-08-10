/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_mesh_asset.cpp>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#include "jolt_mesh_asset.h"

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

namespace argos {

   /****************************************/
   /****************************************/

   /* One cooked, unscaled mesh shape per (path, y_up, double_sided) key */
   struct SMeshCacheEntry {
      std::string Path;
      bool YUp;
      bool DoubleSided;
      JPH::RefConst<JPH::Shape> Shape;
      size_t Vertices;
      size_t Triangles;
   };

   static std::vector<SMeshCacheEntry> m_vecMeshCache;

   /****************************************/
   /****************************************/

   /* Resolves the file relative to the working directory first, then to
    * the directory holding the .argos experiment file */
   static std::string ResolvePath(const std::string& str_file) {
      if(std::FILE* pFile = std::fopen(str_file.c_str(), "rb")) {
         std::fclose(pFile);
         return str_file;
      }
      if(!str_file.empty() && str_file[0] != '/') {
         const std::string& strExp =
            CSimulator::GetInstance().GetExperimentFileName();
         size_t unSlash = strExp.rfind('/');
         if(unSlash != std::string::npos) {
            std::string strCandidate = strExp.substr(0, unSlash + 1) + str_file;
            if(std::FILE* pFile = std::fopen(strCandidate.c_str(), "rb")) {
               std::fclose(pFile);
               return strCandidate;
            }
         }
      }
      THROW_ARGOSEXCEPTION("Mesh file \"" << str_file << "\" not found "
                           "(searched the working directory and the "
                           "experiment file directory)");
   }

   /****************************************/
   /****************************************/

   /* Reads every triangle of every mesh node of a glTF file into
    * ARGoS-frame vertices and indices */
   static void LoadGltf(const std::string& str_path,
                        bool b_y_up,
                        JPH::VertexList& vec_vertices,
                        JPH::IndexedTriangleList& vec_triangles) {
      cgltf_options tOptions;
      std::memset(&tOptions, 0, sizeof(tOptions));
      cgltf_data* ptData = nullptr;
      if(cgltf_parse_file(&tOptions, str_path.c_str(), &ptData) != cgltf_result_success) {
         THROW_ARGOSEXCEPTION("Could not parse glTF file \"" << str_path << "\"");
      }
      if(cgltf_load_buffers(&tOptions, ptData, str_path.c_str()) != cgltf_result_success) {
         cgltf_free(ptData);
         THROW_ARGOSEXCEPTION("Could not load the buffers of glTF file \""
                              << str_path << "\" (external .bin missing?)");
      }
      std::vector<cgltf_float> vecPositions;
      std::vector<cgltf_uint> vecIndices;
      for(cgltf_size unNode = 0; unNode < ptData->nodes_count; ++unNode) {
         const cgltf_node* ptNode = &ptData->nodes[unNode];
         if(ptNode->mesh == nullptr) {
            continue;
         }
         cgltf_float pfWorld[16];
         cgltf_node_transform_world(ptNode, pfWorld);
         for(cgltf_size unPrim = 0; unPrim < ptNode->mesh->primitives_count; ++unPrim) {
            const cgltf_primitive* ptPrim = &ptNode->mesh->primitives[unPrim];
            if(ptPrim->type != cgltf_primitive_type_triangles) {
               continue;
            }
            const cgltf_accessor* ptPos =
               cgltf_find_accessor(ptPrim, cgltf_attribute_type_position, 0);
            if(ptPos == nullptr || ptPos->count == 0) {
               continue;
            }
            /* Positions, transformed by the node's world matrix */
            vecPositions.resize(ptPos->count * 3);
            if(cgltf_accessor_unpack_floats(ptPos, vecPositions.data(),
                                            vecPositions.size()) != vecPositions.size()) {
               cgltf_free(ptData);
               THROW_ARGOSEXCEPTION("Could not read the POSITION accessor of \""
                                    << str_path << "\"");
            }
            const uint32_t unBase = uint32_t(vec_vertices.size());
            vec_vertices.reserve(vec_vertices.size() + ptPos->count);
            for(cgltf_size i = 0; i < ptPos->count; ++i) {
               const cgltf_float* pfV = &vecPositions[i * 3];
               /* Column-major 4x4, as glTF and cgltf store it */
               float fX = pfWorld[0] * pfV[0] + pfWorld[4] * pfV[1] + pfWorld[8]  * pfV[2] + pfWorld[12];
               float fY = pfWorld[1] * pfV[0] + pfWorld[5] * pfV[1] + pfWorld[9]  * pfV[2] + pfWorld[13];
               float fZ = pfWorld[2] * pfV[0] + pfWorld[6] * pfV[1] + pfWorld[10] * pfV[2] + pfWorld[14];
               if(b_y_up) {
                  /* +90 deg about X: glTF up (+Y) becomes ARGoS up (+Z);
                   * a rotation, so triangle winding is preserved */
                  vec_vertices.push_back(JPH::Float3(fX, -fZ, fY));
               }
               else {
                  vec_vertices.push_back(JPH::Float3(fX, fY, fZ));
               }
            }
            /* Indices; a primitive without them is an implicit triangle soup */
            if(ptPrim->indices != nullptr) {
               const cgltf_size unCount = ptPrim->indices->count;
               vecIndices.resize(unCount);
               if(cgltf_accessor_unpack_indices(ptPrim->indices, vecIndices.data(),
                                                sizeof(cgltf_uint), unCount) != unCount) {
                  cgltf_free(ptData);
                  THROW_ARGOSEXCEPTION("Could not read the index accessor of \""
                                       << str_path << "\"");
               }
               vec_triangles.reserve(vec_triangles.size() + unCount / 3);
               for(cgltf_size i = 0; i + 2 < unCount; i += 3) {
                  vec_triangles.push_back(
                     JPH::IndexedTriangle(unBase + vecIndices[i],
                                          unBase + vecIndices[i + 1],
                                          unBase + vecIndices[i + 2]));
               }
            }
            else {
               vec_triangles.reserve(vec_triangles.size() + ptPos->count / 3);
               for(cgltf_size i = 0; i + 2 < ptPos->count; i += 3) {
                  vec_triangles.push_back(
                     JPH::IndexedTriangle(unBase + uint32_t(i),
                                          unBase + uint32_t(i + 1),
                                          unBase + uint32_t(i + 2)));
               }
            }
         }
      }
      cgltf_free(ptData);
      if(vec_triangles.empty()) {
         THROW_ARGOSEXCEPTION("glTF file \"" << str_path
                              << "\" contains no triangles");
      }
   }

   /****************************************/
   /****************************************/

   /* Cooks one triangle set into a Jolt mesh shape. The settings
    * constructor sanitizes: degenerate and duplicate triangles are
    * dropped instead of aborting the cook. The arguments are taken by
    * value because Jolt moves them into the shape. */
   static JPH::RefConst<JPH::Shape> CookMesh(const std::string& str_path,
                                             JPH::VertexList vec_vertices,
                                             JPH::IndexedTriangleList vec_triangles) {
      JPH::MeshShapeSettings cSettings(std::move(vec_vertices),
                                       std::move(vec_triangles));
      JPH::Shape::ShapeResult cResult = cSettings.Create();
      if(cResult.HasError()) {
         THROW_ARGOSEXCEPTION("Error cooking the Jolt mesh shape for \""
                              << str_path << "\": "
                              << cResult.GetError().c_str());
      }
      return cResult.Get();
   }

   /****************************************/
   /****************************************/

   SJoltMeshAsset CJoltMeshAsset::Request(const std::string& str_file,
                                          bool b_y_up,
                                          bool b_double_sided,
                                          Real f_scale) {
      if(f_scale <= 0.0) {
         THROW_ARGOSEXCEPTION("Mesh scale must be positive, got " << f_scale);
      }
      SJoltMeshAsset sAsset;
      sAsset.Path = ResolvePath(str_file);
      /* Cache hit? */
      for(const SMeshCacheEntry& sEntry : m_vecMeshCache) {
         if(sEntry.Path == sAsset.Path &&
            sEntry.YUp == b_y_up &&
            sEntry.DoubleSided == b_double_sided) {
            sAsset.Shape = sEntry.Shape;
            sAsset.Vertices = sEntry.Vertices;
            sAsset.Triangles = sEntry.Triangles;
            sAsset.Cooked = false;
            break;
         }
      }
      if(sAsset.Shape == nullptr) {
         auto tStart = std::chrono::steady_clock::now();
         JPH::VertexList vecVertices;
         JPH::IndexedTriangleList vecTriangles;
         LoadGltf(sAsset.Path, b_y_up, vecVertices, vecTriangles);
         sAsset.Vertices = vecVertices.size();
         sAsset.Triangles = vecTriangles.size();
         if(!b_double_sided) {
            sAsset.Shape = CookMesh(sAsset.Path, std::move(vecVertices),
                                    std::move(vecTriangles));
         }
         else {
            /* Every triangle is emitted a second time with its winding
             * reversed, so that contact does not depend on how the
             * asset is wound. The two windings are two mesh shapes in a
             * compound rather than one mesh shape holding both, because
             * Jolt marks an edge shared by two back-to-back triangles
             * as active (Jolt/Physics/Collision/ActiveEdges.h) and a
             * body sliding over an active edge is deflected by the edge
             * normal instead of the face normal. Cooked apart, each
             * shape keeps the active-edge information its own winding
             * implies, and a contact is generated by whichever of the
             * two the body approaches from the front. */
            JPH::IndexedTriangleList vecFlipped;
            vecFlipped.reserve(vecTriangles.size());
            for(const JPH::IndexedTriangle& cTriangle : vecTriangles) {
               vecFlipped.push_back(JPH::IndexedTriangle(cTriangle.mIdx[0],
                                                         cTriangle.mIdx[2],
                                                         cTriangle.mIdx[1]));
            }
            sAsset.Triangles += vecFlipped.size();
            JPH::StaticCompoundShapeSettings cCompound;
            cCompound.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(),
                               CookMesh(sAsset.Path, vecVertices,
                                        std::move(vecTriangles)));
            cCompound.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(),
                               CookMesh(sAsset.Path, std::move(vecVertices),
                                        std::move(vecFlipped)));
            JPH::Shape::ShapeResult cResult = cCompound.Create();
            if(cResult.HasError()) {
               THROW_ARGOSEXCEPTION("Error building the double-sided Jolt mesh "
                                    "shape for \"" << sAsset.Path << "\": "
                                    << cResult.GetError().c_str());
            }
            sAsset.Shape = cResult.Get();
         }
         sAsset.Cooked = true;
         sAsset.CookSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tStart).count();
         m_vecMeshCache.push_back({sAsset.Path, b_y_up, b_double_sided,
                                   sAsset.Shape, sAsset.Vertices,
                                   sAsset.Triangles});
      }
      /* Scaling wraps the cached shape, so one asset is cooked once
       * however many times and at whatever sizes it is declared */
      if(f_scale != 1.0) {
         JPH::ScaledShapeSettings cSettings(sAsset.Shape,
                                            JPH::Vec3::sReplicate(float(f_scale)));
         JPH::Shape::ShapeResult cResult = cSettings.Create();
         if(cResult.HasError()) {
            THROW_ARGOSEXCEPTION("Error scaling the Jolt mesh shape for \""
                                 << sAsset.Path << "\": "
                                 << cResult.GetError().c_str());
         }
         sAsset.Shape = cResult.Get();
      }
      return sAsset;
   }

   /****************************************/
   /****************************************/

}
