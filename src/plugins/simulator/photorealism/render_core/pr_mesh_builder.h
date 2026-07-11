/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_mesh_builder.h>
 *
 * Procedural meshes for the primitive ARGoS entities (box, cylinder,
 * floor plane). Geometry follows the qt-opengl conventions: the unit
 * box spans x,y in [-0.5,0.5] and z in [0,1] and is scaled by the
 * entity size; the unit cylinder has radius 1 and z in [0,1] and is
 * scaled by (radius, radius, height).
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_MESH_BUILDER_H
#define PR_MESH_BUILDER_H

#include <argos3/core/utility/datatypes/datatypes.h>

#include <filament/Box.h>

namespace filament {
   class Engine;
   class VertexBuffer;
   class IndexBuffer;
}

namespace argos {

   struct SPRMesh {
      filament::VertexBuffer* Vertices = nullptr;
      filament::IndexBuffer*  Indices  = nullptr;
      filament::Box           Aabb;

      void Release(filament::Engine& c_engine);
   };

   class CPRMeshBuilder {

   public:

      static SPRMesh BuildBox(filament::Engine& c_engine);

      static SPRMesh BuildCylinder(filament::Engine& c_engine,
                                   UInt32 un_segments = 32);

      /**
       * A unit plane on z=0 spanning x,y in [-0.5,0.5], normal +z,
       * scaled by the arena size for the floor. Carries UV0 with
       * (0,0) at (-0.5,-0.5) and (1,1) at (0.5,0.5).
       */
      static SPRMesh BuildPlane(filament::Engine& c_engine);

   private:

      /**
       * Uploads positions/normals as Filament vertex buffers
       * (POSITION float3, TANGENTS quat float4 computed from normals,
       * optional UV0 float2) plus an index buffer.
       */
      static SPRMesh Upload(filament::Engine& c_engine,
                            const float* pf_positions,
                            const float* pf_normals,
                            UInt32 un_vertices,
                            const UInt16* pun_indices,
                            UInt32 un_indices,
                            const float* pf_uvs = nullptr);

   };

}

#endif
