/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_mesh_asset.h>
 *
 * Loads a glTF 2.0 asset (.glb or .gltf) and cooks it into a Jolt
 * triangle-mesh shape, caching the result so that the same file
 * declared several times in an arena is cooked only once.
 *
 * Axis convention: glTF is Y-up (the specification mandates it),
 * ARGoS and Jolt are Z-up. Unlike the photorealism <scenery><prop>
 * path, which does not convert and expects the user to compensate
 * with orientation="0,0,90", this loader converts by default:
 *
 *    (x, y, z)_glTF  ->  (x, -z, y)_ARGoS
 *
 * which is a rotation of +90 degrees about X and therefore preserves
 * triangle winding. Set y_up="false" on the <mesh> entity for assets
 * that are already exported Z-up. The entity's own position and
 * orientation are applied to the Jolt body, not baked into the shape,
 * so they do not defeat the shape cache.
 *
 * Jolt triangles are single-sided for simulation: a body only collides
 * with the face the winding points at. When double-sided loading is
 * requested (the default) every triangle is emitted twice, once as read
 * and once with the two last indices swapped, so contact works whatever
 * the winding of the source asset is, at the cost of twice the
 * triangles. The two windings are cooked into two mesh shapes held by a
 * compound, not into one mesh shape, so that neither of them sees the
 * other as a back-to-back neighbour. Ray casts are two-sided either
 * way.
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#ifndef JOLT_MESH_ASSET_H
#define JOLT_MESH_ASSET_H

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_common.h>

#include <argos3/core/utility/datatypes/datatypes.h>

#include <string>

namespace argos {

   struct SJoltMeshAsset {
      /** The cooked shape, wrapped in a scaled shape when scale != 1 */
      JPH::RefConst<JPH::Shape> Shape;
      /** Absolute path the asset was read from */
      std::string Path;
      /** Vertex and triangle counts after loading, before Jolt's
       *  sanitation; the triangle count includes the flipped copies
       *  when the mesh was loaded double-sided */
      size_t Vertices = 0;
      size_t Triangles = 0;
      /** True when this call cooked the mesh, false when it hit the cache */
      bool Cooked = false;
      /** Seconds spent reading and cooking, 0 on a cache hit */
      double CookSeconds = 0.0;
   };

   class CJoltMeshAsset {

   public:

      /**
       * Returns the cooked shape for the given asset, loading and
       * cooking it on first use. The unscaled mesh shape is cached on
       * (absolute path, y_up, double_sided); a non-unit scale wraps the
       * cached shape in a JPH::ScaledShape rather than re-cooking it.
       * @param str_file path to the .glb/.gltf file, absolute or
       *        relative to the working directory or to the directory
       *        of the .argos experiment file
       * @param b_y_up whether the asset is Y-up and must be converted
       * @param b_double_sided whether to emit every triangle a second
       *        time with flipped winding, so that contact works
       *        regardless of how the asset is wound
       * @param f_scale uniform scale factor, must be positive
       * @throws CARGoSException if the file cannot be read or cooked
       */
      static SJoltMeshAsset Request(const std::string& str_file,
                                    bool b_y_up,
                                    bool b_double_sided,
                                    Real f_scale);

   };

}

#endif
