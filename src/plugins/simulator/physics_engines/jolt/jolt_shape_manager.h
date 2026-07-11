/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.h>
 *
 * Caches Jolt collision shapes so that entities with the same
 * dimensions share them.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_SHAPE_MANAGER_H
#define JOLT_SHAPE_MANAGER_H

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_common.h>

#include <vector>

namespace argos {

   class CJoltShapeManager {

   public:

      static JPH::RefConst<JPH::Shape> RequestBox(const JPH::Vec3& c_half_extents);

      /** A cylinder with the axis along z (ARGoS convention) */
      static JPH::RefConst<JPH::Shape> RequestCylinder(float f_half_height,
                                                       float f_radius);

   private:

      struct SBox {
         JPH::Vec3 HalfExtents;
         JPH::RefConst<JPH::Shape> Shape;
      };

      struct SCylinder {
         float HalfHeight;
         float Radius;
         JPH::RefConst<JPH::Shape> Shape;
      };

      static std::vector<SBox> m_vecBoxes;
      static std::vector<SCylinder> m_vecCylinders;

   };

}

#endif
