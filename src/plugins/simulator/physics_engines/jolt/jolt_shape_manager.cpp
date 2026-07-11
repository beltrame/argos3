/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_shape_manager.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_shape_manager.h"

#include <argos3/core/utility/configuration/argos_exception.h>

#include <algorithm>

namespace argos {

   /****************************************/
   /****************************************/

   std::vector<CJoltShapeManager::SBox> CJoltShapeManager::m_vecBoxes;
   std::vector<CJoltShapeManager::SCylinder> CJoltShapeManager::m_vecCylinders;

   /* Jolt's default convex radius (5 cm) is too large for robot-scale
    * geometry; keep it well below the smallest shape dimension */
   static float ConvexRadius(float f_min_dimension) {
      return std::min(JPH::cDefaultConvexRadius, 0.25f * f_min_dimension);
   }

   /****************************************/
   /****************************************/

   JPH::RefConst<JPH::Shape> CJoltShapeManager::RequestBox(const JPH::Vec3& c_half_extents) {
      for(const SBox& s_box : m_vecBoxes) {
         if(s_box.HalfExtents == c_half_extents) {
            return s_box.Shape;
         }
      }
      float fMinDimension = c_half_extents.ReduceMin();
      JPH::BoxShapeSettings cSettings(c_half_extents,
                                      ConvexRadius(fMinDimension));
      JPH::Shape::ShapeResult cResult = cSettings.Create();
      if(cResult.HasError()) {
         THROW_ARGOSEXCEPTION("Error creating a Jolt box shape: "
                              << cResult.GetError().c_str());
      }
      m_vecBoxes.push_back({c_half_extents, cResult.Get()});
      return m_vecBoxes.back().Shape;
   }

   /****************************************/
   /****************************************/

   JPH::RefConst<JPH::Shape> CJoltShapeManager::RequestCylinder(float f_half_height,
                                                                float f_radius) {
      for(const SCylinder& s_cylinder : m_vecCylinders) {
         if(s_cylinder.HalfHeight == f_half_height &&
            s_cylinder.Radius == f_radius) {
            return s_cylinder.Shape;
         }
      }
      /* Jolt cylinders have the axis along y; rotate it onto z */
      JPH::CylinderShapeSettings cCylinderSettings(
         f_half_height, f_radius,
         ConvexRadius(std::min(f_half_height, f_radius)));
      JPH::Shape::ShapeResult cCylinderResult = cCylinderSettings.Create();
      if(cCylinderResult.HasError()) {
         THROW_ARGOSEXCEPTION("Error creating a Jolt cylinder shape: "
                              << cCylinderResult.GetError().c_str());
      }
      JPH::RotatedTranslatedShapeSettings cSettings(
         JPH::Vec3::sZero(),
         JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI),
         cCylinderResult.Get());
      JPH::Shape::ShapeResult cResult = cSettings.Create();
      if(cResult.HasError()) {
         THROW_ARGOSEXCEPTION("Error creating a Jolt cylinder shape: "
                              << cResult.GetError().c_str());
      }
      m_vecCylinders.push_back({f_half_height, f_radius, cResult.Get()});
      return m_vecCylinders.back().Shape;
   }

   /****************************************/
   /****************************************/

}
