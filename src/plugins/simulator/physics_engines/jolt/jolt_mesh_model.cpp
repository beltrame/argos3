/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_mesh_model.cpp>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#include "jolt_mesh_model.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/physics_engines/jolt/mesh_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_mesh_asset.h>

namespace argos {

   /****************************************/
   /****************************************/

   CJoltMeshModel::CJoltMeshModel(CJoltEngine& c_engine,
                                  CMeshEntity& c_mesh) :
      CJoltSingleBodyObjectModel(c_engine, c_mesh) {
      SJoltMeshAsset sAsset = CJoltMeshAsset::Request(c_mesh.GetFile(),
                                                      c_mesh.IsYUp(),
                                                      c_mesh.IsDoubleSided(),
                                                      c_mesh.GetScale());
      if(sAsset.Cooked) {
         LOG << "[INFO] Cooked mesh \"" << sAsset.Path << "\": "
             << sAsset.Vertices << " vertices, "
             << sAsset.Triangles << " triangles"
             << (c_mesh.IsDoubleSided() ? " (double-sided)" : "") << ", "
             << sAsset.CookSeconds << " s" << std::endl;
      }
      /* The mesh origin is the file origin; no anchor offset. Jolt
       * mesh shapes must be static (MeshShape::MustBeStatic) and the
       * entity is declared non-movable, so both agree. */
      SAnchor& sAnchor = c_mesh.GetEmbodiedEntity().GetOriginAnchor();
      JPH::BodyCreationSettings cSettings(sAsset.Shape,
                                          ToJolt(sAnchor.Position),
                                          ToJolt(sAnchor.Orientation),
                                          JPH::EMotionType::Static,
                                          JoltLayers::NON_MOVING);
      cSettings.mFriction = c_engine.GetDefaultFriction();
      /* CreateBody sets the body's user data to this model, which is
       * what makes ray hits visible to ARGoS: CJoltEngine::
       * CheckIntersectionWithRay discards hits on bodies with no user
       * data (the reason the <floor> plugin is invisible to rays). */
      CreateBody(cSettings, &sAnchor, JPH::Vec3::sZero(), JPH::Quat::sIdentity());
      /* Finalize the model: fills the ARGoS bounding box from the
       * mesh AABB, which the space uses for indexing */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY(CMeshEntity, CJoltMeshModel);

   /****************************************/
   /****************************************/

}
