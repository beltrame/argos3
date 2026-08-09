/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_mesh_model.h>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#ifndef JOLT_MESH_MODEL_H
#define JOLT_MESH_MODEL_H

namespace argos {
   class CMeshEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   class CJoltMeshModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltMeshModel(CJoltEngine& c_engine,
                     CMeshEntity& c_mesh);

      virtual ~CJoltMeshModel() {}

      /**
       * World geometry is never "in collision", in the same sense in
       * which the floor plugin is not. Overriding also avoids a
       * mesh-versus-mesh narrow-phase query, which Jolt does not
       * support.
       */
      virtual bool IsCollidingWithSomething() const {
         return false;
      }

   };

}

#endif
