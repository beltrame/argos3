/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/mesh_entity.h>
 *
 * A static triangle-mesh piece of world geometry, loaded from a glTF
 * file. It lives in the Jolt plugin because CJoltMeshModel is the only
 * physics model that can represent it: neither dynamics2d nor
 * dynamics3d exposes a triangle-mesh shape.
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#ifndef MESH_ENTITY_H
#define MESH_ENTITY_H

namespace argos {
   class CMeshEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <string>

namespace argos {

   class CMeshEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      CMeshEntity();

      virtual ~CMeshEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline const CEmbodiedEntity& GetEmbodiedEntity() const {
         return *m_pcEmbodiedEntity;
      }

      /** Path to the glTF file, as written in the XML */
      inline const std::string& GetFile() const {
         return m_strFile;
      }

      /** Whether the asset is Y-up and needs conversion to ARGoS Z-up */
      inline bool IsYUp() const {
         return m_bYUp;
      }

      /** Whether every triangle is also emitted with flipped winding,
       *  so that collision does not depend on the winding of the asset */
      inline bool IsDoubleSided() const {
         return m_bDoubleSided;
      }

      inline Real GetScale() const {
         return m_fScale;
      }

      virtual std::string GetTypeDescription() const {
         return "mesh";
      }

   private:

      CEmbodiedEntity* m_pcEmbodiedEntity;
      std::string m_strFile;
      bool m_bYUp;
      bool m_bDoubleSided;
      Real m_fScale;

   };

}

#endif
