/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_id_scene.h>
 *
 * A parallel Filament scene used for the segmentation/depth camera
 * pass. Every renderable of the main scene gets a paired renderable
 * here that shares its vertex/index buffers and is transform-parented
 * to it, so poses stay in sync for free. All paired renderables use
 * one unlit material that packs (entity id, class id, linear view
 * depth) into an RGBA32F color target.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_ID_SCENE_H
#define PR_ID_SCENE_H

namespace argos {
   class CPRRenderEngine;
   struct SPRMesh;
}

namespace filament {
   class Scene;
   class Material;
   class MaterialInstance;
}

#include <argos3/core/utility/datatypes/datatypes.h>

#include <utils/Entity.h>

#include <map>

namespace argos {

   /**
    * Class ids of the built-in visuals. Robot models (milestone M3)
    * extend these through the asset registry.
    */
   enum class EPRClass : UInt8 {
      None = 0,
      Floor = 1,
      Box = 2,
      Cylinder = 3,
      FootBot = 4,
      Drone = 5,
      /* Static glTF scenery props. Like every other renderable, scenery
       * must carry a NONZERO entity id: the aux buffer clears to zero,
       * so id 0 is the "no geometry here" sentinel the camera sensor
       * uses to fill background pixels with the far plane. Scenery was
       * originally registered with id 0 and was therefore invisible to
       * the depth and segmentation modalities. */
      Scenery = 6,
      BunkerMini = 7
   };


   class CPRIdScene {

   public:

      void Init(CPRRenderEngine& c_engine);
      void Destroy();

      inline filament::Scene& GetScene() {
         return *m_pcScene;
      }

      /** The material that packs (entityId, classId, viewDepth) */
      inline filament::Material& GetAuxMaterial() {
         return *m_pcAuxMaterial;
      }

      /**
       * Creates the paired renderable for a main-scene renderable.
       * @param s_mesh The mesh shared with the main renderable
       * @param c_main_renderable The main renderable to follow
       * @param un_entity_id The per-run numeric id of the entity
       * @param un_class_id The class id of the entity
       */
      void AddInstance(const SPRMesh& s_mesh,
                       utils::Entity c_main_renderable,
                       UInt16 un_entity_id,
                       UInt8 un_class_id);

      /**
       * Destroys the paired renderable of a main-scene renderable.
       */
      void RemoveInstance(utils::Entity c_main_renderable);

   private:

      struct SPair {
         utils::Entity Renderable;
         filament::MaterialInstance* Material = nullptr;
      };

      CPRRenderEngine* m_pcEngine = nullptr;
      filament::Scene* m_pcScene = nullptr;
      filament::Material* m_pcAuxMaterial = nullptr;
      std::map<utils::Entity, SPair> m_mapPairs;

   };

}

#endif
