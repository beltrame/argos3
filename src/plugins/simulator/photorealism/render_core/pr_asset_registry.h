/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_asset_registry.h>
 *
 * Data-driven robot visuals: maps an entity type (its ARGoS type
 * description, e.g. "foot-bot") to a glTF model through a
 * `<type>.visual.xml` descriptor:
 *
 *   <visual>
 *     <model path="footbot.glb" scale="1"
 *            position="0,0,0" orientation="0,0,90" />
 *     <segmentation class="4" />
 *   </visual>
 *
 * The model path is relative to the descriptor; 'orientation' (Euler
 * z,y,x in degrees) maps the model frame onto the ARGoS frame (glTF
 * models are y-up; "0,0,90" makes them z-up). Descriptors are looked
 * up in the medium's asset_path directories, then in the directories
 * of the ARGOS_PHOTOREALISM_ASSET_PATH environment variable, then in
 * the installed assets.
 *
 * Every entity gets two gltfio instances sharing the model's GPU
 * buffers: one in the main scene (glTF PBR materials) and one in the
 * segmentation scene, with its materials swapped to the aux material
 * that packs (entityId, classId, depth). Since gltfio instances
 * cannot be destroyed individually, released instances are detached
 * from the scenes and recycled.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_ASSET_REGISTRY_H
#define PR_ASSET_REGISTRY_H

namespace argos {
   class CPRRenderEngine;
   class CPRIdScene;
}

namespace filament {
   class MaterialInstance;
}

namespace filament::gltfio {
   class AssetLoader;
   class FilamentAsset;
   class FilamentInstance;
   class MaterialProvider;
   class ResourceLoader;
   class TextureProvider;
}

#include <argos3/core/utility/datatypes/datatypes.h>

#include <math/mat4.h>

#include <map>
#include <string>
#include <vector>

namespace argos {

   struct SPRVisualDescriptor {
      /** Absolute path of the glTF model */
      std::string ModelPath;
      /** Segmentation class id */
      UInt8 ClassId = 0;
      /** Model-to-ARGoS frame transform (scale included) */
      filament::math::mat4f Offset;
   };

   class CPRAssetRegistry {

   public:

      struct SInstance {
         filament::gltfio::FilamentInstance* Main = nullptr;
         filament::gltfio::FilamentInstance* Aux = nullptr;
         filament::MaterialInstance* AuxMaterial = nullptr;
      };

   public:

      /**
       * @param str_search_path colon-separated list of directories
       *        searched before the environment and the install dir
       */
      void Init(CPRRenderEngine& c_engine,
                CPRIdScene& c_id_scene,
                const std::string& str_search_path);

      void Destroy();

      /**
       * Returns the visual descriptor for the given entity type, or
       * nullptr when none of the search directories has one. Lookups
       * are cached either way.
       */
      const SPRVisualDescriptor* GetDescriptor(const std::string& str_type);

      /**
       * Creates (or recycles) the pair of gltf instances for one
       * entity of the given type and adds them to the scenes. Returns
       * an empty instance (Main == nullptr) on failure.
       */
      SInstance CreateInstance(const std::string& str_type,
                               UInt16 un_entity_id,
                               UInt8 un_class_id);

      /**
       * Detaches the instances from the scenes and recycles them.
       */
      void ReleaseInstance(const std::string& str_type,
                           SInstance& s_instance);

      /**
       * Like CreateInstance()/ReleaseInstance(), but for a glTF model
       * given directly by path (no descriptor); used for scenery.
       */
      SInstance CreateModelInstance(const std::string& str_model_path,
                                    UInt16 un_entity_id,
                                    UInt8 un_class_id);

      void ReleaseModelInstance(const std::string& str_model_path,
                                SInstance& s_instance);

   private:

      struct SAsset {
         SPRVisualDescriptor Descriptor;
         /** nullptr when the type has no visual (negative cache) or
          *  the model failed to load */
         filament::gltfio::FilamentAsset* Asset = nullptr;
         std::vector<SInstance> Recycled;
      };

      SAsset& LoadAsset(const std::string& str_type);
      void LoadModelFile(SAsset& s_asset, const std::string& str_label);
      SInstance InstantiateAsset(SAsset& s_asset,
                                 UInt16 un_entity_id,
                                 UInt8 un_class_id,
                                 const std::string& str_label);
      bool ParseDescriptor(const std::string& str_file,
                           SPRVisualDescriptor& s_descriptor);
      void AttachToScenes(SInstance& s_instance);
      void DetachFromScenes(SInstance& s_instance);

   private:

      CPRRenderEngine* m_pcEngine = nullptr;
      CPRIdScene* m_pcIdScene = nullptr;
      std::vector<std::string> m_vecSearchPaths;
      filament::gltfio::MaterialProvider* m_pcMaterials = nullptr;
      filament::gltfio::TextureProvider* m_pcStbProvider = nullptr;
      filament::gltfio::TextureProvider* m_pcKtx2Provider = nullptr;
      filament::gltfio::AssetLoader* m_pcLoader = nullptr;
      filament::gltfio::ResourceLoader* m_pcResourceLoader = nullptr;
      std::map<std::string, SAsset> m_mapAssets;

   };

}

#endif
