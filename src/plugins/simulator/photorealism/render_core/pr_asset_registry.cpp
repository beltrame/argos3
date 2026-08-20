/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_asset_registry.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_asset_registry.h"
#include "pr_render_engine.h"
#include "pr_id_scene.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/FilamentInstance.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/TextureProvider.h>
#include <gltfio/materials/uberarchive.h>
#include <math/quat.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace argos {

   using filament::math::float2;
   using filament::math::float3;
   using filament::math::mat4f;
   using filament::math::quatf;
   namespace gltfio = filament::gltfio;

   /****************************************/
   /****************************************/

   static void SplitPathList(const std::string& str_paths,
                             std::vector<std::string>& vec_paths) {
      std::string::size_type unStart = 0;
      while(unStart <= str_paths.size()) {
         std::string::size_type unEnd = str_paths.find(':', unStart);
         if(unEnd == std::string::npos) {
            unEnd = str_paths.size();
         }
         if(unEnd > unStart) {
            vec_paths.push_back(str_paths.substr(unStart, unEnd - unStart));
         }
         unStart = unEnd + 1;
      }
   }

   /****************************************/
   /****************************************/

   void CPRAssetRegistry::Init(CPRRenderEngine& c_engine,
                               CPRIdScene& c_id_scene,
                               const std::string& str_search_path) {
      m_pcEngine = &c_engine;
      m_pcIdScene = &c_id_scene;
      /* Search order: XML attribute, environment, installed assets */
      SplitPathList(str_search_path, m_vecSearchPaths);
      const char* pchEnvironment = std::getenv("ARGOS_PHOTOREALISM_ASSET_PATH");
      if(pchEnvironment != nullptr) {
         SplitPathList(pchEnvironment, m_vecSearchPaths);
      }
#ifdef ARGOS_PR_ASSET_DIR
      m_vecSearchPaths.push_back(ARGOS_PR_ASSET_DIR);
#endif
      filament::Engine& cEngine = c_engine.GetEngine();
      m_pcMaterials = gltfio::createUbershaderProvider(
         &cEngine, UBERARCHIVE_DEFAULT_DATA, UBERARCHIVE_DEFAULT_SIZE);
      gltfio::AssetConfiguration sAssetConfiguration;
      sAssetConfiguration.engine = &cEngine;
      sAssetConfiguration.materials = m_pcMaterials;
      m_pcLoader = gltfio::AssetLoader::create(sAssetConfiguration);
      gltfio::ResourceConfiguration sResourceConfiguration;
      sResourceConfiguration.engine = &cEngine;
      /* Left null here and set per model in LoadModelFile: one registry
       * serves assets from many directories, so there is no single base
       * path that would be correct at this point. */
      sResourceConfiguration.gltfPath = nullptr;
      sResourceConfiguration.normalizeSkinningWeights = true;
      m_pcResourceLoader = new gltfio::ResourceLoader(sResourceConfiguration);
      m_pcStbProvider = gltfio::createStbProvider(&cEngine);
      m_pcKtx2Provider = gltfio::createKtx2Provider(&cEngine);
      m_pcResourceLoader->addTextureProvider("image/png", m_pcStbProvider);
      m_pcResourceLoader->addTextureProvider("image/jpeg", m_pcStbProvider);
      m_pcResourceLoader->addTextureProvider("image/ktx2", m_pcKtx2Provider);
   }

   /****************************************/
   /****************************************/

   void CPRAssetRegistry::Destroy() {
      if(m_pcEngine == nullptr || !m_pcEngine->IsCreated()) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      cEngine.flushAndWait();
      for(auto& tAsset : m_mapAssets) {
         /* The asset's renderables reference the aux material
          * instances: destroy the asset first */
         if(tAsset.second.Asset != nullptr) {
            m_pcLoader->destroyAsset(tAsset.second.Asset);
         }
         for(SInstance& s_instance : tAsset.second.Recycled) {
            if(s_instance.AuxMaterial != nullptr) {
               cEngine.destroy(s_instance.AuxMaterial);
            }
         }
      }
      m_mapAssets.clear();
      delete m_pcResourceLoader;
      m_pcResourceLoader = nullptr;
      delete m_pcStbProvider;
      m_pcStbProvider = nullptr;
      delete m_pcKtx2Provider;
      m_pcKtx2Provider = nullptr;
      gltfio::AssetLoader::destroy(&m_pcLoader);
      m_pcMaterials->destroyMaterials();
      delete m_pcMaterials;
      m_pcMaterials = nullptr;
      m_pcEngine = nullptr;
   }

   /****************************************/
   /****************************************/

   bool CPRAssetRegistry::ParseDescriptor(const std::string& str_file,
                                          SPRVisualDescriptor& s_descriptor) {
      try {
         ticpp::Document cDocument(str_file);
         cDocument.LoadFile();
         TConfigurationNode* ptRoot = cDocument.FirstChildElement("visual");
         TConfigurationNode& tModel = GetNode(*ptRoot, "model");
         std::string strModel;
         GetNodeAttribute(tModel, "path", strModel);
         s_descriptor.ModelPath =
            (std::filesystem::path(str_file).parent_path() / strModel).string();
         Real fScale = 1.0;
         GetNodeAttributeOrDefault(tModel, "scale", fScale, fScale);
         CVector3 cPosition;
         GetNodeAttributeOrDefault(tModel, "position", cPosition, cPosition);
         CVector3 cOrientationAngles;
         GetNodeAttributeOrDefault(tModel, "orientation",
                                   cOrientationAngles, cOrientationAngles);
         CQuaternion cOrientation;
         cOrientation.FromEulerAngles(
            ToRadians(CDegrees(cOrientationAngles.GetX())),
            ToRadians(CDegrees(cOrientationAngles.GetY())),
            ToRadians(CDegrees(cOrientationAngles.GetZ())));
         s_descriptor.Offset =
            mat4f::translation(float3{float(cPosition.GetX()),
                                      float(cPosition.GetY()),
                                      float(cPosition.GetZ())}) *
            mat4f(quatf(float(cOrientation.GetW()),
                        float(cOrientation.GetX()),
                        float(cOrientation.GetY()),
                        float(cOrientation.GetZ()))) *
            mat4f::scaling(float3{float(fScale)});
         UInt32 unClassId = 0;
         if(NodeExists(*ptRoot, "segmentation")) {
            GetNodeAttributeOrDefault(GetNode(*ptRoot, "segmentation"),
                                      "class", unClassId, unClassId);
         }
         s_descriptor.ClassId = UInt8(unClassId);
         return true;
      }
      catch(std::exception& ex) {
         LOGERR << "[WARNING] Ignoring visual descriptor \"" << str_file
                << "\": " << ex.what() << std::endl;
         return false;
      }
   }

   /****************************************/
   /****************************************/

   CPRAssetRegistry::SAsset&
   CPRAssetRegistry::LoadAsset(const std::string& str_type) {
      auto itAsset = m_mapAssets.find(str_type);
      if(itAsset != m_mapAssets.end()) {
         return itAsset->second;
      }
      SAsset& sAsset = m_mapAssets[str_type];
      /* Find the descriptor */
      std::string strDescriptorFile;
      for(const std::string& str_path : m_vecSearchPaths) {
         std::string strCandidate = str_path + "/" + str_type + ".visual.xml";
         if(std::filesystem::exists(strCandidate)) {
            strDescriptorFile = strCandidate;
            break;
         }
      }
      if(strDescriptorFile.empty() ||
         !ParseDescriptor(strDescriptorFile, sAsset.Descriptor)) {
         return sAsset;
      }
      LoadModelFile(sAsset, "entity type \"" + str_type + "\"");
      return sAsset;
   }

   /****************************************/
   /****************************************/

   void CPRAssetRegistry::LoadModelFile(SAsset& s_asset,
                                        const std::string& str_label) {
      std::ifstream cFile(s_asset.Descriptor.ModelPath,
                          std::ios::binary | std::ios::ate);
      if(!cFile) {
         LOGERR << "[WARNING] Cannot open model \""
                << s_asset.Descriptor.ModelPath << "\" for "
                << str_label << std::endl;
         return;
      }
      std::vector<UInt8> vecBytes(size_t(cFile.tellg()));
      cFile.seekg(0);
      cFile.read(reinterpret_cast<char*>(vecBytes.data()), vecBytes.size());
      gltfio::FilamentInstance* pcFirstInstance = nullptr;
      s_asset.Asset = m_pcLoader->createInstancedAsset(
         vecBytes.data(), uint32_t(vecBytes.size()), &pcFirstInstance, 1);
      if(s_asset.Asset == nullptr) {
         LOGERR << "[WARNING] Cannot parse model \""
                << s_asset.Descriptor.ModelPath << "\" for "
                << str_label << std::endl;
         return;
      }
      /* A glTF names its textures and .bin buffers by URIs relative to itself.
       * Filament resolves them against gltfPath, and falls back to the process
       * working directory when it is null, so a model loaded from anywhere but
       * that directory loses every texture and renders untextured. The .argos
       * file may name assets by any path, and ARGoS may be started from any
       * directory, so the two coincide only by accident.
       *
       * Set per model rather than once at Init because the registry serves
       * assets from many directories. Filament documents that it does not
       * retain the string, so pointing at ModelPath's buffer is safe. */
      gltfio::ResourceConfiguration sResourceConfiguration;
      sResourceConfiguration.engine = &m_pcEngine->GetEngine();
      sResourceConfiguration.gltfPath = s_asset.Descriptor.ModelPath.c_str();
      sResourceConfiguration.normalizeSkinningWeights = true;
      m_pcResourceLoader->setConfiguration(sResourceConfiguration);
      if(!m_pcResourceLoader->loadResources(s_asset.Asset)) {
         LOGERR << "[WARNING] Cannot load the resources of model \""
                << s_asset.Descriptor.ModelPath << "\" for "
                << str_label << std::endl;
         m_pcLoader->destroyAsset(s_asset.Asset);
         s_asset.Asset = nullptr;
         return;
      }
      /* The first instance is kept for the first entity */
      SInstance sSpare;
      sSpare.Main = pcFirstInstance;
      s_asset.Recycled.push_back(sSpare);
      LOG << "[INFO] Loaded visual \"" << s_asset.Descriptor.ModelPath
          << "\" for " << str_label << std::endl;
   }

   /****************************************/
   /****************************************/

   const SPRVisualDescriptor*
   CPRAssetRegistry::GetDescriptor(const std::string& str_type) {
      SAsset& sAsset = LoadAsset(str_type);
      return sAsset.Asset != nullptr ? &sAsset.Descriptor : nullptr;
   }

   /****************************************/
   /****************************************/

   CPRAssetRegistry::SInstance
   CPRAssetRegistry::CreateInstance(const std::string& str_type,
                                    UInt16 un_entity_id,
                                    UInt8 un_class_id) {
      return InstantiateAsset(LoadAsset(str_type),
                              un_entity_id, un_class_id, str_type);
   }

   /****************************************/
   /****************************************/

   CPRAssetRegistry::SInstance
   CPRAssetRegistry::CreateModelInstance(const std::string& str_model_path,
                                         UInt16 un_entity_id,
                                         UInt8 un_class_id) {
      /* Path-based assets are cached under a key that cannot clash
       * with entity type descriptions */
      SAsset& sAsset = m_mapAssets["model:" + str_model_path];
      if(sAsset.Asset == nullptr && sAsset.Descriptor.ModelPath.empty()) {
         sAsset.Descriptor.ModelPath = str_model_path;
         LoadModelFile(sAsset, "the scenery");
      }
      return InstantiateAsset(sAsset, un_entity_id, un_class_id,
                              str_model_path);
   }

   /****************************************/
   /****************************************/

   void CPRAssetRegistry::ReleaseModelInstance(const std::string& str_model_path,
                                               SInstance& s_instance) {
      if(s_instance.Main == nullptr) {
         return;
      }
      DetachFromScenes(s_instance);
      m_mapAssets["model:" + str_model_path].Recycled.push_back(s_instance);
      s_instance = SInstance();
   }

   /****************************************/
   /****************************************/

   CPRAssetRegistry::SInstance
   CPRAssetRegistry::InstantiateAsset(SAsset& sAsset,
                                      UInt16 un_entity_id,
                                      UInt8 un_class_id,
                                      const std::string& str_label) {
      if(sAsset.Asset == nullptr) {
         return SInstance();
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      filament::RenderableManager& cRenderables = cEngine.getRenderableManager();
      filament::TransformManager& cTransforms = cEngine.getTransformManager();
      SInstance sInstance;
      if(!sAsset.Recycled.empty()) {
         sInstance = sAsset.Recycled.back();
         sAsset.Recycled.pop_back();
      }
      if(sInstance.Main == nullptr) {
         sInstance.Main = m_pcLoader->createInstance(sAsset.Asset);
      }
      if(sInstance.Main == nullptr) {
         LOGERR << "[WARNING] Cannot instantiate visual \""
                << str_label << "\"" << std::endl;
         return SInstance();
      }
      if(sInstance.Aux == nullptr) {
         sInstance.Aux = m_pcLoader->createInstance(sAsset.Asset);
         if(sInstance.Aux == nullptr) {
            LOGERR << "[WARNING] Cannot instantiate the segmentation visual "
                      "\"" << str_label << "\"" << std::endl;
            sAsset.Recycled.push_back(sInstance);
            return SInstance();
         }
         sInstance.AuxMaterial = m_pcIdScene->GetAuxMaterial().createInstance();
         /* Swap all the aux instance materials for the id material */
         const utils::Entity* pcEntities = sInstance.Aux->getEntities();
         for(size_t i = 0; i < sInstance.Aux->getEntityCount(); ++i) {
            auto cRenderable = cRenderables.getInstance(pcEntities[i]);
            if(cRenderable) {
               for(size_t j = 0;
                   j < cRenderables.getPrimitiveCount(cRenderable); ++j) {
                  cRenderables.setMaterialInstanceAt(cRenderable, j,
                                                     sInstance.AuxMaterial);
               }
               cRenderables.setCastShadows(cRenderable, false);
               cRenderables.setReceiveShadows(cRenderable, false);
            }
         }
         /* The aux instance follows the main one */
         cTransforms.setParent(
            cTransforms.getInstance(sInstance.Aux->getRoot()),
            cTransforms.getInstance(sInstance.Main->getRoot()));
         cTransforms.setTransform(
            cTransforms.getInstance(sInstance.Aux->getRoot()), mat4f());
      }
      sInstance.AuxMaterial->setParameter(
         "idClass", float2{float(un_entity_id), float(un_class_id)});
      AttachToScenes(sInstance);
      return sInstance;
   }

   /****************************************/
   /****************************************/

   void CPRAssetRegistry::ReleaseInstance(const std::string& str_type,
                                          SInstance& s_instance) {
      if(s_instance.Main == nullptr) {
         return;
      }
      DetachFromScenes(s_instance);
      m_mapAssets[str_type].Recycled.push_back(s_instance);
      s_instance = SInstance();
   }

   /****************************************/
   /****************************************/

   void CPRAssetRegistry::AttachToScenes(SInstance& s_instance) {
      m_pcEngine->GetScene().addEntities(s_instance.Main->getEntities(),
                                         s_instance.Main->getEntityCount());
      m_pcIdScene->GetScene().addEntities(s_instance.Aux->getEntities(),
                                          s_instance.Aux->getEntityCount());
   }

   /****************************************/
   /****************************************/

   void CPRAssetRegistry::DetachFromScenes(SInstance& s_instance) {
      m_pcEngine->GetScene().removeEntities(s_instance.Main->getEntities(),
                                            s_instance.Main->getEntityCount());
      m_pcIdScene->GetScene().removeEntities(s_instance.Aux->getEntities(),
                                             s_instance.Aux->getEntityCount());
   }

   /****************************************/
   /****************************************/

}
