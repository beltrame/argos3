/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_id_scene.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_id_scene.h"
#include "pr_render_engine.h"
#include "pr_mesh_builder.h"

#include <argos3/core/utility/configuration/argos_exception.h>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>
#include <utils/EntityManager.h>
#include <math/vec2.h>

namespace argos {

   /* Generated from materials/pr_aux.mat at build time */
   extern const unsigned char PR_AUX_FILAMAT[];
   extern const size_t PR_AUX_FILAMAT_SIZE;

   /****************************************/
   /****************************************/

   void CPRIdScene::Init(CPRRenderEngine& c_engine) {
      m_pcEngine = &c_engine;
      filament::Engine& cEngine = c_engine.GetEngine();
      m_pcScene = cEngine.createScene();
      m_pcAuxMaterial = filament::Material::Builder()
         .package(PR_AUX_FILAMAT, PR_AUX_FILAMAT_SIZE)
         .build(cEngine);
      if(m_pcAuxMaterial == nullptr) {
         THROW_ARGOSEXCEPTION("Failed to load the embedded pr_aux material");
      }
   }

   /****************************************/
   /****************************************/

   void CPRIdScene::Destroy() {
      if(m_pcEngine == nullptr || !m_pcEngine->IsCreated()) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      for(auto& tPair : m_mapPairs) {
         m_pcScene->remove(tPair.second.Renderable);
         cEngine.destroy(tPair.second.Renderable);
         utils::EntityManager::get().destroy(tPair.second.Renderable);
         cEngine.destroy(tPair.second.Material);
      }
      m_mapPairs.clear();
      if(m_pcAuxMaterial != nullptr) {
         cEngine.destroy(m_pcAuxMaterial);
         m_pcAuxMaterial = nullptr;
      }
      if(m_pcScene != nullptr) {
         cEngine.destroy(m_pcScene);
         m_pcScene = nullptr;
      }
      m_pcEngine = nullptr;
   }

   /****************************************/
   /****************************************/

   void CPRIdScene::AddInstance(const SPRMesh& s_mesh,
                                utils::Entity c_main_renderable,
                                UInt16 un_entity_id,
                                UInt8 un_class_id) {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      SPair sPair;
      sPair.Material = m_pcAuxMaterial->createInstance();
      sPair.Material->setParameter(
         "idClass",
         filament::math::float2{float(un_entity_id), float(un_class_id)});
      sPair.Renderable = utils::EntityManager::get().create();
      filament::RenderableManager::Builder(1)
         .boundingBox(s_mesh.Aabb)
         .material(0, sPair.Material)
         .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES,
                   s_mesh.Vertices, s_mesh.Indices)
         .receiveShadows(false)
         .castShadows(false)
         .build(cEngine, sPair.Renderable);
      /* Follow the main renderable: parent with identity local transform */
      filament::TransformManager& cTransforms = cEngine.getTransformManager();
      cTransforms.create(sPair.Renderable,
                         cTransforms.getInstance(c_main_renderable));
      m_pcScene->addEntity(sPair.Renderable);
      m_mapPairs[c_main_renderable] = sPair;
   }

   /****************************************/
   /****************************************/

   void CPRIdScene::RemoveInstance(utils::Entity c_main_renderable) {
      auto itPair = m_mapPairs.find(c_main_renderable);
      if(itPair == m_mapPairs.end()) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      m_pcScene->remove(itPair->second.Renderable);
      cEngine.destroy(itPair->second.Renderable);
      utils::EntityManager::get().destroy(itPair->second.Renderable);
      cEngine.destroy(itPair->second.Material);
      m_mapPairs.erase(itPair);
   }

   /****************************************/
   /****************************************/

}
