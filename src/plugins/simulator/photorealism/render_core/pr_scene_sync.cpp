/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_scene_sync.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_scene_sync.h"
#include "pr_render_engine.h"
#include "pr_id_scene.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/entities/directional_led_equipped_entity.h>

#include <gltfio/FilamentInstance.h>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>
#include <filament/LightManager.h>
#include <filament/IndirectLight.h>
#include <filament/Skybox.h>
#include <image/Ktx1Bundle.h>
#include <ktxreader/Ktx1Reader.h>

#include <algorithm>
#include <fstream>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>
#include <utils/EntityManager.h>
#include <math/mat4.h>
#include <math/quat.h>

namespace argos {

   using filament::math::float3;
   using filament::math::mat4f;
   using filament::math::quatf;

   /* Generated from materials/pr_lit_tex.mat at build time */
   extern const unsigned char PR_LIT_TEX_FILAMAT[];
   extern const size_t PR_LIT_TEX_FILAMAT_SIZE;

   /* Emissive luminance of a fully lit LED, in nits; must compete
    * with the sunny-day exposure of the cameras */
   static const float LED_EMISSIVE_NITS = 25000.0f;

   /* Edge of the emissive cube that visualizes an LED. Real LEDs
    * light up a lens/diffusor area much larger than the die, and the
    * cube must span at least a pixel in a low-resolution robot camera
    * at typical interaction distances */
   static const float LED_CUBE_SIZE = 0.016f;

   /* Foot-bot dimensions from qtopengl_footbot.cpp */
   static const float FOOTBOT_BODY_RADIUS = 0.085036758f;
   static const float FOOTBOT_TURRET_RADIUS = 0.069f;
   static const float FOOTBOT_WHEEL_RADIUS = 0.029112741f;
   static const float FOOTBOT_WHEEL_WIDTH = 0.022031354f;
   static const float FOOTBOT_HALF_INTERWHEEL = 0.0635f;
   static const float FOOTBOT_HEIGHT = 0.258f;

   /****************************************/
   /****************************************/

   static mat4f TRS(float f_tx, float f_ty, float f_tz,
                    const quatf& c_rotation,
                    float f_sx, float f_sy, float f_sz) {
      return mat4f::translation(float3{f_tx, f_ty, f_tz}) *
             mat4f(c_rotation) *
             mat4f::scaling(float3{f_sx, f_sy, f_sz});
   }

   static mat4f TS(float f_tx, float f_ty, float f_tz,
                   float f_sx, float f_sy, float f_sz) {
      return mat4f::translation(float3{f_tx, f_ty, f_tz}) *
             mat4f::scaling(float3{f_sx, f_sy, f_sz});
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::Init(CPRRenderEngine& c_engine,
                           CPRIdScene& c_id_scene,
                           CPRAssetRegistry& c_assets,
                           const SSunlight& s_sunlight,
                           const CVector3& c_arena_size,
                           const CVector3& c_arena_center) {
      m_pcEngine = &c_engine;
      m_pcIdScene = &c_id_scene;
      m_pcAssets = &c_assets;
      filament::Engine& cEngine = c_engine.GetEngine();
      m_sBoxMesh = CPRMeshBuilder::BuildBox(cEngine);
      m_sCylinderMesh = CPRMeshBuilder::BuildCylinder(cEngine);
      m_sPlaneMesh = CPRMeshBuilder::BuildPlane(cEngine);
      m_pcTexturedMaterial = filament::Material::Builder()
         .package(PR_LIT_TEX_FILAMAT, PR_LIT_TEX_FILAMAT_SIZE)
         .build(cEngine);
      if(m_pcTexturedMaterial == nullptr) {
         THROW_ARGOSEXCEPTION("Failed to load the embedded pr_lit_tex material");
      }
      /* Sunlight */
      m_cSunlight = utils::EntityManager::get().create();
      CVector3 cDirection(s_sunlight.Direction);
      cDirection.Normalize();
      filament::LightManager::Builder(filament::LightManager::Type::SUN)
         .color(filament::Color::toLinear<filament::ACCURATE>({0.98f, 0.92f, 0.89f}))
         .intensity(float(s_sunlight.Intensity))
         .direction({float(cDirection.GetX()),
                     float(cDirection.GetY()),
                     float(cDirection.GetZ())})
         .castShadows(s_sunlight.CastShadows)
         .build(cEngine, m_cSunlight);
      c_engine.GetScene().addEntity(m_cSunlight);
      /* Constant ambient irradiance (single spherical-harmonics band):
       * HDR environment maps arrive with the asset pipeline; without
       * indirect light any surface not facing the sun renders pitch
       * black */
      const float3 pcIrradiance[1] = {
         {0.85f, 0.88f, 1.0f}
      };
      m_pcAmbientLight = filament::IndirectLight::Builder()
         .irradiance(1, pcIrradiance)
         .intensity(float(s_sunlight.Intensity) * 0.2f)
         .build(cEngine);
      c_engine.GetScene().setIndirectLight(m_pcAmbientLight);
      m_cArenaSize = c_arena_size;
      m_cArenaCenter = c_arena_center;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::InitFloor(CSpace& c_space,
                                const CVector3& c_arena_size,
                                const CVector3& c_arena_center) {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      /* Use the floor entity's colors when the arena has one */
      try {
         m_pcFloorEntity = &c_space.GetFloorEntity();
      }
      catch(CARGoSException&) {
         m_pcFloorEntity = nullptr;
      }
      if(m_pcFloorEntity != nullptr) {
         m_pcFloorTexture = filament::Texture::Builder()
            .width(m_unFloorTextureSize)
            .height(m_unFloorTextureSize)
            .levels(1)
            .usage(filament::Texture::Usage::SAMPLEABLE |
                   filament::Texture::Usage::UPLOADABLE)
            .format(filament::Texture::InternalFormat::SRGB8_A8)
            .build(cEngine);
         m_pcFloorMaterial = m_pcTexturedMaterial->createInstance();
         filament::TextureSampler cSampler(
            filament::TextureSampler::MinFilter::LINEAR,
            filament::TextureSampler::MagFilter::LINEAR);
         m_pcFloorMaterial->setParameter("baseColorMap",
                                         m_pcFloorTexture, cSampler);
         m_pcFloorMaterial->setParameter("roughness", 0.9f);
         m_pcFloorMaterial->setParameter("metallic", 0.0f);
         SyncFloorTexture();
      }
      else {
         m_pcFloorMaterial = m_pcEngine->GetLitMaterial().createInstance();
         m_pcFloorMaterial->setParameter("baseColor", float3{0.75f, 0.75f, 0.75f});
         m_pcFloorMaterial->setParameter("roughness", 0.9f);
         m_pcFloorMaterial->setParameter("metallic", 0.0f);
         m_pcFloorMaterial->setParameter("emissive", float3{0.0f});
      }
      m_cFloor = utils::EntityManager::get().create();
      filament::RenderableManager::Builder(1)
         .boundingBox(m_sPlaneMesh.Aabb)
         .material(0, m_pcFloorMaterial)
         .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES,
                   m_sPlaneMesh.Vertices, m_sPlaneMesh.Indices)
         .receiveShadows(true)
         .castShadows(false)
         .build(cEngine, m_cFloor);
      filament::TransformManager& cTransforms = cEngine.getTransformManager();
      cTransforms.setTransform(
         cTransforms.getInstance(m_cFloor),
         TS(float(c_arena_center.GetX()), float(c_arena_center.GetY()), 0.0f,
            float(c_arena_size.GetX()), float(c_arena_size.GetY()), 1.0f));
      m_pcEngine->GetScene().addEntity(m_cFloor);
      m_pcIdScene->AddInstance(m_sPlaneMesh, m_cFloor,
                               1, UInt8(EPRClass::Floor));
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::SyncFloorTexture() {
      const UInt32 unSize = m_unFloorTextureSize;
      auto* punPixels = new UInt8[size_t(unSize) * unSize * 4];
      Real fMinX = m_cArenaCenter.GetX() - m_cArenaSize.GetX() * 0.5;
      Real fMinY = m_cArenaCenter.GetY() - m_cArenaSize.GetY() * 0.5;
      for(UInt32 j = 0; j < unSize; ++j) {
         Real fY = fMinY + (Real(j) + 0.5) / unSize * m_cArenaSize.GetY();
         for(UInt32 i = 0; i < unSize; ++i) {
            Real fX = fMinX + (Real(i) + 0.5) / unSize * m_cArenaSize.GetX();
            CColor cColor = m_pcFloorEntity->GetColorAtPoint(fX, fY);
            UInt8* punPixel = punPixels + (size_t(j) * unSize + i) * 4;
            punPixel[0] = cColor.GetRed();
            punPixel[1] = cColor.GetGreen();
            punPixel[2] = cColor.GetBlue();
            punPixel[3] = 255;
         }
      }
      filament::Texture::PixelBufferDescriptor cDescriptor(
         punPixels, size_t(unSize) * unSize * 4,
         filament::Texture::Format::RGBA,
         filament::Texture::Type::UBYTE,
         [](void* pt_buffer, size_t, void*) {
            delete[] static_cast<UInt8*>(pt_buffer);
         });
      m_pcFloorTexture->setImage(m_pcEngine->GetEngine(), 0,
                                 std::move(cDescriptor));
      m_pcFloorEntity->ClearChanged();
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::Destroy() {
      if(m_pcEngine == nullptr || !m_pcEngine->IsCreated()) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      for(auto& tPair : m_mapInstances) {
         RemoveInstance(tPair.second);
      }
      m_mapInstances.clear();
      if(m_cFloor) {
         m_pcIdScene->RemoveInstance(m_cFloor);
         m_pcEngine->GetScene().remove(m_cFloor);
         cEngine.destroy(m_cFloor);
         utils::EntityManager::get().destroy(m_cFloor);
      }
      if(m_pcFloorMaterial != nullptr) {
         cEngine.destroy(m_pcFloorMaterial);
         m_pcFloorMaterial = nullptr;
      }
      if(m_pcFloorTexture != nullptr) {
         cEngine.destroy(m_pcFloorTexture);
         m_pcFloorTexture = nullptr;
      }
      if(m_pcTexturedMaterial != nullptr) {
         cEngine.destroy(m_pcTexturedMaterial);
         m_pcTexturedMaterial = nullptr;
      }
      if(m_pcAmbientLight != nullptr) {
         m_pcEngine->GetScene().setIndirectLight(nullptr);
         cEngine.destroy(m_pcAmbientLight);
         m_pcAmbientLight = nullptr;
      }
      if(m_pcEnvironmentSkybox != nullptr) {
         m_pcEngine->GetScene().setSkybox(nullptr);
         cEngine.destroy(m_pcEnvironmentSkybox);
         m_pcEnvironmentSkybox = nullptr;
      }
      if(m_pcEnvironmentReflections != nullptr) {
         cEngine.destroy(m_pcEnvironmentReflections);
         m_pcEnvironmentReflections = nullptr;
      }
      if(m_pcEnvironmentSkyboxTexture != nullptr) {
         cEngine.destroy(m_pcEnvironmentSkyboxTexture);
         m_pcEnvironmentSkyboxTexture = nullptr;
      }
      if(m_cSunlight) {
         m_pcEngine->GetScene().remove(m_cSunlight);
         cEngine.destroy(m_cSunlight);
         utils::EntityManager::get().destroy(m_cSunlight);
      }
      m_sBoxMesh.Release(cEngine);
      m_sCylinderMesh.Release(cEngine);
      m_sPlaneMesh.Release(cEngine);
      m_pcEngine = nullptr;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::Sync(CSpace& c_space) {
      m_pcEngine->AssertRenderThread();
      /* The floor is created on the first sync (the floor entity does
       * not exist yet when Init() runs) and refreshed on change */
      if(m_bDrawFloor && !m_cFloor) {
         InitFloor(c_space, m_cArenaSize, m_cArenaCenter);
      }
      else if(m_pcFloorEntity != nullptr && m_pcFloorEntity->HasChanged()) {
         SyncFloorTexture();
      }
      /* Collect the embodied entities currently in the space */
      std::set<CEmbodiedEntity*> setCurrent;
      CEntity::TVector& vecRoots = c_space.GetRootEntityVector();
      for(CEntity* pcRoot : vecRoots) {
         auto* pcEmbodied = dynamic_cast<CEmbodiedEntity*>(pcRoot);
         if(pcEmbodied == nullptr) {
            auto* pcComposable = dynamic_cast<CComposableEntity*>(pcRoot);
            if(pcComposable != nullptr && pcComposable->HasComponent("body")) {
               pcEmbodied = &pcComposable->GetComponent<CEmbodiedEntity>("body");
            }
         }
         if(pcEmbodied != nullptr && !IsHidden(*pcEmbodied)) {
            setCurrent.insert(pcEmbodied);
         }
      }
      /* Remove instances whose entity left the space */
      for(auto itInstance = m_mapInstances.begin();
          itInstance != m_mapInstances.end(); ) {
         if(setCurrent.count(itInstance->first) == 0) {
            RemoveInstance(itInstance->second);
            itInstance = m_mapInstances.erase(itInstance);
         }
         else {
            ++itInstance;
         }
      }
      /* Add new entities */
      for(CEmbodiedEntity* pcEmbodied : setCurrent) {
         if(m_mapInstances.count(pcEmbodied) == 0) {
            AddEntity(*pcEmbodied);
         }
      }
      /* Update all transforms and LED emissives */
      for(auto& tPair : m_mapInstances) {
         UpdateInstance(*tPair.first, tPair.second);
      }
   }

   /****************************************/
   /****************************************/

   CPRSceneSync::SPart CPRSceneSync::MakePart(const SPRMesh& s_mesh,
                                              const mat4f& c_local,
                                              float f_r, float f_g, float f_b,
                                              float f_roughness,
                                              UInt16 un_entity_id,
                                              EPRClass e_class) {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      SPart sPart;
      sPart.Local = c_local;
      sPart.Material = m_pcEngine->GetLitMaterial().createInstance();
      sPart.Material->setParameter("baseColor", float3{f_r, f_g, f_b});
      sPart.Material->setParameter("roughness", f_roughness);
      sPart.BaseColor.Set(f_r, f_g, f_b);
      sPart.BaseRoughness = f_roughness;
      sPart.Material->setParameter("metallic", 0.0f);
      sPart.Material->setParameter("emissive", float3{0.0f});
      sPart.Renderable = utils::EntityManager::get().create();
      filament::RenderableManager::Builder(1)
         .boundingBox(s_mesh.Aabb)
         .material(0, sPart.Material)
         .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES,
                   s_mesh.Vertices, s_mesh.Indices)
         .receiveShadows(true)
         .castShadows(true)
         .build(cEngine, sPart.Renderable);
      m_pcEngine->GetScene().addEntity(sPart.Renderable);
      m_pcIdScene->AddInstance(s_mesh, sPart.Renderable,
                               un_entity_id, UInt8(e_class));
      return sPart;
   }

   /****************************************/
   /****************************************/

   bool CPRSceneSync::IsHidden(const CEmbodiedEntity& c_entity) const {
      if(m_vecHiddenIdPrefixes.empty()) {
         return false;
      }
      const std::string& strId = c_entity.GetRootEntity().GetId();
      for(const std::string& strPrefix : m_vecHiddenIdPrefixes) {
         if(strId.compare(0, strPrefix.size(), strPrefix) == 0) {
            return true;
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::SetHiddenIdPrefixes(const std::vector<std::string>& vec_prefixes) {
      m_vecHiddenIdPrefixes = vec_prefixes;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::SetDrawFloor(bool b_draw_floor) {
      m_bDrawFloor = b_draw_floor;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::AddEntity(CEmbodiedEntity& c_entity) {
      CEntity& cRoot = c_entity.GetRootEntity();
      const std::string& strType = cRoot.GetTypeDescription();
      SInstance sInstance;
      sInstance.EntityId = m_unNextEntityId++;
      sInstance.Type = strType;
      /* A glTF visual from the asset registry takes precedence over
       * the built-in visuals */
      const SPRVisualDescriptor* psDescriptor = m_pcAssets->GetDescriptor(strType);
      if(psDescriptor != nullptr) {
         sInstance.Gltf = m_pcAssets->CreateInstance(strType,
                                                     sInstance.EntityId,
                                                     psDescriptor->ClassId);
      }
      if(sInstance.Gltf.Main != nullptr) {
         sInstance.Class = EPRClass(psDescriptor->ClassId);
         sInstance.GltfOffset = psDescriptor->Offset;
      }
      else if(auto* pcBox = dynamic_cast<CBoxEntity*>(&cRoot)) {
         sInstance.Class = EPRClass::Box;
         sInstance.Parts.push_back(
            MakePart(m_sBoxMesh,
                     mat4f::scaling(float3{float(pcBox->GetSize().GetX()),
                                           float(pcBox->GetSize().GetY()),
                                           float(pcBox->GetSize().GetZ())}),
                     0.5f, 0.5f, 0.5f, 0.6f,
                     sInstance.EntityId, EPRClass::Box));
      }
      else if(auto* pcCylinder = dynamic_cast<CCylinderEntity*>(&cRoot)) {
         sInstance.Class = EPRClass::Cylinder;
         sInstance.Parts.push_back(
            MakePart(m_sCylinderMesh,
                     mat4f::scaling(float3{float(pcCylinder->GetRadius()),
                                           float(pcCylinder->GetRadius()),
                                           float(pcCylinder->GetHeight())}),
                     0.5f, 0.5f, 0.5f, 0.6f,
                     sInstance.EntityId, EPRClass::Cylinder));
      }
      else if(strType == "foot-bot") {
         sInstance.Class = EPRClass::FootBot;
         BuildFootBot(sInstance);
      }
      else if(strType == "drone") {
         sInstance.Class = EPRClass::Drone;
         BuildDrone(sInstance);
      }
      else {
         if(m_setWarnedTypes.insert(strType).second) {
            LOG << "[INFO] Photorealism: no visual model for entity type \""
                << strType << "\", skipping" << std::endl;
         }
         return;
      }
      /* Resolve LED components generically */
      if(auto* pcComposable = dynamic_cast<CComposableEntity*>(&cRoot)) {
         if(pcComposable->HasComponent("leds")) {
            CEntity& cComponent = pcComposable->GetComponent<CEntity>("leds");
            sInstance.LEDs = dynamic_cast<CLEDEquippedEntity*>(&cComponent);
            sInstance.DirectionalLEDs =
               dynamic_cast<CDirectionalLEDEquippedEntity*>(&cComponent);
         }
         else if(pcComposable->HasComponent("directional_leds")) {
            sInstance.DirectionalLEDs =
               &pcComposable->GetComponent<CDirectionalLEDEquippedEntity>("directional_leds");
         }
      }
      /* The LED cubes follow the LEDs' anchors, which the physics
       * engines only update while enabled. Media that consume the
       * anchors must enable them; without this, LED visuals freeze
       * at the start pose when no <led> medium is configured */
      if(sInstance.LEDs != nullptr) {
         for(CLEDEquippedEntity::SActuator* psActuator :
                sInstance.LEDs->GetLEDs()) {
            psActuator->Anchor.Enable();
         }
      }
      if(sInstance.DirectionalLEDs != nullptr) {
         for(CDirectionalLEDEquippedEntity::SInstance& sLED :
                sInstance.DirectionalLEDs->GetInstances()) {
            sLED.Anchor.Enable();
         }
      }
      m_mapInstances[&c_entity] = sInstance;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::BuildFootBot(SInstance& s_instance) {
      const UInt16 unId = s_instance.EntityId;
      /* Wheels: unit cylinder rotated so its axis lies along y */
      const quatf cXRot(0.7071068f, 0.7071068f, 0.0f, 0.0f);
      s_instance.Parts.push_back(
         MakePart(m_sCylinderMesh,
                  TRS(0.0f, FOOTBOT_HALF_INTERWHEEL + FOOTBOT_WHEEL_WIDTH * 0.5f,
                      FOOTBOT_WHEEL_RADIUS, cXRot,
                      FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_RADIUS,
                      FOOTBOT_WHEEL_WIDTH),
                  0.15f, 0.15f, 0.15f, 0.8f, unId, EPRClass::FootBot));
      s_instance.Parts.push_back(
         MakePart(m_sCylinderMesh,
                  TRS(0.0f, -FOOTBOT_HALF_INTERWHEEL + FOOTBOT_WHEEL_WIDTH * 0.5f,
                      FOOTBOT_WHEEL_RADIUS, cXRot,
                      FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_RADIUS,
                      FOOTBOT_WHEEL_WIDTH),
                  0.15f, 0.15f, 0.15f, 0.8f, unId, EPRClass::FootBot));
      /* Base module */
      s_instance.Parts.push_back(
         MakePart(m_sCylinderMesh,
                  TS(0.0f, 0.0f, 0.032f,
                     FOOTBOT_BODY_RADIUS, FOOTBOT_BODY_RADIUS, 0.043f),
                  0.55f, 0.55f, 0.55f, 0.5f, unId, EPRClass::FootBot));
      /* Turret module (the LED ring mounts around this) */
      s_instance.Parts.push_back(
         MakePart(m_sCylinderMesh,
                  TS(0.0f, 0.0f, 0.075f,
                     FOOTBOT_TURRET_RADIUS, FOOTBOT_TURRET_RADIUS, 0.055f),
                  0.65f, 0.65f, 0.65f, 0.5f, unId, EPRClass::FootBot));
      /* Distance scanner / RAB / camera pole */
      s_instance.Parts.push_back(
         MakePart(m_sCylinderMesh,
                  TS(0.0f, 0.0f, 0.13f,
                     0.015f, 0.015f, FOOTBOT_HEIGHT - 0.143f),
                  0.2f, 0.2f, 0.2f, 0.7f, unId, EPRClass::FootBot));
      /* Camera mirror housing on top */
      s_instance.Parts.push_back(
         MakePart(m_sBoxMesh,
                  TS(0.0f, 0.0f, FOOTBOT_HEIGHT - 0.013f,
                     0.03f, 0.03f, 0.013f),
                  0.1f, 0.1f, 0.1f, 0.4f, unId, EPRClass::FootBot));
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::BuildDrone(SInstance& s_instance) {
      const UInt16 unId = s_instance.EntityId;
      /* Central body */
      s_instance.Parts.push_back(
         MakePart(m_sBoxMesh,
                  TS(0.0f, 0.0f, 0.06f, 0.1f, 0.1f, 0.04f),
                  0.2f, 0.2f, 0.22f, 0.5f, unId, EPRClass::Drone));
      /* Four arms along the axes, rotors at the LED positions (0.1) */
      const float pfArmX[] = { 1.0f, 0.0f, -1.0f, 0.0f };
      const float pfArmY[] = { 0.0f, 1.0f, 0.0f, -1.0f };
      for(UInt32 i = 0; i < 4; ++i) {
         s_instance.Parts.push_back(
            MakePart(m_sBoxMesh,
                     TS(pfArmX[i] * 0.055f, pfArmY[i] * 0.055f, 0.085f,
                        pfArmX[i] != 0.0f ? 0.09f : 0.015f,
                        pfArmY[i] != 0.0f ? 0.09f : 0.015f,
                        0.01f),
                     0.3f, 0.3f, 0.3f, 0.6f, unId, EPRClass::Drone));
         s_instance.Parts.push_back(
            MakePart(m_sCylinderMesh,
                     TS(pfArmX[i] * 0.1f, pfArmY[i] * 0.1f, 0.095f,
                        0.04f, 0.04f, 0.006f),
                     0.12f, 0.12f, 0.12f, 0.7f, unId, EPRClass::Drone));
      }
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::RemoveInstance(SInstance& s_instance) {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      if(s_instance.Gltf.Main != nullptr) {
         m_pcAssets->ReleaseInstance(s_instance.Type, s_instance.Gltf);
      }
      /* No Disable() to balance the anchor Enable() calls made in
       * AddEntity: instances are removed only after their entity has
       * already been destroyed (the diff in Sync() and the simulator
       * teardown both run after entity destruction), so the LED
       * component pointers must not be dereferenced here */
      for(SPart& sPart : s_instance.Parts) {
         m_pcIdScene->RemoveInstance(sPart.Renderable);
         m_pcEngine->GetScene().remove(sPart.Renderable);
         cEngine.destroy(sPart.Renderable);
         utils::EntityManager::get().destroy(sPart.Renderable);
         if(sPart.Material != nullptr) {
            cEngine.destroy(sPart.Material);
         }
      }
      s_instance.Parts.clear();
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::UpdateInstance(CEmbodiedEntity& c_entity,
                                     SInstance& s_instance) {
      /* Create LED parts on the first update */
      UInt32 unLEDCount = 0;
      if(s_instance.LEDs != nullptr) {
         unLEDCount = s_instance.LEDs->GetLEDs().size();
      }
      else if(s_instance.DirectionalLEDs != nullptr) {
         unLEDCount = s_instance.DirectionalLEDs->GetInstances().size();
      }
      bool bHasLEDParts = false;
      for(const SPart& sPart : s_instance.Parts) {
         if(sPart.LEDIndex >= 0) {
            bHasLEDParts = true;
            break;
         }
      }
      if(!bHasLEDParts && unLEDCount > 0) {
         for(UInt32 i = 0; i < unLEDCount; ++i) {
            /* Emissive cube centered on the LED position */
            SPart sPart = MakePart(m_sBoxMesh,
                                   TS(0.0f, 0.0f, -LED_CUBE_SIZE * 0.5f,
                                      LED_CUBE_SIZE, LED_CUBE_SIZE, LED_CUBE_SIZE),
                                   0.05f, 0.05f, 0.05f, 0.5f,
                                   s_instance.EntityId,
                                   s_instance.Class);
            sPart.LEDIndex = SInt32(i);
            s_instance.Parts.push_back(sPart);
         }
      }
      /* Origin anchor transform */
      const SAnchor& sAnchor = c_entity.GetOriginAnchor();
      mat4f cAnchorTransform =
         mat4f::translation(float3{float(sAnchor.Position.GetX()),
                                   float(sAnchor.Position.GetY()),
                                   float(sAnchor.Position.GetZ())}) *
         mat4f(quatf(float(sAnchor.Orientation.GetW()),
                     float(sAnchor.Orientation.GetX()),
                     float(sAnchor.Orientation.GetY()),
                     float(sAnchor.Orientation.GetZ())));
      filament::TransformManager& cTransforms =
         m_pcEngine->GetEngine().getTransformManager();
      /* The glTF visual follows the origin anchor (the aux instance
       * is parented to the main one) */
      if(s_instance.Gltf.Main != nullptr) {
         cTransforms.setTransform(
            cTransforms.getInstance(s_instance.Gltf.Main->getRoot()),
            cAnchorTransform * s_instance.GltfOffset);
      }
      for(SPart& sPart : s_instance.Parts) {
         if(sPart.LEDIndex < 0) {
            cTransforms.setTransform(
               cTransforms.getInstance(sPart.Renderable),
               cAnchorTransform * sPart.Local);
         }
         else {
            /* LED parts follow the LED's anchor and offset. The
             * global position is computed here because the LED
             * entities themselves are only kept up to date when they
             * are attached to an LED medium */
            CVector3 cPosition;
            CColor cColor;
            if(s_instance.LEDs != nullptr) {
               const CLEDEquippedEntity::SActuator& sActuator =
                  *s_instance.LEDs->GetLEDs()[sPart.LEDIndex];
               cPosition = sActuator.Offset;
               cPosition.Rotate(sActuator.Anchor.Orientation);
               cPosition += sActuator.Anchor.Position;
               cColor = sActuator.LED.GetColor();
            }
            else {
               const CDirectionalLEDEquippedEntity::SInstance& sLED =
                  s_instance.DirectionalLEDs->GetInstances()[sPart.LEDIndex];
               cPosition = sLED.PositionOffset;
               cPosition.Rotate(sLED.Anchor.Orientation);
               cPosition += sLED.Anchor.Position;
               cColor = sLED.LED.GetColor();
            }
            cTransforms.setTransform(
               cTransforms.getInstance(sPart.Renderable),
               mat4f::translation(float3{float(cPosition.GetX()),
                                         float(cPosition.GetY()),
                                         float(cPosition.GetZ())}) *
               sPart.Local);
            sPart.Material->setParameter(
               "emissive",
               float3{float(cColor.GetRed()) / 255.0f,
                      float(cColor.GetGreen()) / 255.0f,
                      float(cColor.GetBlue()) / 255.0f} * LED_EMISSIVE_NITS);
         }
      }
   }

   /****************************************/
   /****************************************/

   UInt16 CPRSceneSync::GetEntityId(const CEmbodiedEntity& c_entity) const {
      auto itInstance = m_mapInstances.find(
         const_cast<CEmbodiedEntity*>(&c_entity));
      return itInstance != m_mapInstances.end() ?
         itInstance->second.EntityId : 0;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::SetSunlight(const CVector3& c_direction,
                                  Real f_intensity) {
      filament::LightManager& cLights =
         m_pcEngine->GetEngine().getLightManager();
      filament::LightManager::Instance cSun =
         cLights.getInstance(m_cSunlight);
      CVector3 cDirection(c_direction);
      cDirection.Normalize();
      cLights.setDirection(cSun, {float(cDirection.GetX()),
                                  float(cDirection.GetY()),
                                  float(cDirection.GetZ())});
      cLights.setIntensity(cSun, float(f_intensity));
      /* The ambient light follows the sun, unless it comes from an
       * HDR environment with its own intensity */
      if(m_pcAmbientLight != nullptr && !m_bHasEnvironment) {
         m_pcAmbientLight->setIntensity(float(f_intensity) * 0.2f);
      }
   }

   /****************************************/
   /****************************************/

   static filament::Texture* LoadKtxTexture(filament::Engine& c_engine,
                                            const std::string& str_file,
                                            image::Ktx1Bundle** ppc_bundle) {
      std::ifstream cFile(str_file, std::ios::binary | std::ios::ate);
      if(!cFile) {
         THROW_ARGOSEXCEPTION("Cannot open environment map \""
                              << str_file << "\"");
      }
      std::vector<UInt8> vecBytes(size_t(cFile.tellg()));
      cFile.seekg(0);
      cFile.read(reinterpret_cast<char*>(vecBytes.data()), vecBytes.size());
      auto* pcBundle = new image::Ktx1Bundle(vecBytes.data(),
                                             uint32_t(vecBytes.size()));
      if(ppc_bundle != nullptr) {
         *ppc_bundle = pcBundle;
      }
      /* The reader destroys the bundle once the data is uploaded */
      return ktxreader::Ktx1Reader::createTexture(&c_engine, pcBundle, false);
   }

   void CPRSceneSync::LoadEnvironment(const std::string& str_ibl_file,
                                      const std::string& str_skybox_file,
                                      Real f_intensity) {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      /* The prefiltered reflections cubemap carries the irradiance
       * spherical harmonics in its metadata */
      image::Ktx1Bundle* pcIblBundle = nullptr;
      m_pcEnvironmentReflections =
         LoadKtxTexture(cEngine, str_ibl_file, &pcIblBundle);
      float3 pcSphericalHarmonics[9];
      if(!pcIblBundle->getSphericalHarmonics(pcSphericalHarmonics)) {
         THROW_ARGOSEXCEPTION("\"" << str_ibl_file << "\" carries no "
                              "irradiance data; generate it with cmgen");
      }
      if(m_pcAmbientLight != nullptr) {
         cEngine.destroy(m_pcAmbientLight);
      }
      m_pcAmbientLight = filament::IndirectLight::Builder()
         .reflections(m_pcEnvironmentReflections)
         .irradiance(3, pcSphericalHarmonics)
         .intensity(float(f_intensity))
         .build(cEngine);
      m_pcEngine->GetScene().setIndirectLight(m_pcAmbientLight);
      if(!str_skybox_file.empty()) {
         m_pcEnvironmentSkyboxTexture =
            LoadKtxTexture(cEngine, str_skybox_file, nullptr);
         m_pcEnvironmentSkybox = filament::Skybox::Builder()
            .environment(m_pcEnvironmentSkyboxTexture)
            .intensity(float(f_intensity))
            .build(cEngine);
         m_pcEngine->GetScene().setSkybox(m_pcEnvironmentSkybox);
      }
      m_bHasEnvironment = true;
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::SetMaterialParam(const CEmbodiedEntity& c_entity,
                                       const std::string& str_name,
                                       Real f_value) {
      if(str_name != "roughness" && str_name != "metallic") {
         THROW_ARGOSEXCEPTION("Unknown material parameter \"" << str_name
                              << "\"; use \"roughness\" or \"metallic\"");
      }
      auto itInstance = m_mapInstances.find(
         const_cast<CEmbodiedEntity*>(&c_entity));
      if(itInstance == m_mapInstances.end()) {
         THROW_ARGOSEXCEPTION("Entity \""
                              << c_entity.GetRootEntity().GetId()
                              << "\" has no visual in the photorealism medium");
      }
      for(SPart& s_part : itInstance->second.Parts) {
         if(s_part.LEDIndex < 0) {
            s_part.Material->setParameter(str_name.c_str(), float(f_value));
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::SetMaterialColor(const CEmbodiedEntity& c_entity,
                                       const CVector3& c_color) {
      auto itInstance = m_mapInstances.find(
         const_cast<CEmbodiedEntity*>(&c_entity));
      if(itInstance == m_mapInstances.end()) {
         THROW_ARGOSEXCEPTION("Entity \""
                              << c_entity.GetRootEntity().GetId()
                              << "\" has no visual in the photorealism medium");
      }
      for(SPart& s_part : itInstance->second.Parts) {
         if(s_part.LEDIndex < 0) {
            s_part.Material->setParameter("baseColor",
                                          float3{float(c_color.GetX()),
                                                 float(c_color.GetY()),
                                                 float(c_color.GetZ())});
            s_part.BaseColor = c_color;
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPRSceneSync::RandomizeMaterials(CRandom::CRNG& c_rng,
                                         const std::set<EPRClass>& set_classes,
                                         const CRange<Real>& c_roughness,
                                         Real f_color_jitter) {
      bool bRandomizeRoughness = c_roughness.GetSpan() > 0.0;
      auto fnJitter = [&c_rng, f_color_jitter](Real f_channel) {
         Real fFactor = 1.0 + c_rng.Uniform(CRange<Real>(-f_color_jitter,
                                                         f_color_jitter));
         return std::min(1.0, std::max(0.0, f_channel * fFactor));
      };
      for(auto& tInstance : m_mapInstances) {
         SInstance& sInstance = tInstance.second;
         if(set_classes.count(sInstance.Class) == 0) {
            continue;
         }
         for(SPart& s_part : sInstance.Parts) {
            if(s_part.LEDIndex >= 0) {
               continue;
            }
            if(bRandomizeRoughness) {
               s_part.Material->setParameter(
                  "roughness", float(c_rng.Uniform(c_roughness)));
            }
            if(f_color_jitter > 0.0) {
               s_part.Material->setParameter(
                  "baseColor",
                  float3{float(fnJitter(s_part.BaseColor.GetX())),
                         float(fnJitter(s_part.BaseColor.GetY())),
                         float(fnJitter(s_part.BaseColor.GetZ()))});
            }
         }
      }
      /* The floor material is not part of an instance */
      if(set_classes.count(EPRClass::Floor) != 0 &&
         m_pcFloorMaterial != nullptr && bRandomizeRoughness) {
         m_pcFloorMaterial->setParameter(
            "roughness", float(c_rng.Uniform(c_roughness)));
      }
   }

   /****************************************/
   /****************************************/

}
