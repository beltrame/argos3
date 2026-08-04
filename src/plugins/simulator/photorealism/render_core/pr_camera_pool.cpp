/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_camera_pool.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_camera_pool.h"
#include "pr_render_engine.h"
#include "pr_id_scene.h"

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/configuration/argos_exception.h>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <filament/Camera.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>
#include <filament/RenderTarget.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>
#include <backend/PixelBufferDescriptor.h>
#include <utils/EntityManager.h>
#include <math/mat4.h>
#include <math/quat.h>

#include <algorithm>
#include <cstring>

namespace argos {

   using filament::math::float3;
   using filament::math::mat3f;
   using filament::math::mat4f;
   using filament::math::quatf;

   /* Generated from materials/pr_blit.mat at build time */
   extern const unsigned char PR_BLIT_FILAMAT[];
   extern const size_t PR_BLIT_FILAMAT_SIZE;

   /** Atlas pages never exceed this side length */
   static const UInt32 PAGE_MAX_SIDE = 4096;

   /****************************************/
   /****************************************/

   void CPRCameraPool::Init(CPRRenderEngine& c_engine,
                            CPRIdScene& c_id_scene,
                            bool b_immediate) {
      m_pcEngine = &c_engine;
      m_pcIdScene = &c_id_scene;
      m_bImmediate = b_immediate;
      /* Clear color for views not covered by the skybox (aux pass);
       * entity id 0 = nothing, depth fixed up on the CPU side */
      filament::Renderer::ClearOptions sClearOptions;
      sClearOptions.clearColor = {0.0f, 0.0f, 0.0f, 0.0f};
      sClearOptions.clear = true;
      c_engine.GetRenderer().setClearOptions(sClearOptions);
      m_pcBlitMaterial = filament::Material::Builder()
         .package(PR_BLIT_FILAMAT, PR_BLIT_FILAMAT_SIZE)
         .build(c_engine.GetEngine());
      if(m_pcBlitMaterial == nullptr) {
         THROW_ARGOSEXCEPTION("Failed to load the embedded pr_blit material");
      }
      m_sQuadMesh = CPRMeshBuilder::BuildPlane(c_engine.GetEngine());
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::Destroy() {
      if(m_pcEngine == nullptr || !m_pcEngine->IsCreated()) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      cEngine.flushAndWait();
      DestroyPages();
      for(auto& tCamera : m_mapCameras) {
         ReleaseResources(tCamera.second);
      }
      m_mapCameras.clear();
      cEngine.destroy(m_pcBlitMaterial);
      m_pcBlitMaterial = nullptr;
      m_sQuadMesh.Release(cEngine);
      m_pcEngine = nullptr;
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::Reset() {
      if(m_pcEngine != nullptr && m_pcEngine->IsCreated()) {
         m_pcEngine->GetEngine().flushAndWait();
      }
      for(auto& tCamera : m_mapCameras) {
         SCamera& sCamera = tCamera.second;
         sCamera.Pending = false;
         sCamera.Front.Fresh = false;
         sCamera.Front.Valid = false;
         sCamera.Front.Tick = 0;
      }
      for(auto& psPage : m_vecPages) {
         psPage->Pending = false;
         psPage->Done = true;
      }
   }

   /****************************************/
   /****************************************/

   UInt32 CPRCameraPool::RegisterCamera(const SPRCameraConfig& s_config) {
      if(s_config.Anchor == nullptr) {
         THROW_ARGOSEXCEPTION("Photorealistic camera registered without an anchor");
      }
      if(s_config.Width > PAGE_MAX_SIDE || s_config.Height > PAGE_MAX_SIDE) {
         THROW_ARGOSEXCEPTION("Photorealistic camera resolution exceeds "
                              << PAGE_MAX_SIDE << " pixels per side");
      }
      UInt32 unHandle = m_unNextHandle++;
      SCamera& sCamera = m_mapCameras[unHandle];
      sCamera.Config = s_config;
      m_sStats.PeakCameras = std::max(m_sStats.PeakCameras, m_mapCameras.size());
      size_t unPixels = size_t(s_config.Width) * s_config.Height;
      sCamera.Front.RGBA.resize(unPixels * 4, 0);
      sCamera.Front.Aux.resize(unPixels * 4, 0.0f);
      m_bPagesDirty = true;
      return unHandle;
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::UnregisterCamera(UInt32 un_handle) {
      auto itCamera = m_mapCameras.find(un_handle);
      if(itCamera == m_mapCameras.end()) {
         return;
      }
      if(m_pcEngine != nullptr && m_pcEngine->IsCreated()) {
         m_pcEngine->GetEngine().flushAndWait();
         /* The pages hold material instances sampling this camera's
          * textures; drop them before the textures go away */
         DestroyPages();
         ReleaseResources(itCamera->second);
      }
      m_mapCameras.erase(itCamera);
      m_bPagesDirty = true;
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::Update(UInt32 un_tick) {
      m_pcEngine->AssertRenderThread();
      using TClock = std::chrono::steady_clock;
      TClock::time_point tStart = TClock::now();
      ++m_sStats.Updates;
      /*
       * Phase 1: collect the readbacks issued at the previous tick
       */
      for(auto& tCamera : m_mapCameras) {
         tCamera.second.Front.Fresh = false;
      }
      bool bWaitNeeded = false;
      for(auto& psPage : m_vecPages) {
         if(psPage->Pending && !psPage->Done) {
            bWaitNeeded = true;
         }
      }
      if(bWaitNeeded) {
         m_pcEngine->GetEngine().flushAndWait();
      }
      DemuxPages();
      TClock::time_point tCollected = TClock::now();
      m_sStats.CollectWait +=
         std::chrono::duration<double>(tCollected - tStart).count();
      /*
       * Phase 2: render all cameras due this tick
       */
      std::vector<SCamera*> vecDue;
      for(auto& tCamera : m_mapCameras) {
         SCamera& sCamera = tCamera.second;
         sCamera.Due = false;
         if((un_tick + tCamera.first) % sCamera.Config.FramerateDivider == 0) {
            sCamera.Due = true;
            vecDue.push_back(&sCamera);
         }
      }
      if(vecDue.empty()) {
         return;
      }
      if(m_bPagesDirty) {
         RebuildPages();
      }
      filament::Renderer& cRenderer = m_pcEngine->GetRenderer();
      /* beginFrame() returning false is only a frame-skipping HINT
       * (e.g. when a window renderer shares the engine and the GPU
       * lags); sensor frames are not optional, so it is ignored */
      cRenderer.beginFrame(&m_pcEngine->GetSwapChain());
      for(SCamera* psCamera : vecDue) {
         UpdateCameraTransform(*psCamera);
         if(psCamera->Config.RenderRGB) {
            cRenderer.render(psCamera->RGBView);
         }
         if(psCamera->Config.RenderAux) {
            cRenderer.render(psCamera->AuxView);
         }
         psCamera->Pending = true;
         psCamera->RenderTick = un_tick;
      }
      /* Composite the camera textures into the atlas pages and issue
       * one readPixels per page */
      for(auto& psPage : m_vecPages) {
         bool bDue = false;
         for(const STile& sTile : psPage->Tiles) {
            if(m_mapCameras.at(sTile.CameraHandle).Due) {
               bDue = true;
               break;
            }
         }
         if(!bDue) {
            continue;
         }
         cRenderer.render(psPage->View);
         psPage->Done = false;
         psPage->Pending = true;
         filament::backend::PixelBufferDescriptor cDescriptor(
            psPage->Buffer.data(), psPage->Buffer.size(),
            filament::backend::PixelDataFormat::RGBA,
            psPage->Float ? filament::backend::PixelDataType::FLOAT
                          : filament::backend::PixelDataType::UBYTE,
            [](void*, size_t, void* pt_user) {
               static_cast<SPage*>(pt_user)->Done = true;
            },
            psPage.get());
         cRenderer.readPixels(psPage->Target, 0, 0,
                              psPage->Width, psPage->Height,
                              std::move(cDescriptor));
      }
      cRenderer.endFrame();
      m_sStats.CamerasRendered += vecDue.size();
      TClock::time_point tSubmitted = TClock::now();
      m_sStats.Submit +=
         std::chrono::duration<double>(tSubmitted - tCollected).count();
      /*
       * Immediate mode: block until this tick's frames are readable
       */
      if(m_bImmediate) {
         m_pcEngine->GetEngine().flushAndWait();
         DemuxPages();
         m_sStats.ImmediateWait +=
            std::chrono::duration<double>(TClock::now() - tSubmitted).count();
      }
      double fTotal =
         std::chrono::duration<double>(TClock::now() - tStart).count();
      if(fTotal > m_sStats.MaxUpdate) {
         m_sStats.MaxUpdate = fTotal;
      }
   }

   /****************************************/
   /****************************************/

   const CPRCameraPool::SOutput&
   CPRCameraPool::GetOutput(UInt32 un_handle) const {
      auto itCamera = m_mapCameras.find(un_handle);
      if(itCamera == m_mapCameras.end()) {
         THROW_ARGOSEXCEPTION("Unknown photorealistic camera handle " << un_handle);
      }
      return itCamera->second.Front;
   }

   /****************************************/
   /****************************************/

   const SPRCameraConfig&
   CPRCameraPool::GetConfig(UInt32 un_handle) const {
      auto itCamera = m_mapCameras.find(un_handle);
      if(itCamera == m_mapCameras.end()) {
         THROW_ARGOSEXCEPTION("Unknown photorealistic camera handle " << un_handle);
      }
      return itCamera->second.Config;
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::DemuxPages() {
      for(auto& psPage : m_vecPages) {
         if(!psPage->Pending) {
            continue;
         }
         if(!psPage->Done) {
            THROW_ARGOSEXCEPTION("Photorealistic camera readback did not complete");
         }
         for(const STile& sTile : psPage->Tiles) {
            SCamera& sCamera = m_mapCameras.at(sTile.CameraHandle);
            if(!sCamera.Pending) {
               continue;
            }
            if(psPage->Float) {
               const float* pfSrc =
                  reinterpret_cast<const float*>(psPage->Buffer.data());
               for(UInt32 r = 0; r < sTile.Height; ++r) {
                  std::memcpy(
                     sCamera.Front.Aux.data() + size_t(r) * sTile.Width * 4,
                     pfSrc + (size_t(sTile.Y + r) * psPage->Width + sTile.X) * 4,
                     size_t(sTile.Width) * 4 * sizeof(float));
               }
            }
            else {
               for(UInt32 r = 0; r < sTile.Height; ++r) {
                  std::memcpy(
                     sCamera.Front.RGBA.data() + size_t(r) * sTile.Width * 4,
                     psPage->Buffer.data() +
                        (size_t(sTile.Y + r) * psPage->Width + sTile.X) * 4,
                     size_t(sTile.Width) * 4);
               }
            }
            sCamera.Front.Tick = sCamera.RenderTick;
            sCamera.Front.Fresh = true;
            sCamera.Front.Valid = true;
         }
         psPage->Pending = false;
      }
      /* A camera is collected once both its pages (RGB and aux) have
       * been demuxed; since pages are read back together, clearing
       * the flags afterwards keeps the loop above simple */
      for(auto& tCamera : m_mapCameras) {
         if(tCamera.second.Pending && tCamera.second.Front.Fresh) {
            tCamera.second.Pending = false;
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::RebuildPages() {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      cEngine.flushAndWait();
      DestroyPages();
      for(auto& tCamera : m_mapCameras) {
         EnsureResources(tCamera.second);
      }
      BuildPages(false);
      BuildPages(true);
      m_bPagesDirty = false;
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::BuildPages(bool b_float) {
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      /* Collect the cameras rendering this modality, tallest first
       * (shelf packing) */
      struct SEntry {
         UInt32 Handle;
         SCamera* Camera;
      };
      std::vector<SEntry> vecEntries;
      size_t unTotalArea = 0;
      UInt32 unWidest = 0;
      for(auto& tCamera : m_mapCameras) {
         SCamera& sCamera = tCamera.second;
         if(b_float ? sCamera.Config.RenderAux : sCamera.Config.RenderRGB) {
            vecEntries.push_back({tCamera.first, &sCamera});
            unTotalArea += size_t(sCamera.Config.Width) * sCamera.Config.Height;
            unWidest = std::max(unWidest, sCamera.Config.Width);
         }
      }
      if(vecEntries.empty()) {
         return;
      }
      std::sort(vecEntries.begin(), vecEntries.end(),
                [](const SEntry& s_a, const SEntry& s_b) {
                   return s_a.Camera->Config.Height > s_b.Camera->Config.Height;
                });
      /* Page width: roughly square total footprint, power of two */
      UInt32 unPageWidth = 64;
      while(unPageWidth < unWidest ||
            (size_t(unPageWidth) * unPageWidth < unTotalArea &&
             unPageWidth < PAGE_MAX_SIDE)) {
         unPageWidth *= 2;
      }
      unPageWidth = std::min(unPageWidth, PAGE_MAX_SIDE);
      /* Shelf-pack the tiles into pages */
      SPage* psPage = nullptr;
      UInt32 unX = 0, unY = 0, unShelfHeight = 0;
      auto fnClosePage = [&]() {
         if(psPage != nullptr) {
            psPage->Height = unY + unShelfHeight;
         }
         psPage = nullptr;
      };
      for(SEntry& sEntry : vecEntries) {
         UInt32 unWidth = sEntry.Camera->Config.Width;
         UInt32 unHeight = sEntry.Camera->Config.Height;
         if(psPage != nullptr && unX + unWidth > unPageWidth) {
            /* New shelf */
            unY += unShelfHeight;
            unX = 0;
            unShelfHeight = 0;
            if(unY + unHeight > PAGE_MAX_SIDE) {
               fnClosePage();
            }
         }
         if(psPage == nullptr) {
            m_vecPages.push_back(std::make_unique<SPage>());
            psPage = m_vecPages.back().get();
            psPage->Float = b_float;
            psPage->Width = unPageWidth;
            unX = 0;
            unY = 0;
            unShelfHeight = 0;
         }
         STile sTile;
         sTile.CameraHandle = sEntry.Handle;
         sTile.X = unX;
         sTile.Y = unY;
         sTile.Width = unWidth;
         sTile.Height = unHeight;
         psPage->Tiles.push_back(sTile);
         unX += unWidth;
         unShelfHeight = std::max(unShelfHeight, unHeight);
      }
      fnClosePage();
      /* Create the GPU resources and blit quads of the new pages */
      filament::TextureSampler cSampler(
         filament::TextureSampler::MinFilter::NEAREST,
         filament::TextureSampler::MagFilter::NEAREST);
      filament::TransformManager& cTransforms = cEngine.getTransformManager();
      for(auto& psNewPage : m_vecPages) {
         SPage& sPage = *psNewPage;
         if(sPage.Float != b_float || sPage.Scene != nullptr) {
            continue;
         }
         sPage.Color = filament::Texture::Builder()
            .width(sPage.Width).height(sPage.Height).levels(1)
            .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                   filament::Texture::Usage::BLIT_SRC)
            .format(b_float ? filament::Texture::InternalFormat::RGBA32F
                            : filament::Texture::InternalFormat::RGBA8)
            .build(cEngine);
         sPage.Depth = filament::Texture::Builder()
            .width(sPage.Width).height(sPage.Height).levels(1)
            .usage(filament::Texture::Usage::DEPTH_ATTACHMENT)
            .format(filament::Texture::InternalFormat::DEPTH32F)
            .build(cEngine);
         sPage.Target = filament::RenderTarget::Builder()
            .texture(filament::RenderTarget::AttachmentPoint::COLOR, sPage.Color)
            .texture(filament::RenderTarget::AttachmentPoint::DEPTH, sPage.Depth)
            .build(cEngine);
         sPage.Scene = cEngine.createScene();
         sPage.CameraEntity = utils::EntityManager::get().create();
         sPage.Camera = cEngine.createCamera(sPage.CameraEntity);
         /* Atlas coordinates: x right, y down from the top-left */
         sPage.Camera->setProjection(
            filament::Camera::Projection::ORTHO,
            0.0, double(sPage.Width), double(sPage.Height), 0.0,
            0.1, 10.0);
         sPage.View = cEngine.createView();
         sPage.View->setViewport(
            filament::Viewport(0, 0, sPage.Width, sPage.Height));
         sPage.View->setRenderTarget(sPage.Target);
         sPage.View->setScene(sPage.Scene);
         sPage.View->setCamera(sPage.Camera);
         sPage.View->setPostProcessingEnabled(false);
         sPage.View->setShadowingEnabled(false);
         sPage.Buffer.assign(size_t(sPage.Width) * sPage.Height * 4 *
                             (b_float ? sizeof(float) : 1), 0);
         for(STile& sTile : sPage.Tiles) {
            SCamera& sCamera = m_mapCameras.at(sTile.CameraHandle);
            sTile.Material = m_pcBlitMaterial->createInstance();
            sTile.Material->setParameter(
               "tex", b_float ? sCamera.AuxColor : sCamera.RGBColor, cSampler);
            sTile.Quad = utils::EntityManager::get().create();
            filament::RenderableManager::Builder(1)
               .boundingBox(m_sQuadMesh.Aabb)
               .material(0, sTile.Material)
               .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES,
                         m_sQuadMesh.Vertices, m_sQuadMesh.Indices)
               .castShadows(false)
               .receiveShadows(false)
               .culling(false)
               .build(cEngine, sTile.Quad);
            cTransforms.setTransform(
               cTransforms.getInstance(sTile.Quad),
               mat4f::translation(float3{sTile.X + sTile.Width * 0.5f,
                                         sTile.Y + sTile.Height * 0.5f,
                                         -1.0f}) *
               mat4f::scaling(float3{float(sTile.Width),
                                     float(sTile.Height),
                                     1.0f}));
            sPage.Scene->addEntity(sTile.Quad);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::DestroyPages() {
      if(m_vecPages.empty()) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      for(auto& psPage : m_vecPages) {
         SPage& sPage = *psPage;
         for(STile& sTile : sPage.Tiles) {
            sPage.Scene->remove(sTile.Quad);
            cEngine.destroy(sTile.Quad);
            utils::EntityManager::get().destroy(sTile.Quad);
            cEngine.destroy(sTile.Material);
         }
         cEngine.destroy(sPage.View);
         cEngine.destroy(sPage.Target);
         cEngine.destroy(sPage.Color);
         cEngine.destroy(sPage.Depth);
         cEngine.destroyCameraComponent(sPage.CameraEntity);
         utils::EntityManager::get().destroy(sPage.CameraEntity);
         cEngine.destroy(sPage.Scene);
      }
      m_vecPages.clear();
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::EnsureResources(SCamera& s_camera) {
      if(s_camera.Created) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      const SPRCameraConfig& sConfig = s_camera.Config;
      s_camera.CameraEntity = utils::EntityManager::get().create();
      s_camera.Camera = cEngine.createCamera(s_camera.CameraEntity);
      s_camera.Camera->setProjection(
         double(sConfig.FieldOfView),
         double(sConfig.Width) / double(sConfig.Height),
         double(sConfig.NearPlane), double(sConfig.FarPlane),
         filament::Camera::Fov::VERTICAL);
      m_pcEngine->ApplyExposure(*s_camera.Camera);
      if(sConfig.RenderRGB) {
         s_camera.RGBColor = filament::Texture::Builder()
            .width(sConfig.Width).height(sConfig.Height).levels(1)
            .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                   filament::Texture::Usage::SAMPLEABLE)
            .format(filament::Texture::InternalFormat::RGBA8)
            .build(cEngine);
         s_camera.RGBDepth = filament::Texture::Builder()
            .width(sConfig.Width).height(sConfig.Height).levels(1)
            .usage(filament::Texture::Usage::DEPTH_ATTACHMENT)
            .format(filament::Texture::InternalFormat::DEPTH32F)
            .build(cEngine);
         s_camera.RGBTarget = filament::RenderTarget::Builder()
            .texture(filament::RenderTarget::AttachmentPoint::COLOR, s_camera.RGBColor)
            .texture(filament::RenderTarget::AttachmentPoint::DEPTH, s_camera.RGBDepth)
            .build(cEngine);
         s_camera.RGBView = cEngine.createView();
         s_camera.RGBView->setViewport(
            filament::Viewport(0, 0, sConfig.Width, sConfig.Height));
         s_camera.RGBView->setRenderTarget(s_camera.RGBTarget);
         s_camera.RGBView->setScene(&m_pcEngine->GetScene());
         s_camera.RGBView->setCamera(s_camera.Camera);
      }
      if(sConfig.RenderAux) {
         s_camera.AuxColor = filament::Texture::Builder()
            .width(sConfig.Width).height(sConfig.Height).levels(1)
            .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                   filament::Texture::Usage::SAMPLEABLE)
            .format(filament::Texture::InternalFormat::RGBA32F)
            .build(cEngine);
         s_camera.AuxDepth = filament::Texture::Builder()
            .width(sConfig.Width).height(sConfig.Height).levels(1)
            .usage(filament::Texture::Usage::DEPTH_ATTACHMENT)
            .format(filament::Texture::InternalFormat::DEPTH32F)
            .build(cEngine);
         s_camera.AuxTarget = filament::RenderTarget::Builder()
            .texture(filament::RenderTarget::AttachmentPoint::COLOR, s_camera.AuxColor)
            .texture(filament::RenderTarget::AttachmentPoint::DEPTH, s_camera.AuxDepth)
            .build(cEngine);
         s_camera.AuxView = cEngine.createView();
         s_camera.AuxView->setViewport(
            filament::Viewport(0, 0, sConfig.Width, sConfig.Height));
         s_camera.AuxView->setRenderTarget(s_camera.AuxTarget);
         s_camera.AuxView->setScene(&m_pcIdScene->GetScene());
         s_camera.AuxView->setCamera(s_camera.Camera);
         /* Raw values: no tone mapping, no AA, no shadows */
         s_camera.AuxView->setPostProcessingEnabled(false);
         s_camera.AuxView->setShadowingEnabled(false);
      }
      s_camera.Created = true;
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::ReleaseResources(SCamera& s_camera) {
      if(!s_camera.Created) {
         return;
      }
      filament::Engine& cEngine = m_pcEngine->GetEngine();
      if(s_camera.RGBView != nullptr) {
         cEngine.destroy(s_camera.RGBView);
         cEngine.destroy(s_camera.RGBTarget);
         cEngine.destroy(s_camera.RGBColor);
         cEngine.destroy(s_camera.RGBDepth);
      }
      if(s_camera.AuxView != nullptr) {
         cEngine.destroy(s_camera.AuxView);
         cEngine.destroy(s_camera.AuxTarget);
         cEngine.destroy(s_camera.AuxColor);
         cEngine.destroy(s_camera.AuxDepth);
      }
      cEngine.destroyCameraComponent(s_camera.CameraEntity);
      utils::EntityManager::get().destroy(s_camera.CameraEntity);
      s_camera.Created = false;
   }

   /****************************************/
   /****************************************/

   mat4f CPRCameraPool::ComputeViewTransform(const SPRCameraConfig& s_config) {
      const CVector3& cAnchorPosition = s_config.Anchor->Position;
      const CQuaternion& cAnchorOrientation = s_config.Anchor->Orientation;
      /* Maps the Filament camera axes (looks along -z, +y up) onto the
       * mount frame (looks along +x, +z up) */
      static const mat4f cAxisFix(mat3f(
         float3{0.0f, -1.0f, 0.0f},   /* camera x -> mount -y */
         float3{0.0f, 0.0f, 1.0f},    /* camera y -> mount +z */
         float3{-1.0f, 0.0f, 0.0f})); /* camera z -> mount -x */
      return
         mat4f::translation(float3{float(cAnchorPosition.GetX()),
                                   float(cAnchorPosition.GetY()),
                                   float(cAnchorPosition.GetZ())}) *
         mat4f(quatf(float(cAnchorOrientation.GetW()),
                     float(cAnchorOrientation.GetX()),
                     float(cAnchorOrientation.GetY()),
                     float(cAnchorOrientation.GetZ()))) *
         mat4f::translation(float3{float(s_config.PositionOffset.GetX()),
                                   float(s_config.PositionOffset.GetY()),
                                   float(s_config.PositionOffset.GetZ())}) *
         mat4f(quatf(float(s_config.OrientationOffset.GetW()),
                     float(s_config.OrientationOffset.GetX()),
                     float(s_config.OrientationOffset.GetY()),
                     float(s_config.OrientationOffset.GetZ()))) *
         cAxisFix;
   }

   /****************************************/
   /****************************************/

   std::vector<UInt32> CPRCameraPool::GetHandles() const {
      std::vector<UInt32> vecHandles;
      vecHandles.reserve(m_mapCameras.size());
      for(const auto& tCamera : m_mapCameras) {
         vecHandles.push_back(tCamera.first);
      }
      return vecHandles;
   }

   /****************************************/
   /****************************************/

   void CPRCameraPool::UpdateCameraTransform(SCamera& s_camera) {
      s_camera.Camera->setModelMatrix(ComputeViewTransform(s_camera.Config));
   }

   /****************************************/
   /****************************************/

}
