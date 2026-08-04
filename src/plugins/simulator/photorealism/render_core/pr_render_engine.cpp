/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_render_engine.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_render_engine.h"

#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/SwapChain.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Camera.h>
#include <filament/Exposure.h>
#include <filament/Material.h>
#include <backend/PixelBufferDescriptor.h>
#include <utils/Entity.h>
#include <utils/EntityManager.h>

#include <cstring>

namespace argos {

   /* Generated from materials/pr_lit.mat at build time */
   extern const unsigned char PR_LIT_FILAMAT[];
   extern const size_t PR_LIT_FILAMAT_SIZE;

   /****************************************/
   /****************************************/

   CPRRenderEngine::~CPRRenderEngine() {
      Destroy();
   }

   /****************************************/
   /****************************************/

   void CPRRenderEngine::Create(const std::string& str_backend) {
      if(m_pcEngine != nullptr) {
         THROW_ARGOSEXCEPTION("CPRRenderEngine::Create() called twice");
      }
      filament::Engine::Backend eBackend;
      if(str_backend == "vulkan") {
         eBackend = filament::Engine::Backend::VULKAN;
      }
      else if(str_backend == "opengl") {
         eBackend = filament::Engine::Backend::OPENGL;
      }
      else {
         THROW_ARGOSEXCEPTION("Unknown render backend \"" << str_backend << "\"; use \"vulkan\" or \"opengl\"");
      }
      m_cRenderThreadId = std::this_thread::get_id();
      /* The driver handle arena must fit the render targets and
       * textures of many cameras (50+ robots); the default runs out
       * and falls back to a slow heap path with a warning */
      filament::Engine::Config sConfig;
      sConfig.driverHandleArenaSizeMB = 16;
      m_pcEngine = filament::Engine::Builder()
         .backend(eBackend)
         .config(&sConfig)
         .build();
      if(m_pcEngine == nullptr) {
         THROW_ARGOSEXCEPTION("Filament engine creation failed for backend \"" << str_backend << "\"");
      }
      /* Headless swapchain: rendering happens into offscreen render
       * targets, so the swapchain size is irrelevant; 16x16 keeps it
       * cheap. */
      m_pcSwapChain = m_pcEngine->createSwapChain(16u, 16u);
      m_pcRenderer = m_pcEngine->createRenderer();
      m_pcScene = m_pcEngine->createScene();
      /* Load the embedded default lit material */
      m_pcLitMaterial = filament::Material::Builder()
         .package(PR_LIT_FILAMAT, PR_LIT_FILAMAT_SIZE)
         .build(*m_pcEngine);
      if(m_pcLitMaterial == nullptr) {
         THROW_ARGOSEXCEPTION("Failed to load the embedded pr_lit material (backend \"" << str_backend << "\")");
      }
      LOG << "[INFO] Photorealism render engine created (backend: "
          << str_backend << ", headless)" << std::endl;
   }

   /****************************************/
   /****************************************/

   void CPRRenderEngine::Destroy() {
      if(m_pcEngine != nullptr) {
         AssertRenderThread();
         m_pcEngine->destroy(m_pcLitMaterial);
         m_pcEngine->destroy(m_pcScene);
         m_pcEngine->destroy(m_pcRenderer);
         m_pcEngine->destroy(m_pcSwapChain);
         filament::Engine::destroy(&m_pcEngine);
         m_pcEngine = nullptr;
         m_pcRenderer = nullptr;
         m_pcSwapChain = nullptr;
         m_pcScene = nullptr;
         m_pcLitMaterial = nullptr;
      }
   }

   /****************************************/
   /****************************************/

   Real CPRRenderEngine::GetExposureScale() const {
      return Real(filament::Exposure::exposure(
                     float(m_sExposure.Aperture),
                     float(m_sExposure.ShutterSpeed),
                     float(m_sExposure.Sensitivity)));
   }

   /****************************************/
   /****************************************/

   void CPRRenderEngine::ApplyExposure(filament::Camera& c_camera) const {
      c_camera.setExposure(float(m_sExposure.Aperture),
                           float(m_sExposure.ShutterSpeed),
                           float(m_sExposure.Sensitivity));
   }

   /****************************************/
   /****************************************/

   void CPRRenderEngine::RenderAndReadRGBA(filament::View& c_view,
                                           UInt32 un_width,
                                           UInt32 un_height,
                                           std::vector<UInt8>& vec_pixels) {
      AssertRenderThread();
      vec_pixels.resize(size_t(un_width) * un_height * 4);
      bool bDone = false;
      /* The false return of beginFrame() is only a frame-skipping
       * hint; this readback must happen */
      m_pcRenderer->beginFrame(m_pcSwapChain);
      m_pcRenderer->render(&c_view);
      filament::backend::PixelBufferDescriptor cDescriptor(
         vec_pixels.data(), vec_pixels.size(),
         filament::backend::PixelDataFormat::RGBA,
         filament::backend::PixelDataType::UBYTE,
         [](void*, size_t, void* pt_user) {
            *static_cast<bool*>(pt_user) = true;
         },
         &bDone);
      m_pcRenderer->readPixels(c_view.getRenderTarget(),
                               0, 0, un_width, un_height,
                               std::move(cDescriptor));
      m_pcRenderer->endFrame();
      m_pcEngine->flushAndWait();
      if(!bDone) {
         THROW_ARGOSEXCEPTION("Filament readPixels did not complete");
      }
   }

   /****************************************/
   /****************************************/

   utils::Entity CPRRenderEngine::CreateEntity() {
      AssertRenderThread();
      return utils::EntityManager::get().create();
   }

   /****************************************/
   /****************************************/

   void CPRRenderEngine::DestroyEntity(utils::Entity c_entity) {
      AssertRenderThread();
      utils::EntityManager::get().destroy(c_entity);
   }

   /****************************************/
   /****************************************/

   const void* CPRRenderEngine::GetEntityManagerAddress() const {
      return static_cast<const void*>(&utils::EntityManager::get());
   }

   /****************************************/
   /****************************************/

   void CPRRenderEngine::AssertRenderThread() const {
#ifndef NDEBUG
      if(std::this_thread::get_id() != m_cRenderThreadId) {
         THROW_ARGOSEXCEPTION("Filament API called from a thread other than the one that created the engine");
      }
#endif
   }

   /****************************************/
   /****************************************/

}
