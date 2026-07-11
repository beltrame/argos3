/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_render_engine.h>
 *
 * Wraps the Filament engine, renderer and headless swapchain. All
 * Filament calls must happen on the thread that created the engine
 * (the ARGoS main thread, which runs CSpace::UpdateMedia()); this is
 * enforced with an assertion in debug builds.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_RENDER_ENGINE_H
#define PR_RENDER_ENGINE_H

#include <argos3/core/utility/datatypes/datatypes.h>

#include <string>
#include <thread>
#include <vector>

namespace filament {
   class Engine;
   class Renderer;
   class SwapChain;
   class Scene;
   class View;
   class Camera;
   class Material;
}

namespace utils {
   class Entity;
}

namespace argos {

   class CPRRenderEngine {

   public:

      CPRRenderEngine() {}
      ~CPRRenderEngine();

      /**
       * Creates the Filament engine with a headless swapchain.
       * @param str_backend "vulkan" (default) or "opengl"
       */
      void Create(const std::string& str_backend);

      void Destroy();

      inline bool IsCreated() const {
         return m_pcEngine != nullptr;
      }

      inline filament::Engine& GetEngine() {
         AssertRenderThread();
         return *m_pcEngine;
      }

      inline filament::Renderer& GetRenderer() {
         AssertRenderThread();
         return *m_pcRenderer;
      }

      inline filament::SwapChain& GetSwapChain() {
         AssertRenderThread();
         return *m_pcSwapChain;
      }

      inline filament::Scene& GetScene() {
         AssertRenderThread();
         return *m_pcScene;
      }

      /**
       * Returns the default lit material (embedded pr_lit.filamat).
       */
      inline filament::Material& GetLitMaterial() {
         AssertRenderThread();
         return *m_pcLitMaterial;
      }

      /**
       * Renders a view and synchronously reads back RGBA8 pixels.
       * The view must be sized w x h. Blocking; used by the M1 debug
       * camera. The asynchronous pipelined path is added with the
       * camera pool.
       * Rows are returned top-to-bottom.
       */
      void RenderAndReadRGBA(filament::View& c_view,
                             UInt32 un_width,
                             UInt32 un_height,
                             std::vector<UInt8>& vec_pixels);

      /**
       * Asserts that the caller runs on the thread that created the
       * Filament engine. No-op in release builds.
       */
      void AssertRenderThread() const;

   private:

      filament::Engine*    m_pcEngine      = nullptr;
      filament::Renderer*  m_pcRenderer    = nullptr;
      filament::SwapChain* m_pcSwapChain   = nullptr;
      filament::Scene*     m_pcScene       = nullptr;
      filament::Material*  m_pcLitMaterial = nullptr;
      std::thread::id      m_cRenderThreadId;

   };

}

#endif
