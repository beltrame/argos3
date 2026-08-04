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

   /**
    * Photographic exposure shared by every camera that renders this
    * scene: the robot sensors, the debug camera and the interactive
    * viewer. Filament is a physically based renderer, so the image is
    * only as bright as the exposure lets it be: leaving the default
    * sunny-16 settings on a scene lit by street lamps produces a black
    * frame no matter how many lamps are added.
    *
    * The defaults are the "sunny 16" rule (f/16 at 1/100 s, ISO 100),
    * correct for the 100 klux default sun.
    */
   struct SPRExposure {
      Real Aperture = 16.0;          /* f-number */
      Real ShutterSpeed = 1.0 / 125.0; /* seconds */
      Real Sensitivity = 100.0;      /* ISO */
   };

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
       * The exposure every camera on this scene is set up with. The
       * medium reads it from the <exposure> node before any camera
       * exists; cameras created later pick it up through
       * ApplyExposure().
       */
      inline const SPRExposure& GetExposure() const {
         return m_sExposure;
      }

      inline void SetExposure(const SPRExposure& s_exposure) {
         m_sExposure = s_exposure;
      }

      /**
       * The factor scene luminance is multiplied by before tone
       * mapping, at the current exposure. Emissive materials are
       * authored in absolute nits, so anything that has to land at a
       * fixed brightness on screen whatever the exposure (an LED, a
       * marker) must be scaled by the inverse of this.
       */
      Real GetExposureScale() const;

      /**
       * Applies the scene exposure to a camera. Every camera rendering
       * this scene must go through here: two cameras with different
       * exposures show the same world at different brightness, which
       * looks like a lighting bug and is very hard to track down.
       */
      void ApplyExposure(filament::Camera& c_camera) const;

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
       * Allocates a Filament entity.
       *
       * Filament is statically linked, with hidden visibility, into
       * every plugin that uses it (the symbol firewall that keeps its
       * libc++ out of the ARGoS runtime), so each plugin gets its own
       * copy of Filament's utils::EntityManager singleton. The
       * managers number their entities independently, but they all
       * index the *same* component managers inside the one shared
       * Engine: entities minted on one side of the firewall therefore
       * collide with entities minted on the other, and the components
       * of one silently overwrite the components of the other.
       *
       * All entities must consequently come from a single manager.
       * These two calls are compiled into the photorealism plugin,
       * which owns the engine, so plugins that render into its scene
       * (the Filament visualization) must allocate their entities
       * through them rather than calling utils::EntityManager
       * themselves.
       */
      utils::Entity CreateEntity();
      void DestroyEntity(utils::Entity c_entity);

      /**
       * The address of the utils::EntityManager singleton as seen by
       * the photorealism plugin. Only useful to check the firewall
       * has not been breached; see CreateEntity().
       */
      const void* GetEntityManagerAddress() const;

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
      SPRExposure          m_sExposure;
      std::thread::id      m_cRenderThreadId;

   };

}

#endif
