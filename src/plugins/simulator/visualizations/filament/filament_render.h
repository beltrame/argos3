/**
 * @file <argos3/plugins/simulator/visualizations/filament/filament_render.h>
 *
 * An interactive window on the photorealistic scene maintained by the
 * <photorealism> medium. The same experiment runs unchanged with or
 * without this visualization: the medium owns the renderer and the
 * scene; this class only adds a windowed swapchain, a free-fly
 * camera, and simulation stepping controls.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef FILAMENT_RENDER_H
#define FILAMENT_RENDER_H

namespace argos {
   class CFilamentRender;
   class CPhotorealismMedium;
}

namespace filament {
   class View;
   class Camera;
   class SwapChain;
   class Renderer;
}

struct SDL_Window;

#include <argos3/core/simulator/visualization/visualization.h>
#include <argos3/core/utility/math/vector3.h>

#include <utils/Entity.h>

#include <string>

namespace argos {

   class CFilamentRender : public CVisualization {

   public:

      CFilamentRender() {}
      virtual ~CFilamentRender() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset() {}
      virtual void Destroy() {}
      virtual void Execute();

   private:

      void CreateWindow();
      void DestroyWindow();
      /** Returns false when the user asked to quit */
      bool HandleEvents(Real f_frame_seconds);
      void UpdateCamera();
      void RenderFrame();

   private:

      /* Configuration */
      std::string m_strMediumId = "pr";
      std::string m_strTitle = "ARGoS (Filament)";
      UInt32 m_unWidth = 1280;
      UInt32 m_unHeight = 720;
      Real m_fFieldOfView = 60.0; /* vertical, degrees */
      CVector3 m_cCameraStart = CVector3(2.5, 2.5, 2.0);
      CVector3 m_cCameraLookAt = CVector3(0.0, 0.0, 0.25);
      Real m_fMoveSpeed = 2.0; /* m/s */
      /* Simulation speed factor (1 = real time, 0 = as fast as
       * possible) */
      Real m_fSpeed = 1.0;
      bool m_bStartPaused = false;
      /* Close automatically after this many ticks (0 = never); used
       * by the automated tests */
      UInt32 m_unAutoCloseTicks = 0;

      /* Runtime state */
      CPhotorealismMedium* m_pcMedium = nullptr;
      SDL_Window* m_ptWindow = nullptr;
      /* The window has its own renderer so that its vsync pacing and
       * frame skipping never interfere with the sensor pipeline */
      filament::Renderer* m_pcRenderer = nullptr;
      filament::SwapChain* m_pcSwapChain = nullptr;
      filament::View* m_pcView = nullptr;
      filament::Camera* m_pcCamera = nullptr;
      utils::Entity m_cCameraEntity;
      /* Free-fly camera pose */
      CVector3 m_cCameraPosition;
      Real m_fYaw = 0.0;
      Real m_fPitch = 0.0;
      /* Held keys / buttons */
      bool m_bDragging = false;
      bool m_bPaused = false;
      bool m_bSingleStep = false;

   };

}

#endif
