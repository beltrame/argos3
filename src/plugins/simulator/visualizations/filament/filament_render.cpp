/**
 * @file <argos3/plugins/simulator/visualizations/filament/filament_render.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "filament_render.h"

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/photorealism/photorealism_medium.h>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <filament/Camera.h>
#include <filament/SwapChain.h>
/* No <utils/EntityManager.h>: this plugin has its own copy of that
 * singleton and must not mint Filament entities with it. They all come
 * from CPRRenderEngine::CreateEntity(), which the photorealism plugin
 * (the owner of the engine) compiles on its side of the symbol
 * firewall */

#include <argos3/plugins/simulator/photorealism/render_core/stb_image_write.h>
#include <backend/PixelBufferDescriptor.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <vector>

namespace argos {

   /****************************************/
   /****************************************/

   void CFilamentRender::Init(TConfigurationNode& t_tree) {
      try {
         GetNodeAttributeOrDefault(t_tree, "medium", m_strMediumId, m_strMediumId);
         GetNodeAttributeOrDefault(t_tree, "title", m_strTitle, m_strTitle);
         std::string strResolution("1280,720");
         GetNodeAttributeOrDefault(t_tree, "resolution", strResolution, strResolution);
         UInt32 punResolution[2];
         ParseValues<UInt32>(strResolution, 2, punResolution, ',');
         m_unWidth = punResolution[0];
         m_unHeight = punResolution[1];
         GetNodeAttributeOrDefault(t_tree, "fov", m_fFieldOfView, m_fFieldOfView);
         GetNodeAttributeOrDefault(t_tree, "position", m_cCameraStart, m_cCameraStart);
         GetNodeAttributeOrDefault(t_tree, "look_at", m_cCameraLookAt, m_cCameraLookAt);
         GetNodeAttributeOrDefault(t_tree, "move_speed", m_fMoveSpeed, m_fMoveSpeed);
         GetNodeAttributeOrDefault(t_tree, "speed", m_fSpeed, m_fSpeed);
         GetNodeAttributeOrDefault(t_tree, "paused", m_bStartPaused, m_bStartPaused);
         GetNodeAttributeOrDefault(t_tree, "autoclose", m_unAutoCloseTicks, m_unAutoCloseTicks);
         /* One or more robot ids, comma separated: one inset per
          * window corner */
         std::string strInsetRobots;
         GetNodeAttributeOrDefault(t_tree, "inset_camera", strInsetRobots, strInsetRobots);
         std::istringstream cInsetRobots(strInsetRobots);
         std::string strRobot;
         while(std::getline(cInsetRobots, strRobot, ',')) {
            /* Trim, so "a, b" works as well as "a,b" */
            size_t unFirst = strRobot.find_first_not_of(" \t\n\r");
            if(unFirst == std::string::npos) {
               continue;
            }
            size_t unLast = strRobot.find_last_not_of(" \t\n\r");
            m_vecInsetRobots.push_back(
               strRobot.substr(unFirst, unLast - unFirst + 1));
         }
         if(m_vecInsetRobots.size() > MAX_INSETS) {
            LOGERR << "[WARNING] Filament visualization: at most "
                   << MAX_INSETS << " insets (one per window corner) are "
                   "supported; ignoring the extra ones" << std::endl;
            m_vecInsetRobots.resize(MAX_INSETS);
         }
         GetNodeAttributeOrDefault(t_tree, "inset_size", m_fInsetSize, m_fInsetSize);
         GetNodeAttributeOrDefault(t_tree, "screenshot", m_strScreenshotPrefix, m_strScreenshotPrefix);
         GetNodeAttributeOrDefault(t_tree, "screenshot_period", m_unScreenshotPeriod, m_unScreenshotPeriod);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the Filament visualization", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CFilamentRender::CreateWindow() {
      m_pcMedium = &m_cSimulator.GetMedium<CPhotorealismMedium>(m_strMediumId);
      CPRRenderEngine& cEngine = m_pcMedium->GetRenderEngine();
      /* The prebuilt Filament Vulkan backend creates X11 surfaces;
       * on Wayland desktops the window runs through XWayland */
      ::setenv("SDL_VIDEODRIVER", "x11", 1);
      if(SDL_Init(SDL_INIT_VIDEO) != 0) {
         THROW_ARGOSEXCEPTION("SDL initialization failed: " << SDL_GetError());
      }
      m_ptWindow = SDL_CreateWindow(m_strTitle.c_str(),
                                    SDL_WINDOWPOS_CENTERED,
                                    SDL_WINDOWPOS_CENTERED,
                                    int(m_unWidth), int(m_unHeight),
                                    SDL_WINDOW_RESIZABLE);
      if(m_ptWindow == nullptr) {
         THROW_ARGOSEXCEPTION("Cannot create a window: " << SDL_GetError());
      }
      SDL_SysWMinfo sWMInfo;
      SDL_VERSION(&sWMInfo.version);
      if(SDL_GetWindowWMInfo(m_ptWindow, &sWMInfo) == SDL_FALSE ||
         sWMInfo.subsystem != SDL_SYSWM_X11) {
         THROW_ARGOSEXCEPTION("Cannot obtain the X11 window handle: "
                              << SDL_GetError());
      }
      m_pcSwapChain = cEngine.GetEngine().createSwapChain(
         reinterpret_cast<void*>(sWMInfo.info.x11.window));
      m_pcRenderer = cEngine.GetEngine().createRenderer();
      /* The window camera and view. Every Filament entity this class
       * creates must be minted by the photorealism plugin, which owns
       * the engine: see CPRRenderEngine::CreateEntity() */
      m_cCameraEntity = cEngine.CreateEntity();
      m_pcCamera = cEngine.GetEngine().createCamera(m_cCameraEntity);
      m_pcCamera->setProjection(double(m_fFieldOfView),
                                double(m_unWidth) / double(m_unHeight),
                                0.05, 250.0,
                                filament::Camera::Fov::VERTICAL);
      /* The same exposure as the robot cameras, so the window shows
       * the world at the brightness the sensors see */
      cEngine.ApplyExposure(*m_pcCamera);
      m_pcView = cEngine.GetEngine().createView();
      m_pcView->setViewport(filament::Viewport(0, 0, m_unWidth, m_unHeight));
      m_pcView->setScene(&cEngine.GetScene());
      m_pcView->setCamera(m_pcCamera);
      /* Initial pose: at 'position', looking at 'look_at' */
      m_cCameraPosition = m_cCameraStart;
      CVector3 cDirection = m_cCameraLookAt - m_cCameraStart;
      m_fYaw = std::atan2(cDirection.GetY(), cDirection.GetX());
      m_fPitch = std::atan2(cDirection.GetZ(),
                            std::sqrt(cDirection.GetX() * cDirection.GetX() +
                                      cDirection.GetY() * cDirection.GetY()));
      if(!m_vecInsetRobots.empty()) {
         CreateInsets();
      }
      m_bPaused = m_bStartPaused;
      LOG << "[INFO] Filament visualization: SPACE pauses, N steps once, "
             "WASD/QE move (SHIFT: faster), left-drag looks, "
             "right-drag pans, wheel dollies, ESC quits" << std::endl;
   }

   /****************************************/
   /****************************************/

   void CFilamentRender::CreateInsets() {
      CPRCameraPool& cPool = m_pcMedium->GetCameraPool();
      filament::Engine& cEngine = m_pcMedium->GetRenderEngine().GetEngine();
      for(const std::string& str_robot : m_vecInsetRobots) {
         /* Find the first photorealistic camera mounted on the robot */
         bool bFound = false;
         SInset sInset;
         sInset.Robot = str_robot;
         for(UInt32 unHandle : cPool.GetHandles()) {
            const SPRCameraConfig& sConfig = cPool.GetConfig(unHandle);
            if(sConfig.Anchor != nullptr &&
               sConfig.Anchor->Body.GetRootEntity().GetId() == str_robot) {
               sInset.CameraHandle = unHandle;
               bFound = true;
               break;
            }
         }
         if(!bFound) {
            LOGERR << "[WARNING] Filament visualization: robot \""
                   << str_robot << "\" has no photorealistic camera; "
                   "its inset is disabled" << std::endl;
            continue;
         }
         sInset.CameraEntity = m_pcMedium->GetRenderEngine().CreateEntity();
         sInset.Camera = cEngine.createCamera(sInset.CameraEntity);
         m_pcMedium->GetRenderEngine().ApplyExposure(*sInset.Camera);
         sInset.View = cEngine.createView();
         sInset.View->setScene(&m_pcMedium->GetRenderEngine().GetScene());
         sInset.View->setCamera(sInset.Camera);
         m_vecInsets.push_back(sInset);
      }
      LayOutInsets();
   }

   /****************************************/
   /****************************************/

   void CFilamentRender::LayOutInsets() {
      const UInt32 unMargin = 12;
      /* With more than one inset the corners share the window, so an
       * inset may take at most half of it minus the margins */
      UInt32 unMaxWidth = (m_vecInsets.size() > 1) ?
         (m_unWidth - 3 * unMargin) / 2 : (m_unWidth - 2 * unMargin);
      UInt32 unMaxHeight = (m_vecInsets.size() > 2) ?
         (m_unHeight - 3 * unMargin) / 2 : (m_unHeight - 2 * unMargin);
      for(size_t i = 0; i < m_vecInsets.size(); ++i) {
         SInset& sInset = m_vecInsets[i];
         const SPRCameraConfig& sConfig =
            m_pcMedium->GetCameraPool().GetConfig(sInset.CameraHandle);
         /* Each inset keeps its own sensor's aspect ratio and field
          * of view, shrunk to fit its half of the window */
         UInt32 unHeight = UInt32(m_unHeight * m_fInsetSize);
         UInt32 unWidth =
            UInt32(Real(unHeight) * sConfig.Width / sConfig.Height);
         if(unWidth > unMaxWidth) {
            unWidth = unMaxWidth;
            unHeight =
               UInt32(Real(unWidth) * sConfig.Height / sConfig.Width);
         }
         if(unHeight > unMaxHeight) {
            unHeight = unMaxHeight;
            unWidth =
               UInt32(Real(unHeight) * sConfig.Width / sConfig.Height);
         }
         /* Corners are filled bottom-right, bottom-left, top-left,
          * top-right, so a lone inset keeps its usual place. The
          * Filament viewport origin is the bottom-left corner */
         bool bRight = (i == 0 || i == 3);
         bool bTop = (i == 2 || i == 3);
         sInset.View->setViewport(
            filament::Viewport(
               bRight ? SInt32(m_unWidth - unWidth - unMargin)
                      : SInt32(unMargin),
               bTop ? SInt32(m_unHeight - unHeight - unMargin)
                    : SInt32(unMargin),
               unWidth, unHeight));
         sInset.Camera->setProjection(
            double(sConfig.FieldOfView),
            double(unWidth) / double(unHeight),
            double(sConfig.NearPlane), double(sConfig.FarPlane),
            filament::Camera::Fov::VERTICAL);
      }
   }

   /****************************************/
   /****************************************/

   void CFilamentRender::DestroyWindow() {
      if(m_pcMedium == nullptr) {
         return;
      }
      filament::Engine& cEngine = m_pcMedium->GetRenderEngine().GetEngine();
      cEngine.flushAndWait();
      for(SInset& s_inset : m_vecInsets) {
         cEngine.destroy(s_inset.View);
         cEngine.destroyCameraComponent(s_inset.CameraEntity);
         m_pcMedium->GetRenderEngine().DestroyEntity(s_inset.CameraEntity);
      }
      m_vecInsets.clear();
      cEngine.destroy(m_pcView);
      cEngine.destroyCameraComponent(m_cCameraEntity);
      m_pcMedium->GetRenderEngine().DestroyEntity(m_cCameraEntity);
      cEngine.destroy(m_pcRenderer);
      cEngine.destroy(m_pcSwapChain);
      m_pcView = nullptr;
      m_pcCamera = nullptr;
      m_pcRenderer = nullptr;
      m_pcSwapChain = nullptr;
      SDL_DestroyWindow(m_ptWindow);
      m_ptWindow = nullptr;
      SDL_QuitSubSystem(SDL_INIT_VIDEO);
   }

   /****************************************/
   /****************************************/

   bool CFilamentRender::HandleEvents(Real f_frame_seconds) {
      SDL_Event tEvent;
      while(SDL_PollEvent(&tEvent)) {
         switch(tEvent.type) {
            case SDL_QUIT:
               return false;
            case SDL_KEYDOWN:
               switch(tEvent.key.keysym.sym) {
                  case SDLK_ESCAPE:
                     return false;
                  case SDLK_SPACE:
                     m_bPaused = !m_bPaused;
                     break;
                  case SDLK_n:
                     m_bSingleStep = true;
                     break;
                  default:
                     break;
               }
               break;
            case SDL_MOUSEBUTTONDOWN:
               if(tEvent.button.button == SDL_BUTTON_LEFT) {
                  m_bDragging = true;
               }
               else if(tEvent.button.button == SDL_BUTTON_RIGHT ||
                       tEvent.button.button == SDL_BUTTON_MIDDLE) {
                  m_bPanning = true;
               }
               break;
            case SDL_MOUSEBUTTONUP:
               if(tEvent.button.button == SDL_BUTTON_LEFT) {
                  m_bDragging = false;
               }
               else if(tEvent.button.button == SDL_BUTTON_RIGHT ||
                       tEvent.button.button == SDL_BUTTON_MIDDLE) {
                  m_bPanning = false;
               }
               break;
            case SDL_MOUSEMOTION:
               if(m_bDragging) {
                  m_fYaw -= tEvent.motion.xrel * 0.005;
                  m_fPitch -= tEvent.motion.yrel * 0.005;
                  m_fPitch = std::min(1.5, std::max(-1.5, m_fPitch));
               }
               else if(m_bPanning) {
                  /* Drag the world with the cursor: move in the view
                   * plane, scaled so it feels constant on screen */
                  Real fScale = 0.0025 * m_fMoveSpeed;
                  CVector3 cRight(std::sin(m_fYaw), -std::cos(m_fYaw), 0.0);
                  CVector3 cUp(-std::cos(m_fYaw) * std::sin(m_fPitch),
                               -std::sin(m_fYaw) * std::sin(m_fPitch),
                               std::cos(m_fPitch));
                  m_cCameraPosition -= cRight * (tEvent.motion.xrel * fScale);
                  m_cCameraPosition += cUp * (tEvent.motion.yrel * fScale);
               }
               break;
            case SDL_MOUSEWHEEL: {
               /* Dolly along the view direction */
               CVector3 cForward(std::cos(m_fYaw) * std::cos(m_fPitch),
                                 std::sin(m_fYaw) * std::cos(m_fPitch),
                                 std::sin(m_fPitch));
               m_cCameraPosition +=
                  cForward * (tEvent.wheel.y * 0.15 * m_fMoveSpeed);
               break;
            }
            case SDL_WINDOWEVENT:
               if(tEvent.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                  int nWidth, nHeight;
                  SDL_GetWindowSize(m_ptWindow, &nWidth, &nHeight);
                  m_unWidth = UInt32(nWidth);
                  m_unHeight = UInt32(nHeight);
                  m_pcView->setViewport(
                     filament::Viewport(0, 0, m_unWidth, m_unHeight));
                  m_pcCamera->setProjection(
                     double(m_fFieldOfView),
                     double(m_unWidth) / double(m_unHeight),
                     0.05, 250.0,
                     filament::Camera::Fov::VERTICAL);
                  LayOutInsets();
               }
               break;
            default:
               break;
         }
      }
      /* Continuous movement from held keys */
      const Uint8* punKeys = SDL_GetKeyboardState(nullptr);
      Real fStep = m_fMoveSpeed * f_frame_seconds *
         (punKeys[SDL_SCANCODE_LSHIFT] != 0 ? 5.0 : 1.0);
      CVector3 cForward(std::cos(m_fYaw) * std::cos(m_fPitch),
                        std::sin(m_fYaw) * std::cos(m_fPitch),
                        std::sin(m_fPitch));
      CVector3 cRight(std::sin(m_fYaw), -std::cos(m_fYaw), 0.0);
      if(punKeys[SDL_SCANCODE_W]) m_cCameraPosition += cForward * fStep;
      if(punKeys[SDL_SCANCODE_S]) m_cCameraPosition -= cForward * fStep;
      if(punKeys[SDL_SCANCODE_D]) m_cCameraPosition += cRight * fStep;
      if(punKeys[SDL_SCANCODE_A]) m_cCameraPosition -= cRight * fStep;
      if(punKeys[SDL_SCANCODE_E]) m_cCameraPosition += CVector3::Z * fStep;
      if(punKeys[SDL_SCANCODE_Q]) m_cCameraPosition -= CVector3::Z * fStep;
      return true;
   }

   /****************************************/
   /****************************************/

   void CFilamentRender::UpdateCamera() {
      CVector3 cForward(std::cos(m_fYaw) * std::cos(m_fPitch),
                        std::sin(m_fYaw) * std::cos(m_fPitch),
                        std::sin(m_fPitch));
      CVector3 cCenter = m_cCameraPosition + cForward;
      m_pcCamera->lookAt({float(m_cCameraPosition.GetX()),
                          float(m_cCameraPosition.GetY()),
                          float(m_cCameraPosition.GetZ())},
                         {float(cCenter.GetX()),
                          float(cCenter.GetY()),
                          float(cCenter.GetZ())},
                         {0.0f, 0.0f, 1.0f});
   }

   /****************************************/
   /****************************************/

   void CFilamentRender::RenderFrame() {
      if(!m_pcRenderer->beginFrame(m_pcSwapChain)) {
         return;
      }
      m_pcRenderer->render(m_pcView);
      for(SInset& s_inset : m_vecInsets) {
         /* Live view from the robot camera's pose */
         s_inset.Camera->setModelMatrix(
            CPRCameraPool::ComputeViewTransform(
               m_pcMedium->GetCameraPool().GetConfig(s_inset.CameraHandle)));
         m_pcRenderer->render(s_inset.View);
      }
      /* Periodic window screenshots */
      bool bShotDone = false;
      std::vector<UInt8> vecPixels;
      UInt32 unClock = m_cSpace.GetSimulationClock();
      bool bShotDue =
         !m_strScreenshotPrefix.empty() &&
         unClock >= m_unLastScreenshotTick + m_unScreenshotPeriod;
      if(bShotDue) {
         m_unLastScreenshotTick = unClock;
         vecPixels.resize(size_t(m_unWidth) * m_unHeight * 4);
         filament::backend::PixelBufferDescriptor cDescriptor(
            vecPixels.data(), vecPixels.size(),
            filament::backend::PixelDataFormat::RGBA,
            filament::backend::PixelDataType::UBYTE,
            [](void*, size_t, void* pt_user) {
               *static_cast<bool*>(pt_user) = true;
            },
            &bShotDone);
         m_pcRenderer->readPixels(0, 0, m_unWidth, m_unHeight,
                                  std::move(cDescriptor));
      }
      m_pcRenderer->endFrame();
      if(bShotDue) {
         m_pcMedium->GetRenderEngine().GetEngine().flushAndWait();
         if(bShotDone) {
            char pchName[256];
            ::snprintf(pchName, sizeof(pchName), "%s_%010u.png",
                       m_strScreenshotPrefix.c_str(), unClock);
            /* Vulkan readPixels rows are already top-to-bottom */
            size_t unStride = size_t(m_unWidth) * 4;
            if(stbi_write_png(pchName, int(m_unWidth), int(m_unHeight), 4,
                              vecPixels.data(), int(unStride)) == 0) {
               LOGERR << "[WARNING] Cannot write screenshot \""
                      << pchName << "\"" << std::endl;
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   void CFilamentRender::Execute() {
      CreateWindow();
      using TClock = std::chrono::steady_clock;
      Real fTickSeconds = CPhysicsEngine::GetSimulationClockTick();
      TClock::time_point tLastFrame = TClock::now();
      Real fStepDebt = 0.0;
      bool bPostExperimentDone = false;
      bool bRunning = true;
      while(bRunning) {
         TClock::time_point tNow = TClock::now();
         Real fFrameSeconds =
            std::chrono::duration<double>(tNow - tLastFrame).count();
         tLastFrame = tNow;
         bRunning = HandleEvents(fFrameSeconds);
         /*
          * Step the simulation
          */
         if(!m_cSimulator.IsExperimentFinished()) {
            if(m_bSingleStep) {
               m_cSimulator.UpdateSpace();
               m_bSingleStep = false;
               fStepDebt = 0.0;
            }
            else if(!m_bPaused) {
               if(m_fSpeed > 0.0) {
                  /* Real-time pacing: accumulate wall time, step when
                   * a tick's worth has passed (at most a few per
                   * frame, so the UI stays responsive) */
                  fStepDebt += fFrameSeconds * m_fSpeed;
                  UInt32 unSteps = 0;
                  while(fStepDebt >= fTickSeconds && unSteps < 10 &&
                        !m_cSimulator.IsExperimentFinished()) {
                     m_cSimulator.UpdateSpace();
                     fStepDebt -= fTickSeconds;
                     ++unSteps;
                  }
                  if(unSteps == 10) {
                     fStepDebt = 0.0;
                  }
               }
               else {
                  /*

                   * As fast as possible: one step per rendered frame */
                  m_cSimulator.UpdateSpace();
               }
            }
         }
         else if(!bPostExperimentDone) {
            m_cSimulator.GetLoopFunctions().PostExperiment();
            LOG.Flush();
            LOGERR.Flush();
            /* PostExperiment may have reset the simulation to start a
             * new run; otherwise the window stays open on the final
             * state until the user closes it */
            bPostExperimentDone = m_cSimulator.IsExperimentFinished();
         }
         UpdateCamera();
         RenderFrame();
         if(m_unAutoCloseTicks > 0 &&
            (m_cSpace.GetSimulationClock() >= m_unAutoCloseTicks ||
             m_cSimulator.IsExperimentFinished())) {
            bRunning = false;
         }
      }
      DestroyWindow();
   }

   /****************************************/
   /****************************************/

   REGISTER_VISUALIZATION(CFilamentRender,
                          "filament",
                          "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                          "1.0",
                          "An interactive window on the photorealistic scene.",
                          "This visualization opens a window showing the scene maintained by the\n"
                          "<photorealism> medium, which must be present in the <media> section. The\n"
                          "experiment behaves exactly as it does headless: the medium owns the\n"
                          "renderer, the scene, and the robot cameras; this window only looks at it.\n\n"

                          "REQUIRED XML CONFIGURATION\n\n"
                          "  <visualization>\n"
                          "    <filament />\n"
                          "  </visualization>\n\n"

                          "OPTIONAL XML CONFIGURATION\n\n"
                          "  <visualization>\n"
                          "    <filament medium=\"pr\" resolution=\"1280,720\" fov=\"60\"\n"
                          "              position=\"2.5,2.5,2\" look_at=\"0,0,0.25\"\n"
                          "              speed=\"1\" paused=\"false\" move_speed=\"2\" />\n"
                          "  </visualization>\n\n"

                          "'medium' is the id of the photorealism medium to display. 'speed' is the\n"
                          "simulation speed factor (1 = real time, 2 = twice as fast, 0 = as fast\n"
                          "as possible). 'position'/'look_at' set the initial camera pose.\n\n"

                          "With inset_camera=\"<robot id>\" the window shows what that robot's\n"
                          "photorealistic camera sees as an inset in the bottom-right corner,\n"
                          "rendered live at the window frame rate from the sensor's pose, field\n"
                          "of view, and aspect ratio. 'inset_size' (default 0.3) is the inset\n"
                          "height as a fraction of the window height.\n\n"

                          "Up to four robot ids can be given, comma separated, to show one inset\n"
                          "per window corner: they fill the corners in the order bottom-right,\n"
                          "bottom-left, top-left, top-right, so a lone inset keeps its usual\n"
                          "place. Each inset keeps its own sensor's aspect ratio and field of\n"
                          "view, shrunk to fit its half of the window when it has neighbours:\n\n"
                          "  <filament medium=\"pr\" inset_camera=\"drone0,drone1,drone2,drone3\"\n"
                          "            inset_size=\"0.3\" />\n\n"

                          "INTERACTION\n\n"
                          "SPACE pauses and resumes; N steps one tick. W/A/S/D fly forward/left/\n"
                          "back/right, Q/E down/up (SHIFT accelerates). Dragging with the left\n"
                          "mouse button looks around, dragging with the right or middle button\n"
                          "pans in the view plane, and the scroll wheel dollies forward and\n"
                          "backward. ESC or closing the window ends the run.\n\n"

                          "The window is an X11 window (through XWayland on Wayland desktops),\n"
                          "matching the surface types supported by the prebuilt Filament Vulkan\n"
                          "backend.",
                          "Usable");

   /****************************************/
   /****************************************/

}
