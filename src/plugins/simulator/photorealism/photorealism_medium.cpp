/**
 * @file <argos3/plugins/simulator/photorealism/photorealism_medium.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "photorealism_medium.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <filament/Camera.h>
#include <filament/Texture.h>
#include <filament/RenderTarget.h>
#include <filament/Skybox.h>
#include <filament/TransformManager.h>
#include <gltfio/FilamentInstance.h>
#include <utils/EntityManager.h>
#include <math/mat4.h>
#include <math/quat.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <argos3/plugins/simulator/photorealism/render_core/stb_image_write.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace argos {

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         GetNodeAttributeOrDefault(t_tree, "backend", m_strBackend, m_strBackend);
         std::string strLatency("pipelined");
         GetNodeAttributeOrDefault(t_tree, "latency", strLatency, strLatency);
         if(strLatency == "immediate") {
            m_bImmediate = true;
         }
         else if(strLatency != "pipelined") {
            THROW_ARGOSEXCEPTION("Unknown latency mode \"" << strLatency
                                 << "\"; use \"pipelined\" or \"immediate\"");
         }
         GetNodeAttributeOrDefault(t_tree, "stats", m_bStats, m_bStats);
         GetNodeAttributeOrDefault(t_tree, "asset_path", m_strAssetPath, m_strAssetPath);
         /* Comma-separated entity id prefixes to exclude from rendering
          * (all modalities). Used for collision proxies that stand in,
          * physics-wise, for scenery already rendered as a glTF prop. */
         GetNodeAttributeOrDefault(t_tree, "draw_floor", m_bDrawFloor, m_bDrawFloor);
         std::string strHidePrefixes;
         GetNodeAttributeOrDefault(t_tree, "hide_prefix", strHidePrefixes, strHidePrefixes);
         if(!strHidePrefixes.empty()) {
            std::istringstream cPrefixes(strHidePrefixes);
            std::string strPrefix;
            while(std::getline(cPrefixes, strPrefix, ',')) {
               if(!strPrefix.empty()) {
                  m_vecHiddenIdPrefixes.push_back(strPrefix);
               }
            }
         }
         if(NodeExists(t_tree, "randomization")) {
            m_cRandomizer.Init(GetNode(t_tree, "randomization"));
         }
         m_pcRNG = CRandom::CreateRNG("argos");
         if(NodeExists(t_tree, "sun")) {
            TConfigurationNode& tSun = GetNode(t_tree, "sun");
            GetNodeAttributeOrDefault(tSun, "direction", m_sSunlight.Direction, m_sSunlight.Direction);
            GetNodeAttributeOrDefault(tSun, "intensity", m_sSunlight.Intensity, m_sSunlight.Intensity);
            GetNodeAttributeOrDefault(tSun, "cast_shadows", m_sSunlight.CastShadows, m_sSunlight.CastShadows);
         }
         if(NodeExists(t_tree, "skybox")) {
            TConfigurationNode& tSkybox = GetNode(t_tree, "skybox");
            GetNodeAttributeOrDefault(tSkybox, "color", m_cSkyColor, m_cSkyColor);
         }
         /* Exposure. Filament is physically based: dimming the sun
          * without opening the camera up gives a black image, so a
          * scene lit by lamps needs both. */
         if(NodeExists(t_tree, "exposure")) {
            TConfigurationNode& tExposure = GetNode(t_tree, "exposure");
            GetNodeAttributeOrDefault(tExposure, "aperture",
                                      m_sExposure.Aperture,
                                      m_sExposure.Aperture);
            GetNodeAttributeOrDefault(tExposure, "shutter_speed",
                                      m_sExposure.ShutterSpeed,
                                      m_sExposure.ShutterSpeed);
            GetNodeAttributeOrDefault(tExposure, "sensitivity",
                                      m_sExposure.Sensitivity,
                                      m_sExposure.Sensitivity);
            if(m_sExposure.Aperture <= 0.0 ||
               m_sExposure.ShutterSpeed <= 0.0 ||
               m_sExposure.Sensitivity <= 0.0) {
               THROW_ARGOSEXCEPTION("Exposure settings must be positive "
                                    "(aperture is an f-number, shutter_speed "
                                    "is in seconds, sensitivity is an ISO)");
            }
         }
         if(NodeExists(t_tree, "lights")) {
            TConfigurationNode& tLights = GetNode(t_tree, "lights");
            for(const std::string& str_kind : {"point", "spot"}) {
               TConfigurationNodeIterator tLightIterator(str_kind);
               for(tLightIterator = tLightIterator.begin(&tLights);
                   tLightIterator != tLightIterator.end();
                   ++tLightIterator) {
                  CPRSceneSync::SLight sLight;
                  sLight.Spot = (str_kind == "spot");
                  GetNodeAttribute(*tLightIterator, "position", sLight.Position);
                  GetNodeAttributeOrDefault(*tLightIterator, "direction",
                                            sLight.Direction, sLight.Direction);
                  GetNodeAttributeOrDefault(*tLightIterator, "color",
                                            sLight.Color, sLight.Color);
                  GetNodeAttributeOrDefault(*tLightIterator, "intensity",
                                            sLight.Intensity, sLight.Intensity);
                  GetNodeAttributeOrDefault(*tLightIterator, "falloff",
                                            sLight.FalloffRadius,
                                            sLight.FalloffRadius);
                  GetNodeAttributeOrDefault(*tLightIterator, "inner_angle",
                                            sLight.InnerAngle, sLight.InnerAngle);
                  GetNodeAttributeOrDefault(*tLightIterator, "outer_angle",
                                            sLight.OuterAngle, sLight.OuterAngle);
                  GetNodeAttributeOrDefault(*tLightIterator, "cast_shadows",
                                            sLight.CastShadows,
                                            sLight.CastShadows);
                  m_vecLights.push_back(sLight);
               }
            }
         }
         if(NodeExists(t_tree, "environment")) {
            TConfigurationNode& tEnvironment = GetNode(t_tree, "environment");
            m_sEnvironment.Enabled = true;
            GetNodeAttribute(tEnvironment, "ibl", m_sEnvironment.Ibl);
            GetNodeAttributeOrDefault(tEnvironment, "skybox",
                                      m_sEnvironment.Skybox,
                                      m_sEnvironment.Skybox);
            GetNodeAttributeOrDefault(tEnvironment, "intensity",
                                      m_sEnvironment.Intensity,
                                      m_sEnvironment.Intensity);
         }
         if(NodeExists(t_tree, "scenery")) {
            TConfigurationNodeIterator tPropIterator("prop");
            for(tPropIterator = tPropIterator.begin(&GetNode(t_tree, "scenery"));
                tPropIterator != tPropIterator.end();
                ++tPropIterator) {
               SProp sProp;
               GetNodeAttribute(*tPropIterator, "model", sProp.Model);
               GetNodeAttributeOrDefault(*tPropIterator, "position",
                                         sProp.Position, sProp.Position);
               GetNodeAttributeOrDefault(*tPropIterator, "orientation",
                                         sProp.OrientationEuler,
                                         sProp.OrientationEuler);
               GetNodeAttributeOrDefault(*tPropIterator, "scale",
                                         sProp.Scale, sProp.Scale);
               m_vecProps.push_back(sProp);
            }
         }
         if(NodeExists(t_tree, "debug_camera")) {
            TConfigurationNode& tCamera = GetNode(t_tree, "debug_camera");
            m_sDebugCamera.Enabled = true;
            GetNodeAttributeOrDefault(tCamera, "position", m_sDebugCamera.Position, m_sDebugCamera.Position);
            GetNodeAttributeOrDefault(tCamera, "look_at", m_sDebugCamera.LookAt, m_sDebugCamera.LookAt);
            GetNodeAttributeOrDefault(tCamera, "fov", m_sDebugCamera.FieldOfView, m_sDebugCamera.FieldOfView);
            std::string strResolution("640,480");
            GetNodeAttributeOrDefault(tCamera, "resolution", strResolution, strResolution);
            UInt32 punResolution[2];
            ParseValues<UInt32>(strResolution, 2, punResolution, ',');
            m_sDebugCamera.Width = punResolution[0];
            m_sDebugCamera.Height = punResolution[1];
            GetNodeAttributeOrDefault(tCamera, "period", m_sDebugCamera.Period, m_sDebugCamera.Period);
            GetNodeAttributeOrDefault(tCamera, "dump", m_sDebugCamera.Directory, m_sDebugCamera.Directory);
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the photorealism medium \"" << GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::PostSpaceInit() {
      try {
         m_cEngine.Create(m_strBackend);
         /* Before any camera is created: the pool and the viewer read
          * the exposure off the engine when they build theirs */
         m_cEngine.SetExposure(m_sExposure);
         m_cDebugDraw.Init(m_cEngine);
         /* Constant-color sky, unless an HDR environment provides
          * the skybox */
         if(!m_sEnvironment.Enabled || m_sEnvironment.Skybox.empty()) {
            m_pcSkybox = filament::Skybox::Builder()
               .color({float(m_cSkyColor.GetX()),
                       float(m_cSkyColor.GetY()),
                       float(m_cSkyColor.GetZ()),
                       1.0f})
               .build(m_cEngine.GetEngine());
            m_cEngine.GetScene().setSkybox(m_pcSkybox);
         }
         CSpace& cSpace = CSimulator::GetInstance().GetSpace();
         m_cIdScene.Init(m_cEngine);
         m_cAssetRegistry.Init(m_cEngine, m_cIdScene, m_strAssetPath);
         m_cSceneSync.Init(m_cEngine, m_cIdScene, m_cAssetRegistry,
                           m_sSunlight,
                           cSpace.GetArenaSize(),
                           cSpace.GetArenaCenter());
         m_cSceneSync.SetHiddenIdPrefixes(m_vecHiddenIdPrefixes);
         m_cSceneSync.SetDrawFloor(m_bDrawFloor);
         if(m_sEnvironment.Enabled) {
            m_cSceneSync.LoadEnvironment(m_sEnvironment.Ibl,
                                         m_sEnvironment.Skybox,
                                         m_sEnvironment.Intensity);
         }
         for(const CPRSceneSync::SLight& s_light : m_vecLights) {
            m_cSceneSync.AddLight(s_light);
         }
         /* Static scenery props */
         filament::TransformManager& cTransforms =
            m_cEngine.GetEngine().getTransformManager();
         for(SProp& s_prop : m_vecProps) {
            /* Scenery needs a real identity like any other renderable:
             * the camera sensor reads entity id 0 as "no geometry" and
             * substitutes the far plane, so a prop registered with id 0
             * is rendered into the aux buffer and then discarded,
             * leaving depth and segmentation blank wherever the scenery
             * is visible. Ids come from the scene sync allocator so
             * they cannot collide with the ones given to entities. */
            s_prop.EntityId = m_cSceneSync.AllocateEntityId();
            s_prop.Instance =
               m_cAssetRegistry.CreateModelInstance(s_prop.Model,
                                                    s_prop.EntityId,
                                                    UInt8(EPRClass::Scenery));
            if(s_prop.Instance.Main == nullptr) {
               THROW_ARGOSEXCEPTION("Cannot load scenery model \""
                                    << s_prop.Model << "\"");
            }
            CQuaternion cOrientation;
            cOrientation.FromEulerAngles(
               ToRadians(CDegrees(s_prop.OrientationEuler.GetX())),
               ToRadians(CDegrees(s_prop.OrientationEuler.GetY())),
               ToRadians(CDegrees(s_prop.OrientationEuler.GetZ())));
            cTransforms.setTransform(
               cTransforms.getInstance(s_prop.Instance.Main->getRoot()),
               filament::math::mat4f::translation(
                  filament::math::float3{float(s_prop.Position.GetX()),
                                         float(s_prop.Position.GetY()),
                                         float(s_prop.Position.GetZ())}) *
               filament::math::mat4f(
                  filament::math::quatf(float(cOrientation.GetW()),
                                        float(cOrientation.GetX()),
                                        float(cOrientation.GetY()),
                                        float(cOrientation.GetZ()))) *
               filament::math::mat4f::scaling(
                  filament::math::float3{float(s_prop.Scale)}));
         }
         m_cCameraPool.Init(m_cEngine, m_cIdScene, m_bImmediate);
         if(m_sDebugCamera.Enabled) {
            CreateDebugCamera();
            std::filesystem::create_directories(m_sDebugCamera.Directory);
         }
         /* Initial sync so the scene is complete before the first tick */
         m_cSceneSync.Sync(cSpace);
         /* Draw the initial random environment */
         if(m_cRandomizer.IsEnabled()) {
            m_cRandomizer.Apply(*m_pcRNG, m_cSceneSync, m_pcSkybox);
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error creating the photorealism render engine", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::Reset() {
      /* The scene is rebuilt from anchors at every Update(); a reset
       * only needs a fresh sync so removed/re-added entities settle,
       * plus dropping in-flight camera frames */
      if(m_cEngine.IsCreated()) {
         m_cDebugDraw.Clear();
         m_cDebugDraw.Commit();
         m_cCameraPool.Reset();
         m_cSceneSync.Sync(CSimulator::GetInstance().GetSpace());
         /* Draw a new random environment on reset (dataset generation
          * restarts the experiment with a randomized scene) */
         if(m_cRandomizer.IsEnabled() && m_cRandomizer.AppliesOnReset()) {
            m_cRandomizer.Apply(*m_pcRNG, m_cSceneSync, m_pcSkybox);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::Destroy() {
      if(!m_cEngine.IsCreated()) {
         return;
      }
      if(m_bStats) {
         LogStats();
      }
      filament::Engine& cEngine = m_cEngine.GetEngine();
      m_cDebugDraw.Destroy();
      m_cCameraPool.Destroy();
      if(m_pcDebugView != nullptr) {
         cEngine.destroy(m_pcDebugView);
         cEngine.destroy(m_pcDebugTarget);
         cEngine.destroy(m_pcDebugColor);
         cEngine.destroy(m_pcDebugDepth);
         cEngine.destroyCameraComponent(*m_pcDebugCameraEntity);
         utils::EntityManager::get().destroy(*m_pcDebugCameraEntity);
         delete m_pcDebugCameraEntity;
         m_pcDebugView = nullptr;
      }
      m_cSceneSync.Destroy();
      for(SProp& s_prop : m_vecProps) {
         m_cAssetRegistry.ReleaseModelInstance(s_prop.Model, s_prop.Instance);
      }
      m_cAssetRegistry.Destroy();
      m_cIdScene.Destroy();
      if(m_pcSkybox != nullptr) {
         cEngine.destroy(m_pcSkybox);
         m_pcSkybox = nullptr;
      }
      m_cEngine.Destroy();
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::Update() {
      using TClock = std::chrono::steady_clock;
      TClock::time_point tStart = TClock::now();
      CSpace& cSpace = CSimulator::GetInstance().GetSpace();
      m_cSceneSync.Sync(cSpace);
      double fSync = std::chrono::duration<double>(TClock::now() - tStart).count();
      /* Before the cameras render, so geometry a loop function drew on the
       * previous PostStep is uploaded for this frame. Costs nothing on the
       * ticks where nothing changed. */
      m_cDebugDraw.Commit();
      m_cCameraPool.Update(cSpace.GetSimulationClock());
      if(m_sDebugCamera.Enabled &&
         cSpace.GetSimulationClock() % m_sDebugCamera.Period == 0) {
         RenderDebugFrame();
      }
      double fTotal = std::chrono::duration<double>(TClock::now() - tStart).count();
      ++m_unStatTicks;
      m_fStatSync += fSync;
      m_fStatSyncMax = std::max(m_fStatSyncMax, fSync);
      m_fStatTotal += fTotal;
      m_fStatTotalMax = std::max(m_fStatTotalMax, fTotal);
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::CreateDebugCamera() {
      filament::Engine& cEngine = m_cEngine.GetEngine();
      m_pcDebugColor = filament::Texture::Builder()
         .width(m_sDebugCamera.Width)
         .height(m_sDebugCamera.Height)
         .levels(1)
         .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                filament::Texture::Usage::SAMPLEABLE |
                filament::Texture::Usage::BLIT_SRC)
         .format(filament::Texture::InternalFormat::RGBA8)
         .build(cEngine);
      m_pcDebugDepth = filament::Texture::Builder()
         .width(m_sDebugCamera.Width)
         .height(m_sDebugCamera.Height)
         .levels(1)
         .usage(filament::Texture::Usage::DEPTH_ATTACHMENT)
         .format(filament::Texture::InternalFormat::DEPTH32F)
         .build(cEngine);
      m_pcDebugTarget = filament::RenderTarget::Builder()
         .texture(filament::RenderTarget::AttachmentPoint::COLOR, m_pcDebugColor)
         .texture(filament::RenderTarget::AttachmentPoint::DEPTH, m_pcDebugDepth)
         .build(cEngine);
      m_pcDebugCameraEntity = new utils::Entity(utils::EntityManager::get().create());
      m_pcDebugCamera = cEngine.createCamera(*m_pcDebugCameraEntity);
      m_pcDebugCamera->setProjection(
         double(m_sDebugCamera.FieldOfView),
         double(m_sDebugCamera.Width) / double(m_sDebugCamera.Height),
         0.05, 100.0,
         filament::Camera::Fov::VERTICAL);
      m_cEngine.ApplyExposure(*m_pcDebugCamera);
      m_pcDebugCamera->lookAt(
         {float(m_sDebugCamera.Position.GetX()),
          float(m_sDebugCamera.Position.GetY()),
          float(m_sDebugCamera.Position.GetZ())},
         {float(m_sDebugCamera.LookAt.GetX()),
          float(m_sDebugCamera.LookAt.GetY()),
          float(m_sDebugCamera.LookAt.GetZ())},
         {0.0f, 0.0f, 1.0f});
      m_pcDebugView = cEngine.createView();
      m_pcDebugView->setViewport(
         filament::Viewport(0, 0, m_sDebugCamera.Width, m_sDebugCamera.Height));
      m_pcDebugView->setRenderTarget(m_pcDebugTarget);
      m_pcDebugView->setScene(&m_cEngine.GetScene());
      m_pcDebugView->setCamera(m_pcDebugCamera);
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::SetSunlight(const CVector3& c_direction,
                                         Real f_intensity) {
      m_cSceneSync.SetSunlight(c_direction, f_intensity);
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::SetSkyColor(const CVector3& c_color) {
      if(m_pcSkybox != nullptr) {
         m_pcSkybox->setColor({float(c_color.GetX()),
                               float(c_color.GetY()),
                               float(c_color.GetZ()),
                               1.0f});
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::SetMaterialParam(const CEmbodiedEntity& c_entity,
                                              const std::string& str_name,
                                              Real f_value) {
      m_cSceneSync.SetMaterialParam(c_entity, str_name, f_value);
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::SetMaterialColor(const CEmbodiedEntity& c_entity,
                                              const CVector3& c_color) {
      m_cSceneSync.SetMaterialColor(c_entity, c_color);
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::RandomizeAll() {
      if(!m_cRandomizer.IsEnabled()) {
         THROW_ARGOSEXCEPTION("RandomizeAll() called, but the photorealism "
                              "medium \"" << GetId() << "\" has no "
                              "<randomization> configuration");
      }
      m_cRandomizer.Apply(*m_pcRNG, m_cSceneSync, m_pcSkybox);
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::LogStats() {
      if(m_unStatTicks == 0) {
         return;
      }
      const CPRCameraPool::SStats& sPool = m_cCameraPool.GetStats();
      double fTicks = double(m_unStatTicks);
      std::ostringstream cReport;
      cReport << std::fixed << std::setprecision(2);
      cReport << "[INFO] Photorealism medium \"" << GetId() << "\" timing over "
              << m_unStatTicks << " ticks, "
              << sPool.PeakCameras << " cameras:\n";
      cReport << "[INFO]   update total : avg "
              << m_fStatTotal / fTicks * 1e3 << " ms/tick, max "
              << m_fStatTotalMax * 1e3 << " ms\n";
      cReport << "[INFO]   scene sync   : avg "
              << m_fStatSync / fTicks * 1e3 << " ms/tick, max "
              << m_fStatSyncMax * 1e3 << " ms\n";
      cReport << "[INFO]   camera pool  : collect wait "
              << sPool.CollectWait / fTicks * 1e3 << " ms/tick, submit "
              << sPool.Submit / fTicks * 1e3 << " ms/tick, immediate wait "
              << sPool.ImmediateWait / fTicks * 1e3 << " ms/tick\n";
      if(sPool.CamerasRendered > 0) {
         cReport << "[INFO]   renders      : " << sPool.CamerasRendered
                 << " total, "
                 << (sPool.CollectWait + sPool.Submit + sPool.ImmediateWait) /
                    double(sPool.CamerasRendered) * 1e3
                 << " ms/render (CPU-side)";
      }
      LOG << cReport.str() << std::endl;
   }

   /****************************************/
   /****************************************/

   void CPhotorealismMedium::RenderDebugFrame() {
      std::vector<UInt8> vecPixels;
      m_cEngine.RenderAndReadRGBA(*m_pcDebugView,
                                  m_sDebugCamera.Width,
                                  m_sDebugCamera.Height,
                                  vecPixels);
      char pchFileName[256];
      ::snprintf(pchFileName, sizeof(pchFileName), "%s/frame_%010u.png",
                 m_sDebugCamera.Directory.c_str(),
                 CSimulator::GetInstance().GetSpace().GetSimulationClock());
      if(stbi_write_png(pchFileName,
                        int(m_sDebugCamera.Width),
                        int(m_sDebugCamera.Height),
                        4, vecPixels.data(),
                        int(m_sDebugCamera.Width) * 4) == 0) {
         THROW_ARGOSEXCEPTION("Cannot write frame to \"" << pchFileName << "\"");
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CPhotorealismMedium,
                   "photorealism",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "Photorealistic rendering of the ARGoS space (Filament).",
                   "This medium maintains a photorealistic 3D replica of the simulated space\n"
                   "using the Filament rendering engine. It renders fully headless (no display\n"
                   "server required) using Vulkan (default) or OpenGL. Photorealistic camera\n"
                   "sensors reference this medium by id to obtain per-robot images; an optional\n"
                   "debug camera dumps PNG frames for testing and dataset generation.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <media>\n"
                   "    ...\n"
                   "    <photorealism id=\"pr\" />\n"
                   "    ...\n"
                   "  </media>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "The 'backend' attribute selects the graphics API (\"vulkan\", the default,\n"
                   "or \"opengl\"). The <sun> child node configures the directional sunlight\n"
                   "('direction' as x,y,z; 'intensity' in lux, default 100000; 'cast_shadows').\n"
                   "The <skybox> child node sets the constant sky color ('color' as r,g,b in\n"
                   "[0,1]). The <debug_camera> child node enables PNG frame dumping:\n\n"
                   "  <media>\n"
                   "    <photorealism id=\"pr\" backend=\"vulkan\">\n"
                   "      <sun direction=\"0.6,0.2,-1\" intensity=\"100000\" cast_shadows=\"true\" />\n"
                   "      <skybox color=\"0.53,0.71,0.92\" />\n"
                   "      <debug_camera position=\"2,2,2\" look_at=\"0,0,0\" fov=\"45\"\n"
                   "                    resolution=\"640,480\" period=\"1\" dump=\"frames\" />\n"
                   "    </photorealism>\n"
                   "  </media>\n\n"
                   "The debug camera renders every 'period' ticks and writes\n"
                   "<dump>/frame_<clock>.png.\n\n"
                   "The <lights> child node adds local lights: lamps, headlights, lit\n"
                   "windows. Unlike the sun they have a position and a finite reach, so\n"
                   "they are what makes a night or indoor scene readable. 'intensity' is\n"
                   "in lumens (the number on a real bulb: about 10000 for a street lamp,\n"
                   "800 for a domestic one) and 'falloff' is the radius in metres beyond\n"
                   "which the light has no effect. Keep the falloff close to the useful\n"
                   "reach: an oversized one costs performance without changing the image.\n"
                   "A <spot> also takes a 'direction' and the half-angles 'inner_angle'\n"
                   "and 'outer_angle' in degrees:\n\n"
                   "  <photorealism id=\"pr\">\n"
                   "    <lights>\n"
                   "      <point position=\"3,0,3.2\" intensity=\"12000\" falloff=\"9\"\n"
                   "             color=\"1.0,0.85,0.6\" />\n"
                   "      <spot position=\"0,0,4\" direction=\"0,0,-1\" intensity=\"20000\"\n"
                   "            falloff=\"12\" inner_angle=\"25\" outer_angle=\"40\" />\n"
                   "    </lights>\n"
                   "  </photorealism>\n\n"
                   "Local lights do not cast shadows unless cast_shadows=\"true\": every\n"
                   "shadow-casting local light shares one shadow atlas and costs an extra\n"
                   "render pass per frame.\n\n"
                   "The renderer is physically based, so the image is only as bright as\n"
                   "the exposure allows. The default is the \"sunny 16\" rule, which is\n"
                   "correct for the 100000 lux default sun and renders a lamp-lit scene\n"
                   "black. The <exposure> child node opens the camera up; it applies to\n"
                   "the robot sensors, the debug camera and the <filament> viewer alike,\n"
                   "so they always agree on brightness:\n\n"
                   "  <photorealism id=\"pr\">\n"
                   "    <!-- dusk: about 6 stops brighter than sunny 16 -->\n"
                   "    <exposure aperture=\"2.8\" shutter_speed=\"0.0166\"\n"
                   "              sensitivity=\"400\" />\n"
                   "  </photorealism>\n\n"
                   "'aperture' is an f-number, 'shutter_speed' is in seconds and\n"
                   "'sensitivity' is an ISO value; each stop down in aperture, or\n"
                   "doubling of shutter_speed or sensitivity, doubles the brightness.\n\n"
                   "With stats=\"true\" the medium prints a timing summary (scene sync,\n"
                   "camera render/readback) at the end of the experiment.\n\n"
                   "The <randomization> child node enables domain randomization for\n"
                   "sim-to-real transfer. A new environment is drawn from the configured\n"
                   "ranges (using the ARGoS RNG, so runs are reproducible per seed) at\n"
                   "startup and, unless on_reset=\"false\", at every reset:\n\n"
                   "  <photorealism id=\"pr\">\n"
                   "    <randomization on_reset=\"true\">\n"
                   "      <sun intensity=\"60000:140000\" elevation=\"25:80\" azimuth=\"0:360\" />\n"
                   "      <sky color_min=\"0.3,0.4,0.55\" color_max=\"0.7,0.8,1.0\" />\n"
                   "      <materials targets=\"box,cylinder,floor\"\n"
                   "                 roughness=\"0.2:1.0\" color_jitter=\"0.3\" />\n"
                   "    </randomization>\n"
                   "  </photorealism>\n\n"
                   "<sun> draws the light intensity (lux) and direction (elevation and\n"
                   "azimuth in degrees); <sky> draws each sky color channel between the\n"
                   "two given colors; <materials> randomizes the roughness and jitters\n"
                   "the base colors of the visuals whose class is listed in 'targets'\n"
                   "(LED emissives are never randomized; segmentation ids are\n"
                   "unaffected). Loop functions can also call SetSunlight(),\n"
                   "SetSkyColor(), SetMaterialParam(), SetMaterialColor(), and\n"
                   "RandomizeAll() on the medium directly.",
                   "Usable");

}
