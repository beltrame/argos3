/**
 * @file <argos3/plugins/simulator/photorealism/photorealism_medium.h>
 *
 * The photorealism medium owns the Filament render engine and keeps a
 * photorealistic replica of the ARGoS space. It works fully headless:
 * no <visualization> section is needed. Camera sensors reference this
 * medium by id to obtain rendered images; an optional debug camera can
 * dump frames to PNG files for testing and dataset generation.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PHOTOREALISM_MEDIUM_H
#define PHOTOREALISM_MEDIUM_H

namespace argos {
   class CPhotorealismMedium;
}

#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_render_engine.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_scene_sync.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_id_scene.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_camera_pool.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_debug_draw.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_randomizer.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_asset_registry.h>

#include <memory>
#include <string>
#include <vector>

namespace filament {
   class View;
   class Camera;
   class Texture;
   class RenderTarget;
   class Skybox;
}

namespace utils {
   class Entity;
}

namespace argos {

   class CPhotorealismMedium : public CMedium {

   public:

      CPhotorealismMedium() {}
      virtual ~CPhotorealismMedium() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void PostSpaceInit();
      virtual void Reset();
      virtual void Destroy();
      virtual void Update();

      inline CPRRenderEngine& GetRenderEngine() {
         return m_cEngine;
      }

      inline CPRSceneSync& GetSceneSync() {
         return m_cSceneSync;
      }

      inline CPRCameraPool& GetCameraPool() {
         return m_cCameraPool;
      }

      /**
       * Line overlays on the scene: planner paths, graphs, anything a
       * loop function wants to annotate the world with. Drawn only in
       * the interactive viewer, never in a camera or lidar reading.
       */
      inline CPRDebugDraw& GetDebugDraw() {
         return m_cDebugDraw;
      }

      /*
       * Domain randomization API for loop functions
       */

      /** Changes the sunlight direction and intensity */
      void SetSunlight(const CVector3& c_direction, Real f_intensity);

      /** Changes the constant sky color (r, g, b in [0, 1]) */
      void SetSkyColor(const CVector3& c_color);

      /** Sets "roughness" or "metallic" on an entity's visual */
      void SetMaterialParam(const CEmbodiedEntity& c_entity,
                            const std::string& str_name,
                            Real f_value);

      /** Sets the base color of an entity's visual */
      void SetMaterialColor(const CEmbodiedEntity& c_entity,
                            const CVector3& c_color);

      /** Draws a new random environment from the <randomization>
       *  configuration (which must be present) */
      void RandomizeAll();

   private:

      void CreateDebugCamera();
      void RenderDebugFrame();
      void LogStats();

   private:

      /* Configuration */
      std::string m_strBackend = "vulkan";
      CPRSceneSync::SSunlight m_sSunlight;
      CVector3 m_cSkyColor = CVector3(0.53, 0.71, 0.92);

      /* Local lights (street lamps, headlights, windows), placed once
       * at PostSpaceInit() and never moved */
      std::vector<CPRSceneSync::SLight> m_vecLights;

      /* Camera exposure, shared by the sensors, the debug camera and
       * the interactive viewer */
      SPRExposure m_sExposure;

      /* HDR environment (cmgen KTX pair), replaces the constant
       * ambient light and the color skybox when configured */
      struct SEnvironment {
         bool Enabled = false;
         std::string Ibl;
         std::string Skybox;
         Real Intensity = 30000.0;
      } m_sEnvironment;

      /* Static glTF props dressing the scene. They take no part in
       * physics, but they are real geometry to the cameras and so
       * carry an entity id like any other renderable. */
      struct SProp {
         std::string Model;
         CVector3 Position;
         CVector3 OrientationEuler; /* z,y,x degrees */
         Real Scale = 1.0;
         UInt16 EntityId = 0;
         CPRAssetRegistry::SInstance Instance;
      };
      std::vector<SProp> m_vecProps;

      struct SDebugCamera {
         bool Enabled = false;
         CVector3 Position = CVector3(2.0, 2.0, 2.0);
         CVector3 LookAt = CVector3(0.0, 0.0, 0.0);
         Real FieldOfView = 45.0; /* vertical, degrees */
         UInt32 Width = 640;
         UInt32 Height = 480;
         UInt32 Period = 1; /* dump every n ticks */
         std::string Directory = "frames";
      } m_sDebugCamera;

      /* Frame delivery latency: pipelined (default) or immediate */
      bool m_bImmediate = false;

      /* Domain randomization */
      CPRRandomizer m_cRandomizer;
      CRandom::CRNG* m_pcRNG = nullptr;

      /* Timing statistics, reported at Destroy() when enabled */
      bool m_bStats = false;
      UInt64 m_unStatTicks = 0;
      double m_fStatSync = 0.0;    /* cumulative scene sync, seconds */
      double m_fStatSyncMax = 0.0;
      double m_fStatTotal = 0.0;   /* cumulative Update(), seconds */
      double m_fStatTotalMax = 0.0;

      /* Asset search path from the XML configuration */
      std::string m_strAssetPath;

      /* Entity id prefixes excluded from rendering (hide_prefix attr) */
      std::vector<std::string> m_vecHiddenIdPrefixes;

      /* When false, the arena floor plane is not rendered */
      bool m_bDrawFloor = true;

      /* Filament state */
      CPRRenderEngine m_cEngine;
      CPRSceneSync m_cSceneSync;
      CPRIdScene m_cIdScene;
      CPRAssetRegistry m_cAssetRegistry;
      CPRCameraPool m_cCameraPool;
      CPRDebugDraw m_cDebugDraw;
      filament::Skybox* m_pcSkybox = nullptr;
      filament::View* m_pcDebugView = nullptr;
      filament::Camera* m_pcDebugCamera = nullptr;
      filament::Texture* m_pcDebugColor = nullptr;
      filament::Texture* m_pcDebugDepth = nullptr;
      filament::RenderTarget* m_pcDebugTarget = nullptr;
      utils::Entity* m_pcDebugCameraEntity = nullptr;

   };

}

#endif
