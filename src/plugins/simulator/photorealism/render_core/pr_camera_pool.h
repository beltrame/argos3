/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_camera_pool.h>
 *
 * Owns the offscreen render targets of all robot cameras and schedules
 * their rendering. All cameras due in a tick are rendered inside one
 * Filament frame; pixel readback is asynchronous and double-buffered.
 *
 * Cameras render into private targets, which are then composited into
 * one atlas render target per modality by a blit pass, so each frame
 * issues a single readPixels per modality regardless of the camera
 * count. Beyond the readback overhead, this works around a race in
 * Filament's Vulkan readPixels implementation, which frees command
 * buffers from a worker thread while the driver thread may still be
 * allocating from the same (externally synchronized) command pool:
 * with one readback per atlas the recording window is microseconds
 * long while the fence cannot signal before the GPU has drained the
 * whole frame, so the two cannot overlap in practice.
 *
 * By default the pool is pipelined: the readback issued at tick N is
 * collected at tick N+1, so sensors see a one-tick-old frame and the
 * GPU works while the CPU simulates. In immediate mode the pool blocks
 * until the frame rendered at tick N is available at tick N.
 *
 * Cameras can be registered before the render engine exists (sensors
 * initialize before the medium's PostSpaceInit()); GPU resources are
 * created lazily.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_CAMERA_POOL_H
#define PR_CAMERA_POOL_H

namespace argos {
   class CPRRenderEngine;
   class CPRIdScene;
   struct SAnchor;
}

namespace filament {
   class Scene;
   class View;
   class Camera;
   class Texture;
   class RenderTarget;
   class Material;
   class MaterialInstance;
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_mesh_builder.h>

#include <utils/Entity.h>
#include <math/mat4.h>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace argos {

   struct SPRCameraConfig {
      /** Anchor the camera is mounted on */
      const SAnchor* Anchor = nullptr;
      /** Mount offset in the anchor frame */
      CVector3 PositionOffset;
      CQuaternion OrientationOffset;
      /** The camera looks along the +x axis of the mount frame,
       *  with +z up */
      UInt32 Width = 64;
      UInt32 Height = 64;
      /** Vertical field of view, degrees */
      Real FieldOfView = 60.0;
      Real NearPlane = 0.05;
      Real FarPlane = 20.0;
      /** Render the RGB pass */
      bool RenderRGB = true;
      /** Render the aux (depth + segmentation) pass */
      bool RenderAux = true;
      /** Render every n-th tick (cameras are skewed round-robin) */
      UInt32 FramerateDivider = 1;
      /** Which tick of the divider's cycle this camera renders on.
       *  Negative means auto: the camera is skewed by its handle, so
       *  cameras sharing a divider spread their load over the cycle.
       *  Set it explicitly to keep a group of cameras coincident, as
       *  the lidar does with the faces of one scan: skewed faces would
       *  each depict a different tick. */
      SInt32 Phase = -1;
   };

   class CPRCameraPool {

   public:

      struct SOutput {
         /** The tick the pixels depict */
         UInt32 Tick = 0;
         /** True when this output was delivered in the current tick */
         bool Fresh = false;
         /** True once any frame has been delivered */
         bool Valid = false;
         /** RGBA8 pixels, top-left origin, Width*Height*4 (RGB pass) */
         std::vector<UInt8> RGBA;
         /** Per pixel (entityId, classId, viewDepth, 1) floats,
          *  Width*Height*4 (aux pass) */
         std::vector<float> Aux;
      };

      /** Cumulative timing over all Update() calls, in seconds */
      struct SStats {
         /** Number of Update() calls */
         UInt64 Updates = 0;
         /** Largest number of simultaneously registered cameras */
         size_t PeakCameras = 0;
         /** Number of per-camera renders submitted */
         UInt64 CamerasRendered = 0;
         /** Time blocked waiting for the previous tick's readbacks */
         double CollectWait = 0.0;
         /** Time submitting render + readback commands */
         double Submit = 0.0;
         /** Time blocked in immediate mode for this tick's frames */
         double ImmediateWait = 0.0;
         /** Longest single Update() call */
         double MaxUpdate = 0.0;
         /** beginFrame() refusals waited out before a sensor frame went
          *  through. Nonzero means the GPU is the bottleneck, typically
          *  an interactive viewer sharing the engine. */
         UInt64 FrameRetries = 0;
         /** Ticks where the retry budget ran out and the sensor frame was
          *  submitted into a cancelled frame anyway, i.e. lost. */
         UInt64 FramesLost = 0;
      };

   public:

      void Init(CPRRenderEngine& c_engine,
                CPRIdScene& c_id_scene,
                bool b_immediate);

      void Destroy();

      void Reset();

      /**
       * Registers a camera and returns its handle. Can be called
       * before Init(); GPU resources are created lazily.
       */
      UInt32 RegisterCamera(const SPRCameraConfig& s_config);

      void UnregisterCamera(UInt32 un_handle);

      /**
       * Collects finished readbacks and renders all cameras due this
       * tick. Must run on the render thread after the scene sync.
       */
      void Update(UInt32 un_tick);

      const SOutput& GetOutput(UInt32 un_handle) const;

      const SPRCameraConfig& GetConfig(UInt32 un_handle) const;

      /** The handles of all registered cameras */
      std::vector<UInt32> GetHandles() const;

      /**
       * The world transform of a camera (anchor pose composed with
       * the mount offsets and the axis fix that maps the Filament
       * camera onto the +x-forward, +z-up mount frame). Also used by
       * the filament visualization for its camera-view inset.
       */
      static filament::math::mat4f
      ComputeViewTransform(const SPRCameraConfig& s_config);

      inline SStats GetStats() const {
         std::lock_guard<std::mutex> cLock(m_cMutex);
         return m_sStats;
      }

      inline size_t GetNumCameras() const {
         std::lock_guard<std::mutex> cLock(m_cMutex);
         return m_mapCameras.size();
      }

   private:

      struct SCamera {
         SPRCameraConfig Config;
         /* Filament resources, created lazily */
         bool Created = false;
         utils::Entity CameraEntity;
         filament::Camera* Camera = nullptr;
         filament::View* RGBView = nullptr;
         filament::Texture* RGBColor = nullptr;
         filament::Texture* RGBDepth = nullptr;
         filament::RenderTarget* RGBTarget = nullptr;
         filament::View* AuxView = nullptr;
         filament::Texture* AuxColor = nullptr;
         filament::Texture* AuxDepth = nullptr;
         filament::RenderTarget* AuxTarget = nullptr;
         /* Delivered output */
         SOutput Front;
         /* Readback in flight */
         bool Pending = false;
         UInt32 RenderTick = 0;
         /* Scratch flag: due for rendering in the current Update() */
         bool Due = false;
      };

      /** One camera's tile inside an atlas page */
      struct STile {
         UInt32 CameraHandle = 0;
         UInt32 X = 0;
         UInt32 Y = 0;
         UInt32 Width = 0;
         UInt32 Height = 0;
         utils::Entity Quad;
         filament::MaterialInstance* Material = nullptr;
      };

      /** An atlas page compositing several camera textures, read back
       *  with a single readPixels call */
      struct SPage {
         /** False: RGBA8 (RGB modality); true: RGBA32F (aux) */
         bool Float = false;
         UInt32 Width = 0;
         UInt32 Height = 0;
         filament::Scene* Scene = nullptr;
         filament::View* View = nullptr;
         utils::Entity CameraEntity;
         filament::Camera* Camera = nullptr;
         filament::Texture* Color = nullptr;
         filament::Texture* Depth = nullptr;
         filament::RenderTarget* Target = nullptr;
         std::vector<STile> Tiles;
         /** Readback destination (bytes; holds floats when Float) */
         std::vector<UInt8> Buffer;
         /** Readback in flight */
         bool Pending = false;
         /** Set by the readPixels callback */
         bool Done = true;
      };

      void EnsureResources(SCamera& s_camera);
      void ReleaseResources(SCamera& s_camera);
      void UpdateCameraTransform(SCamera& s_camera);
      /** Copies completed page buffers into the tiles' camera outputs */
      void DemuxPages();
      /** Destroys and re-creates the atlas pages from the current
       *  camera set */
      void RebuildPages();
      void BuildPages(bool b_float);
      void DestroyPages();

   private:

      CPRRenderEngine* m_pcEngine = nullptr;
      CPRIdScene* m_pcIdScene = nullptr;
      bool m_bImmediate = false;
      UInt32 m_unNextHandle = 1;
      std::map<UInt32, SCamera> m_mapCameras;
      std::vector<std::unique_ptr<SPage>> m_vecPages;
      bool m_bPagesDirty = true;
      filament::Material* m_pcBlitMaterial = nullptr;
      SPRMesh m_sQuadMesh;
      SStats m_sStats;
      mutable std::mutex m_cMutex;

   };

}

#endif
