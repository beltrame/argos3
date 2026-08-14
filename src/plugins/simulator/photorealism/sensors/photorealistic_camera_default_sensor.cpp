/**
 * @file <argos3/plugins/simulator/photorealism/sensors/photorealistic_camera_default_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "photorealistic_camera_default_sensor.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/plugins/simulator/photorealism/photorealism_medium.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_camera_pool.h>

#include <algorithm>
#include <cmath>

namespace argos {

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      m_pcEmbodiedEntity = &c_entity.GetComponent<CEmbodiedEntity>("body");
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::ParseMount(
      TConfigurationNode& t_node, const std::string& str_id) {
      SPRCameraConfig sConfig;
      SMount sMount;
      sMount.Id = str_id;
      /* Mount anchor */
      std::string strAnchor("origin");
      GetNodeAttributeOrDefault(t_node, "anchor", strAnchor, strAnchor);
      if(strAnchor == "origin") {
         sConfig.Anchor = &m_pcEmbodiedEntity->GetOriginAnchor();
      }
      else {
         m_pcEmbodiedEntity->EnableAnchor(strAnchor);
         sConfig.Anchor = &m_pcEmbodiedEntity->GetAnchor(strAnchor);
      }
      GetNodeAttributeOrDefault(t_node, "position",
                                sConfig.PositionOffset, sConfig.PositionOffset);
      CVector3 cOrientationEuler;
      GetNodeAttributeOrDefault(t_node, "orientation",
                                cOrientationEuler, cOrientationEuler);
      sConfig.OrientationOffset.FromEulerAngles(
         ToRadians(CDegrees(cOrientationEuler.GetX())),
         ToRadians(CDegrees(cOrientationEuler.GetY())),
         ToRadians(CDegrees(cOrientationEuler.GetZ())));
      /* Image */
      std::string strResolution("64,64");
      GetNodeAttributeOrDefault(t_node, "resolution", strResolution, strResolution);
      UInt32 punResolution[2];
      ParseValues<UInt32>(strResolution, 2, punResolution, ',');
      sConfig.Width = punResolution[0];
      sConfig.Height = punResolution[1];
      GetNodeAttributeOrDefault(t_node, "fov", sConfig.FieldOfView, sConfig.FieldOfView);
      GetNodeAttributeOrDefault(t_node, "near", sConfig.NearPlane, sConfig.NearPlane);
      GetNodeAttributeOrDefault(t_node, "far", sConfig.FarPlane, sConfig.FarPlane);
      GetNodeAttributeOrDefault(t_node, "framerate_divider",
                                sConfig.FramerateDivider,
                                sConfig.FramerateDivider);
      if(sConfig.FramerateDivider == 0) {
         THROW_ARGOSEXCEPTION("framerate_divider must be at least 1");
      }
      sMount.FarPlane = sConfig.FarPlane;
      /* Modalities */
      std::string strModalities("rgb,depth,seg");
      GetNodeAttributeOrDefault(t_node, "modalities", strModalities, strModalities);
      sMount.RGBEnabled = strModalities.find("rgb") != std::string::npos;
      sMount.DepthEnabled = strModalities.find("depth") != std::string::npos;
      sMount.SegEnabled = strModalities.find("seg") != std::string::npos;
      sConfig.RenderRGB = sMount.RGBEnabled;
      sConfig.RenderAux = sMount.DepthEnabled || sMount.SegEnabled;
      /* Register with the pool; GPU resources are created lazily once the
       * medium's render engine exists */
      sMount.Handle = m_pcMedium->GetCameraPool().RegisterCamera(sConfig);
      sMount.Frame.Id = str_id;
      sMount.Frame.Width = sConfig.Width;
      sMount.Frame.Height = sConfig.Height;
      m_vecMounts.push_back(std::move(sMount));
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_PhotorealisticCameraSensor::Init(t_tree);
         /* Medium */
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         m_pcMedium = &CSimulator::GetInstance().GetMedium<CPhotorealismMedium>(strMedium);
         /* Either a list of <camera> children, or the classic form with the
          * attributes on this element. Both are supported: every existing
          * experiment uses the latter. */
         if(NodeExists(t_tree, "camera")) {
            TConfigurationNodeIterator itCamera("camera");
            for(itCamera = itCamera.begin(&t_tree);
                itCamera != itCamera.end();
                ++itCamera) {
               std::string strId;
               GetNodeAttribute(*itCamera, "id", strId);
               for(const SMount& sExisting : m_vecMounts) {
                  if(sExisting.Id == strId) {
                     THROW_ARGOSEXCEPTION("duplicate camera id \"" << strId << "\"");
                  }
               }
               ParseMount(*itCamera, strId);
            }
            if(m_vecMounts.empty()) {
               THROW_ARGOSEXCEPTION("no <camera> children found");
            }
         }
         else {
            ParseMount(t_tree, "default");
         }
         Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the photorealistic camera sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::Update() {
      for(SMount& sMount : m_vecMounts) {
         sMount.NewFrame = false;
      }
      if(IsDisabled()) {
         return;
      }
      for(SMount& sMount : m_vecMounts) {
         const CPRCameraPool::SOutput& sOutput =
            m_pcMedium->GetCameraPool().GetOutput(sMount.Handle);
         if(!sOutput.Fresh || !sOutput.Valid) {
            continue;
         }
         sMount.NewFrame = true;
         SFrame& sFrame = sMount.Frame;
         sFrame.Tick = sOutput.Tick;
         const size_t unPixels = size_t(sFrame.Width) * sFrame.Height;
         if(sMount.RGBEnabled) {
            sFrame.RGB.resize(unPixels * 3);
            for(size_t i = 0; i < unPixels; ++i) {
               sFrame.RGB[i * 3]     = sOutput.RGBA[i * 4];
               sFrame.RGB[i * 3 + 1] = sOutput.RGBA[i * 4 + 1];
               sFrame.RGB[i * 3 + 2] = sOutput.RGBA[i * 4 + 2];
            }
         }
         if(sMount.DepthEnabled) {
            sFrame.Depth.resize(unPixels);
         }
         if(sMount.SegEnabled) {
            sFrame.EntityId.resize(unPixels);
            sFrame.ClassId.resize(unPixels);
         }
         if(sMount.DepthEnabled || sMount.SegEnabled) {
            for(size_t i = 0; i < unPixels; ++i) {
               const float* pfAux = &sOutput.Aux[i * 4];
               auto unEntityId = UInt16(std::lround(pfAux[0]));
               if(sMount.SegEnabled) {
                  sFrame.EntityId[i] = unEntityId;
                  sFrame.ClassId[i] = UInt8(std::lround(pfAux[1]));
               }
               if(sMount.DepthEnabled) {
                  /* Background pixels carry the far-plane distance. This
                   * relies on every renderable having a nonzero entity id:
                   * the aux buffer clears to zero, so id 0 means nothing
                   * was drawn. Geometry registered with id 0 would be
                   * silently reported as empty space instead.
                   *
                   * Real fragments can land marginally beyond the nominal
                   * far plane, so clamp to keep the documented range. */
                  sFrame.Depth[i] = unEntityId != 0
                     ? std::min(Real(pfAux[2]), sMount.FarPlane)
                     : sMount.FarPlane;
               }
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::Reset() {
      for(SMount& sMount : m_vecMounts) {
         sMount.NewFrame = false;
         sMount.Frame.Tick = 0;
         sMount.Frame.RGB.clear();
         sMount.Frame.Depth.clear();
         sMount.Frame.EntityId.clear();
         sMount.Frame.ClassId.clear();
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::Destroy() {
      if(m_pcMedium == nullptr) {
         return;
      }
      for(SMount& sMount : m_vecMounts) {
         if(sMount.Handle != 0) {
            m_pcMedium->GetCameraPool().UnregisterCamera(sMount.Handle);
            sMount.Handle = 0;
         }
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPhotorealisticCameraDefaultSensor,
                   "photorealistic_camera", "default",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "A photorealistic camera producing RGB, depth and segmentation images.",
                   "This sensor renders the scene maintained by the <photorealism> medium\n"
                   "from a camera mounted on the robot. Each frame carries RGB pixels, metric\n"
                   "depth along the optical axis, and per-pixel entity/class segmentation ids.\n"
                   "The camera looks along the +x axis of its mount frame, with +z up.\n\n"
                   "By default frames are pipelined: the frame available in a control step\n"
                   "depicts the previous tick (like a real camera; keeps the GPU and CPU\n"
                   "working in parallel). Set latency=\"immediate\" on the medium for\n"
                   "same-tick frames.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <photorealistic_camera implementation=\"default\" medium=\"pr\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "The 'medium' attribute must name a <photorealism> medium in <media>.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  <photorealistic_camera implementation=\"default\" medium=\"pr\"\n"
                   "                         anchor=\"origin\" position=\"0.1,0,0.15\"\n"
                   "                         orientation=\"0,0,0\" resolution=\"64,64\"\n"
                   "                         fov=\"60\" near=\"0.05\" far=\"20\"\n"
                   "                         modalities=\"rgb,depth,seg\"\n"
                   "                         framerate_divider=\"1\" />\n\n"
                   "'anchor' selects the mount anchor of the robot body. 'position' and\n"
                   "'orientation' (Euler z,y,x in degrees) offset the camera in the anchor\n"
                   "frame. 'fov' is the vertical field of view in degrees. 'modalities'\n"
                   "selects any of rgb, depth, seg. 'framerate_divider' renders the camera\n"
                   "every n-th tick only (cameras are skewed round-robin to spread load).",
                   "Usable");

}
