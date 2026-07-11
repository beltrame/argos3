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

#include <cmath>

namespace argos {

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      m_pcEmbodiedEntity = &c_entity.GetComponent<CEmbodiedEntity>("body");
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
         /* Mount */
         SPRCameraConfig sConfig;
         std::string strAnchor("origin");
         GetNodeAttributeOrDefault(t_tree, "anchor", strAnchor, strAnchor);
         if(strAnchor == "origin") {
            sConfig.Anchor = &m_pcEmbodiedEntity->GetOriginAnchor();
         }
         else {
            m_pcEmbodiedEntity->EnableAnchor(strAnchor);
            sConfig.Anchor = &m_pcEmbodiedEntity->GetAnchor(strAnchor);
         }
         GetNodeAttributeOrDefault(t_tree, "position",
                                   sConfig.PositionOffset,
                                   sConfig.PositionOffset);
         CVector3 cOrientationEuler;
         GetNodeAttributeOrDefault(t_tree, "orientation",
                                   cOrientationEuler, cOrientationEuler);
         sConfig.OrientationOffset.FromEulerAngles(
            ToRadians(CDegrees(cOrientationEuler.GetX())),
            ToRadians(CDegrees(cOrientationEuler.GetY())),
            ToRadians(CDegrees(cOrientationEuler.GetZ())));
         /* Image */
         std::string strResolution("64,64");
         GetNodeAttributeOrDefault(t_tree, "resolution", strResolution, strResolution);
         UInt32 punResolution[2];
         ParseValues<UInt32>(strResolution, 2, punResolution, ',');
         sConfig.Width = punResolution[0];
         sConfig.Height = punResolution[1];
         GetNodeAttributeOrDefault(t_tree, "fov", sConfig.FieldOfView, sConfig.FieldOfView);
         GetNodeAttributeOrDefault(t_tree, "near", sConfig.NearPlane, sConfig.NearPlane);
         GetNodeAttributeOrDefault(t_tree, "far", sConfig.FarPlane, sConfig.FarPlane);
         GetNodeAttributeOrDefault(t_tree, "framerate_divider",
                                   sConfig.FramerateDivider,
                                   sConfig.FramerateDivider);
         if(sConfig.FramerateDivider == 0) {
            THROW_ARGOSEXCEPTION("framerate_divider must be at least 1");
         }
         m_fFarPlane = sConfig.FarPlane;
         /* Modalities */
         std::string strModalities("rgb,depth,seg");
         GetNodeAttributeOrDefault(t_tree, "modalities", strModalities, strModalities);
         m_bRGBEnabled = strModalities.find("rgb") != std::string::npos;
         m_bDepthEnabled = strModalities.find("depth") != std::string::npos;
         m_bSegEnabled = strModalities.find("seg") != std::string::npos;
         sConfig.RenderRGB = m_bRGBEnabled;
         sConfig.RenderAux = m_bDepthEnabled || m_bSegEnabled;
         /* Register with the pool; GPU resources are created lazily
          * once the medium's render engine exists */
         m_unCameraHandle = m_pcMedium->GetCameraPool().RegisterCamera(sConfig);
         /* Frame metadata is valid from the start */
         m_sFrame.Width = sConfig.Width;
         m_sFrame.Height = sConfig.Height;
         Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the photorealistic camera sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::Update() {
      m_bNewFrame = false;
      if(IsDisabled()) {
         return;
      }
      const CPRCameraPool::SOutput& sOutput =
         m_pcMedium->GetCameraPool().GetOutput(m_unCameraHandle);
      if(!sOutput.Fresh || !sOutput.Valid) {
         return;
      }
      m_bNewFrame = true;
      m_sFrame.Tick = sOutput.Tick;
      const size_t unPixels = size_t(m_sFrame.Width) * m_sFrame.Height;
      if(m_bRGBEnabled) {
         m_sFrame.RGB.resize(unPixels * 3);
         for(size_t i = 0; i < unPixels; ++i) {
            m_sFrame.RGB[i * 3]     = sOutput.RGBA[i * 4];
            m_sFrame.RGB[i * 3 + 1] = sOutput.RGBA[i * 4 + 1];
            m_sFrame.RGB[i * 3 + 2] = sOutput.RGBA[i * 4 + 2];
         }
      }
      if(m_bDepthEnabled) {
         m_sFrame.Depth.resize(unPixels);
      }
      if(m_bSegEnabled) {
         m_sFrame.EntityId.resize(unPixels);
         m_sFrame.ClassId.resize(unPixels);
      }
      if(m_bDepthEnabled || m_bSegEnabled) {
         for(size_t i = 0; i < unPixels; ++i) {
            const float* pfAux = &sOutput.Aux[i * 4];
            auto unEntityId = UInt16(std::lround(pfAux[0]));
            if(m_bSegEnabled) {
               m_sFrame.EntityId[i] = unEntityId;
               m_sFrame.ClassId[i] = UInt8(std::lround(pfAux[1]));
            }
            if(m_bDepthEnabled) {
               /* Background pixels carry the far-plane distance */
               m_sFrame.Depth[i] = unEntityId != 0 ? Real(pfAux[2]) : m_fFarPlane;
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::Reset() {
      m_bNewFrame = false;
      m_sFrame.Tick = 0;
      m_sFrame.RGB.clear();
      m_sFrame.Depth.clear();
      m_sFrame.EntityId.clear();
      m_sFrame.ClassId.clear();
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticCameraDefaultSensor::Destroy() {
      if(m_pcMedium != nullptr && m_unCameraHandle != 0) {
         m_pcMedium->GetCameraPool().UnregisterCamera(m_unCameraHandle);
         m_unCameraHandle = 0;
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
