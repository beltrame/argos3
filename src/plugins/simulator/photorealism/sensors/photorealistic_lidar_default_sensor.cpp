/**
 * @file <argos3/plugins/simulator/photorealism/sensors/photorealistic_lidar_default_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "photorealistic_lidar_default_sensor.h"

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

   void CPhotorealisticLidarDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      m_pcEmbodiedEntity = &c_entity.GetComponent<CEmbodiedEntity>("body");
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticLidarDefaultSensor::BuildRayTable(
      TConfigurationNode& t_tree,
      const SPRCameraConfig& s_base,
      UInt32 un_faces,
      UInt32 un_face_width,
      UInt32 un_face_height) {
      /* Vertical ray pattern. A VLP-16 has 16 channels evenly spaced
       * from -15 to +15 degrees. */
      UInt32 unRings = 16;
      GetNodeAttributeOrDefault(t_tree, "rings", unRings, unRings);
      if(unRings == 0) {
         THROW_ARGOSEXCEPTION("rings must be at least 1");
      }
      std::string strVerticalFov("-15,15");
      GetNodeAttributeOrDefault(t_tree, "vertical_fov",
                                strVerticalFov, strVerticalFov);
      Real pfVerticalFov[2];
      ParseValues<Real>(strVerticalFov, 2, pfVerticalFov, ',');
      const CRadians cElevationMin = ToRadians(CDegrees(pfVerticalFov[0]));
      const CRadians cElevationMax = ToRadians(CDegrees(pfVerticalFov[1]));
      if(cElevationMax < cElevationMin) {
         THROW_ARGOSEXCEPTION("vertical_fov must be given as \"min,max\"");
      }
      /* Azimuth ray pattern */
      Real fHorizontalResolution = 0.2;
      GetNodeAttributeOrDefault(t_tree, "horizontal_resolution",
                                fHorizontalResolution, fHorizontalResolution);
      if(fHorizontalResolution <= 0.0) {
         THROW_ARGOSEXCEPTION("horizontal_resolution must be positive");
      }
      auto unAzimuths = UInt32(std::lround(360.0 / fHorizontalResolution));
      if(unAzimuths == 0) {
         THROW_ARGOSEXCEPTION("horizontal_resolution is coarser than a full turn");
      }

      /*
       * Face geometry. The faces have to tile the azimuth range
       * seamlessly, so the horizontal field of view is not free: it
       * must be exactly 360/faces. Filament is driven with a vertical
       * fov and the aspect ratio decides the horizontal one,
       *
       *    tan(hfov/2) = aspect * tan(vfov/2)
       *
       * so the vertical fov is what gets solved for. That makes the
       * elevation coverage a consequence of the pixel budget rather
       * than something the user sets, hence the check below.
       */
      const CRadians cFaceSpan = CRadians::TWO_PI / Real(un_faces);
      const Real fTanHalfH = std::tan(0.5 * cFaceSpan.GetValue());
      const Real fAspect = Real(un_face_width) / Real(un_face_height);
      const Real fTanHalfV = fTanHalfH / fAspect;
      const CRadians cVerticalFov(2.0 * std::atan(fTanHalfV));
      const CRadians cElevationLimit = 0.5 * cVerticalFov;
      if(cElevationLimit.GetValue() <= std::max(std::abs(cElevationMin.GetValue()),
                                                std::abs(cElevationMax.GetValue()))) {
         THROW_ARGOSEXCEPTION(
            "the faces are too wide to cover the requested vertical_fov: "
            << un_faces << " faces of " << un_face_width << "x"
            << un_face_height << " pixels see +/- "
            << ToDegrees(cElevationLimit) << ", the rays need +/- "
            << ToDegrees(CRadians(std::max(std::abs(cElevationMin.GetValue()),
                                           std::abs(cElevationMax.GetValue()))))
            << ". Make the faces taller relative to their width, or use "
               "fewer of them.");
      }

      /* Register one pool camera per face, all sharing a phase so that
       * every ray of a scan depicts the same tick */
      m_vecFaces.resize(un_faces);
      for(UInt32 unFace = 0; unFace < un_faces; ++unFace) {
         SPRCameraConfig sConfig = s_base;
         sConfig.Width = un_face_width;
         sConfig.Height = un_face_height;
         sConfig.FieldOfView = ToDegrees(cVerticalFov).GetValue();
         /* Depth and segmentation only: the lidar never needs colour */
         sConfig.RenderRGB = false;
         sConfig.RenderAux = true;
         sConfig.Phase = 0;
         /* Turn the face about the sensor's own vertical axis */
         CQuaternion cFaceRotation;
         cFaceRotation.FromAngleAxis(cFaceSpan * Real(unFace), CVector3::Z);
         sConfig.OrientationOffset = s_base.OrientationOffset * cFaceRotation;
         m_vecFaces[unFace].Handle =
            m_pcMedium->GetCameraPool().RegisterCamera(sConfig);
         m_vecFaces[unFace].Width = un_face_width;
         m_vecFaces[unFace].Height = un_face_height;
      }

      /*
       * Precompute, for every ray, which pixel of which face carries
       * its depth. The pattern is fixed in the sensor frame, so this
       * survives the whole run: a tick then costs one lookup and one
       * multiply per ray.
       */
      m_sScan.NumRings = unRings;
      m_sScan.NumAzimuths = unAzimuths;
      m_sScan.MaxRange = s_base.FarPlane;
      m_sScan.Readings.resize(size_t(unAzimuths) * unRings);
      m_vecSamples.resize(m_sScan.Readings.size());
      const CRadians cElevationStep = unRings > 1
         ? (cElevationMax - cElevationMin) / Real(unRings - 1)
         : CRadians::ZERO;
      const CRadians cAzimuthStep = CRadians::TWO_PI / Real(unAzimuths);
      for(UInt32 unAzimuth = 0; unAzimuth < unAzimuths; ++unAzimuth) {
         const CRadians cAzimuth = cAzimuthStep * Real(unAzimuth);
         /* The face whose centre is nearest this azimuth */
         auto unFace = UInt32(std::lround(cAzimuth / cFaceSpan)) % un_faces;
         CRadians cLocalAzimuth = cAzimuth - cFaceSpan * Real(unFace);
         cLocalAzimuth.SignedNormalize();
         const Real fCosLocal = std::cos(cLocalAzimuth.GetValue());
         const Real fSinLocal = std::sin(cLocalAzimuth.GetValue());
         for(UInt32 unRing = 0; unRing < unRings; ++unRing) {
            const CRadians cElevation =
               cElevationMin + cElevationStep * Real(unRing);
            const Real fCosElevation = std::cos(cElevation.GetValue());
            const Real fSinElevation = std::sin(cElevation.GetValue());
            /* Ray in the face's own frame: +x along the optical axis */
            const Real fForward = fCosElevation * fCosLocal;
            const Real fLeft    = fCosElevation * fSinLocal;
            const Real fUp      = fSinElevation;
            /* Project onto the image. The renderer maps screen right
             * to mount -y and screen up to mount +z. */
            const Real fU = (-fLeft / fForward) / fTanHalfH;
            const Real fV = ( fUp   / fForward) / fTanHalfV;
            auto nCol = SInt32(std::floor(0.5 * (fU + 1.0) * un_face_width));
            auto nRow = SInt32(std::floor(0.5 * (1.0 - fV) * un_face_height));
            /* Rays on a face boundary can land a hair outside */
            nCol = std::min(std::max(nCol, 0), SInt32(un_face_width) - 1);
            nRow = std::min(std::max(nRow, 0), SInt32(un_face_height) - 1);

            const size_t unIndex = size_t(unAzimuth) * unRings + unRing;
            SSample& sSample = m_vecSamples[unIndex];
            sSample.Face = unFace;
            sSample.Pixel = UInt32(nRow) * un_face_width + UInt32(nCol);
            /* The aux pass stores depth along the optical axis; the
             * ray direction is a unit vector, so its forward component
             * is the cosine of the angle off that axis */
            sSample.DepthToRange = 1.0 / fForward;
            sSample.Direction.Set(fCosElevation * std::cos(cAzimuth.GetValue()),
                                  fCosElevation * std::sin(cAzimuth.GetValue()),
                                  fSinElevation);

            SReading& sReading = m_sScan.Readings[unIndex];
            sReading.Ring = unRing;
            sReading.Azimuth = cAzimuth;
            sReading.Elevation = cElevation;
            sReading.Range = m_sScan.MaxRange;
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticLidarDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_PhotorealisticLidarSensor::Init(t_tree);
         /* Medium */
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         m_pcMedium = &CSimulator::GetInstance().GetMedium<CPhotorealismMedium>(strMedium);
         /* Mount */
         SPRCameraConfig sBase;
         std::string strAnchor("origin");
         GetNodeAttributeOrDefault(t_tree, "anchor", strAnchor, strAnchor);
         if(strAnchor == "origin") {
            sBase.Anchor = &m_pcEmbodiedEntity->GetOriginAnchor();
         }
         else {
            m_pcEmbodiedEntity->EnableAnchor(strAnchor);
            sBase.Anchor = &m_pcEmbodiedEntity->GetAnchor(strAnchor);
         }
         GetNodeAttributeOrDefault(t_tree, "position",
                                   sBase.PositionOffset, sBase.PositionOffset);
         CVector3 cOrientationEuler;
         GetNodeAttributeOrDefault(t_tree, "orientation",
                                   cOrientationEuler, cOrientationEuler);
         sBase.OrientationOffset.FromEulerAngles(
            ToRadians(CDegrees(cOrientationEuler.GetX())),
            ToRadians(CDegrees(cOrientationEuler.GetY())),
            ToRadians(CDegrees(cOrientationEuler.GetZ())));
         /* Ranges. A VLP-16 reaches 100 m; 20 m is the range the
          * planners using this sensor are configured for. */
         sBase.FarPlane = 20.0;
         GetNodeAttributeOrDefault(t_tree, "max_range",
                                   sBase.FarPlane, sBase.FarPlane);
         if(sBase.FarPlane <= 0.0) {
            THROW_ARGOSEXCEPTION("max_range must be positive");
         }
         GetNodeAttributeOrDefault(t_tree, "near", sBase.NearPlane, sBase.NearPlane);
         GetNodeAttributeOrDefault(t_tree, "framerate_divider",
                                   sBase.FramerateDivider, sBase.FramerateDivider);
         if(sBase.FramerateDivider == 0) {
            THROW_ARGOSEXCEPTION("framerate_divider must be at least 1");
         }
         /* Faces */
         UInt32 unFaces = 4;
         GetNodeAttributeOrDefault(t_tree, "faces", unFaces, unFaces);
         if(unFaces < 3) {
            /* Two faces would each need a 180 degree horizontal field
             * of view, which a pinhole camera cannot reach */
            THROW_ARGOSEXCEPTION("faces must be at least 3");
         }
         std::string strFaceResolution("512,192");
         GetNodeAttributeOrDefault(t_tree, "face_resolution",
                                   strFaceResolution, strFaceResolution);
         UInt32 punFaceResolution[2];
         ParseValues<UInt32>(strFaceResolution, 2, punFaceResolution, ',');
         if(punFaceResolution[0] == 0 || punFaceResolution[1] == 0) {
            THROW_ARGOSEXCEPTION("face_resolution must be positive");
         }
         BuildRayTable(t_tree, sBase, unFaces,
                       punFaceResolution[0], punFaceResolution[1]);
         Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the photorealistic lidar sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticLidarDefaultSensor::Update() {
      m_bNewScan = false;
      if(IsDisabled()) {
         return;
      }
      /* A scan is only assembled when every face has rendered this
       * tick, so it never mixes views from different ticks. The faces
       * share a phase and a divider, so they arrive together. */
      CPRCameraPool& cPool = m_pcMedium->GetCameraPool();
      for(const SFace& sFace : m_vecFaces) {
         const CPRCameraPool::SOutput& sOutput = cPool.GetOutput(sFace.Handle);
         if(!sOutput.Fresh || !sOutput.Valid) {
            return;
         }
      }
      std::vector<const CPRCameraPool::SOutput*> vecOutputs;
      vecOutputs.reserve(m_vecFaces.size());
      for(const SFace& sFace : m_vecFaces) {
         vecOutputs.push_back(&cPool.GetOutput(sFace.Handle));
      }
      m_bNewScan = true;
      m_sScan.Tick = vecOutputs.front()->Tick;
      for(size_t i = 0; i < m_vecSamples.size(); ++i) {
         const SSample& sSample = m_vecSamples[i];
         SReading& sReading = m_sScan.Readings[i];
         const float* pfAux =
            &vecOutputs[sSample.Face]->Aux[size_t(sSample.Pixel) * 4];
         /* The aux buffer clears to zero and every renderable carries
          * a nonzero entity id, so id 0 means nothing was drawn */
         auto unEntityId = UInt16(std::lround(pfAux[0]));
         const Real fRange = unEntityId != 0
            ? Real(pfAux[2]) * sSample.DepthToRange
            : m_sScan.MaxRange;
         if(unEntityId == 0 || fRange >= m_sScan.MaxRange) {
            /* Nothing within range along this ray. Off-axis rays can
             * see past the far plane, which is a planar clip, so a
             * return beyond the maximum range is possible and is
             * reported as a miss rather than clamped onto an obstacle
             * that is not there. */
            sReading.Range = m_sScan.MaxRange;
            sReading.Hit = false;
            sReading.EntityId = 0;
            sReading.ClassId = 0;
            sReading.Position = sSample.Direction * m_sScan.MaxRange;
         }
         else {
            sReading.Range = fRange;
            sReading.Hit = true;
            sReading.EntityId = unEntityId;
            sReading.ClassId = UInt8(std::lround(pfAux[1]));
            sReading.Position = sSample.Direction * fRange;
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticLidarDefaultSensor::Reset() {
      m_bNewScan = false;
      m_sScan.Tick = 0;
      for(SReading& sReading : m_sScan.Readings) {
         sReading.Range = m_sScan.MaxRange;
         sReading.Hit = false;
         sReading.EntityId = 0;
         sReading.ClassId = 0;
         sReading.Position = CVector3::ZERO;
      }
   }

   /****************************************/
   /****************************************/

   void CPhotorealisticLidarDefaultSensor::Destroy() {
      if(m_pcMedium == nullptr) {
         return;
      }
      for(SFace& sFace : m_vecFaces) {
         if(sFace.Handle != 0) {
            m_pcMedium->GetCameraPool().UnregisterCamera(sFace.Handle);
            sFace.Handle = 0;
         }
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPhotorealisticLidarDefaultSensor,
                   "photorealistic_lidar", "default",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "A spinning lidar rendered from the photorealism medium.",
                   "This sensor produces a full revolution of laser returns over the scene\n"
                   "maintained by the <photorealism> medium. The defaults model a Velodyne\n"
                   "VLP-16: 16 channels evenly spaced from -15 to +15 degrees, a 360 degree\n"
                   "sweep at 0.2 degree steps, and a 20 m maximum range.\n\n"
                   "Readings are ordered azimuth-major, the order a VLP-16 emits them:\n"
                   "index = azimuth * rings + ring. Each carries a range, an endpoint in the\n"
                   "sensor frame (+x forward, +y left, +z up), and the entity and class ids\n"
                   "of what it hit. A ray that reaches the maximum range without hitting\n"
                   "anything is flagged as a miss: it still clears the space it crossed, but\n"
                   "marks no obstacle at its endpoint.\n\n"
                   "A scan is a snapshot. Every ray in it depicts one simulation tick, where\n"
                   "a real spinning lidar smears a revolution over its duration.\n\n"
                   "Internally the scene is rendered as a ring of depth views which are\n"
                   "resampled into the ray pattern, because a pinhole camera cannot cover a\n"
                   "full turn. The views cost the same as the equivalent depth cameras.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <photorealistic_lidar implementation=\"default\" medium=\"pr\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "The 'medium' attribute must name a <photorealism> medium in <media>.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  <photorealistic_lidar implementation=\"default\" medium=\"pr\"\n"
                   "                        anchor=\"origin\" position=\"0,0,0.3\"\n"
                   "                        orientation=\"0,0,0\" rings=\"16\"\n"
                   "                        vertical_fov=\"-15,15\"\n"
                   "                        horizontal_resolution=\"0.2\"\n"
                   "                        max_range=\"20\" near=\"0.05\"\n"
                   "                        faces=\"4\" face_resolution=\"512,192\"\n"
                   "                        framerate_divider=\"1\" />\n\n"
                   "'anchor' selects the mount anchor of the robot body. 'position' and\n"
                   "'orientation' (Euler z,y,x in degrees) offset the sensor in the anchor\n"
                   "frame. 'rings' is the number of laser channels, spread evenly over\n"
                   "'vertical_fov' (min,max in degrees). 'horizontal_resolution' is the\n"
                   "azimuth step in degrees. 'framerate_divider' scans every n-th tick only.\n\n"
                   "'faces' and 'face_resolution' control the rendering rather than the ray\n"
                   "pattern, trading fidelity against cost. The faces must tile the azimuth\n"
                   "range exactly, so each spans 360/faces degrees horizontally and the\n"
                   "vertical field of view follows from the aspect ratio: taller faces see\n"
                   "more elevation. Initialization fails if the result cannot cover\n"
                   "'vertical_fov'. Faces sample the azimuth unevenly, most finely at their\n"
                   "centres, so a face_resolution width well above 360/faces divided by\n"
                   "'horizontal_resolution' keeps the ray pattern from undersampling.",
                   "Usable");

}
