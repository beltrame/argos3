#include "controller.h"

#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/utility/string_utilities.h>

#include <algorithm>
#include <fstream>

/****************************************/
/****************************************/

void CCameraTestController::Init(TConfigurationNode& t_tree) {
   m_pcCamera = GetSensor<CCI_PhotorealisticCameraSensor>("photorealistic_camera");
   GetNodeAttributeOrDefault(t_tree, "dump_ppm", m_strDumpPrefix, m_strDumpPrefix);
   GetNodeAttributeOrDefault(t_tree, "mount_depth_checks",
                             m_strMountDepthChecks, m_strMountDepthChecks);
   std::string strExpectedSize;
   GetNodeAttributeOrDefault(t_tree, "expected_size", strExpectedSize, strExpectedSize);
   if(!strExpectedSize.empty()) {
      UInt32 punSize[2];
      ParseValues<UInt32>(strExpectedSize, 2, punSize, ',');
      m_unExpectedWidth = punSize[0];
      m_unExpectedHeight = punSize[1];
   }
}

/****************************************/
/****************************************/

void CCameraTestController::ControlStep() {
   if(!m_pcCamera->HasNewFrame()) {
      return;
   }
   /* Multi-mount checks. ARGoS keys sensors by type, so several viewpoints
    * have to be mounts of one sensor rather than several sensor entries; this
    * verifies they are genuinely independent cameras and not aliases of the
    * same render. Each entry is "id:min,max" bounding that mount's minimum
    * depth. */
   if(!m_strMountDepthChecks.empty()) {
      std::vector<std::string> vecChecks;
      Tokenize(m_strMountDepthChecks, vecChecks, ";");
      if(vecChecks.size() != m_pcCamera->GetNumCameras()) {
         THROW_ARGOSEXCEPTION("Expected " << vecChecks.size() << " mounts, sensor has "
                              << m_pcCamera->GetNumCameras());
      }
      for(size_t i = 0; i < m_pcCamera->GetNumCameras(); ++i) {
         if(!m_pcCamera->HasNewFrame(i)) return;   /* wait for all mounts */
      }
      for(size_t i = 0; i < m_pcCamera->GetNumCameras(); ++i) {
         const CCI_PhotorealisticCameraSensor::SFrame& sM = m_pcCamera->GetFrame(i);
         if(sM.Depth.empty()) continue;
         const size_t unCentre = size_t(sM.Height / 2) * sM.Width + sM.Width / 2;
         LOG << "[MOUNT] " << sM.Id << " centre=" << sM.Depth[unCentre]
             << " min=" << *std::min_element(sM.Depth.begin(), sM.Depth.end())
             << std::endl;
      }
      LOG.Flush();
      for(const std::string& strCheck : vecChecks) {
         const size_t unColon = strCheck.find(':');
         const std::string strId = strCheck.substr(0, unColon);
         Real pfRange[2];
         ParseValues<Real>(strCheck.substr(unColon + 1), 2, pfRange, ',');
         bool bFound = false;
         for(size_t i = 0; i < m_pcCamera->GetNumCameras(); ++i) {
            const CCI_PhotorealisticCameraSensor::SFrame& sM = m_pcCamera->GetFrame(i);
            if(sM.Id != strId) continue;
            bFound = true;
            if(sM.Depth.empty()) {
               THROW_ARGOSEXCEPTION("Mount \"" << strId << "\" produced no depth");
            }
            /* Centre pixel, i.e. along this mount's optical axis. The global
             * minimum is the floor, which every mount sees regardless of where
             * it points, so it cannot tell the mounts apart. */
            const size_t unCentre = size_t(sM.Height / 2) * sM.Width + sM.Width / 2;
            const Real fCentre = sM.Depth[unCentre];
            if(fCentre < pfRange[0] || fCentre > pfRange[1]) {
               THROW_ARGOSEXCEPTION("Mount \"" << strId << "\" centre depth " << fCentre
                                    << " outside [" << pfRange[0] << ", "
                                    << pfRange[1] << "]");
            }
         }
         if(!bFound) {
            THROW_ARGOSEXCEPTION("No mount with id \"" << strId << "\"");
         }
      }
   }
   const CCI_PhotorealisticCameraSensor::SFrame& sFrame = m_pcCamera->GetFrame();
   size_t unPixels = size_t(sFrame.Width) * sFrame.Height;
   if(sFrame.Width != m_unExpectedWidth || sFrame.Height != m_unExpectedHeight) {
      THROW_ARGOSEXCEPTION("Unexpected frame size " << sFrame.Width << "x" << sFrame.Height);
   }
   if(sFrame.RGB.size() != unPixels * 3) {
      THROW_ARGOSEXCEPTION("Unexpected RGB buffer size " << sFrame.RGB.size());
   }
   if(sFrame.Depth.size() != unPixels) {
      THROW_ARGOSEXCEPTION("Unexpected depth buffer size " << sFrame.Depth.size());
   }
   if(sFrame.EntityId.size() != unPixels || sFrame.ClassId.size() != unPixels) {
      THROW_ARGOSEXCEPTION("Unexpected segmentation buffer size");
   }
   m_sLastFrame = sFrame;
   m_bHasFrame = true;
   ++m_unFrameCount;
   /* Overwritten every frame; the files keep the last one */
   if(!m_strDumpPrefix.empty()) {
      /* RGB as binary PPM */
      std::ofstream cRGBFile(m_strDumpPrefix + "_rgb.ppm", std::ios::binary);
      cRGBFile << "P6\n" << sFrame.Width << " " << sFrame.Height << "\n255\n";
      cRGBFile.write(reinterpret_cast<const char*>(sFrame.RGB.data()),
                     sFrame.RGB.size());
      /* Depth and entity ids as binary PGMs scaled to 8 bits */
      std::ofstream cDepthFile(m_strDumpPrefix + "_depth.pgm", std::ios::binary);
      cDepthFile << "P5\n" << sFrame.Width << " " << sFrame.Height << "\n255\n";
      for(size_t i = 0; i < unPixels; ++i) {
         Real fNormalized = std::min(sFrame.Depth[i] / 5.0, 1.0);
         cDepthFile.put(char(UInt8(fNormalized * 255.0)));
      }
      std::ofstream cSegFile(m_strDumpPrefix + "_seg.pgm", std::ios::binary);
      cSegFile << "P5\n" << sFrame.Width << " " << sFrame.Height << "\n255\n";
      for(size_t i = 0; i < unPixels; ++i) {
         cSegFile.put(char(UInt8(sFrame.EntityId[i] * 60 % 256)));
      }
   }
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CCameraTestController, "camera_test_controller");
