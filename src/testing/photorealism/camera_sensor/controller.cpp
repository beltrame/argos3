#include "controller.h"

#include <argos3/core/utility/string_utilities.h>

#include <algorithm>
#include <fstream>

/****************************************/
/****************************************/

void CCameraTestController::Init(TConfigurationNode& t_tree) {
   m_pcCamera = GetSensor<CCI_PhotorealisticCameraSensor>("photorealistic_camera");
   GetNodeAttributeOrDefault(t_tree, "dump_ppm", m_strDumpPrefix, m_strDumpPrefix);
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
