#ifndef PHOTOREALISM_CAMERA_TEST_CONTROLLER_H
#define PHOTOREALISM_CAMERA_TEST_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.h>

using namespace argos;

/**
 * Reads the photorealistic camera every step, validates the frame
 * layout, and keeps the latest frame for the loop functions to check.
 */
class CCameraTestController : public CCI_Controller {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void ControlStep();

public:

   CCI_PhotorealisticCameraSensor* m_pcCamera = nullptr;
   CCI_PhotorealisticCameraSensor::SFrame m_sLastFrame;
   bool m_bHasFrame = false;
   /* Number of fresh frames received over the experiment */
   UInt32 m_unFrameCount = 0;
   /* Expected frame size (XML params attribute "expected_size") */
   UInt32 m_unExpectedWidth = 64;
   UInt32 m_unExpectedHeight = 64;
   /* When set (XML params attribute "dump_ppm"), the first frame is
    * written to <prefix>_rgb.ppm / _depth.pgm / _seg.pgm */
   std::string m_strDumpPrefix;
   /** Per-mount expectations, "id:min,max" of the minimum depth,
    *  separated by ';'. Empty disables the multi-mount check. */
   std::string m_strMountDepthChecks;
   bool m_bDumped = false;

};

#endif
