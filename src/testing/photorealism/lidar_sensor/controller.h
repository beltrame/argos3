#ifndef LIDAR_TEST_CONTROLLER_H
#define LIDAR_TEST_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>

#include <string>

using namespace argos;

/**
 * Checks a lidar scan against the geometry that produced it.
 *
 * The robot stands inside a closed rectangular room, which makes the
 * true range of every ray a closed-form ray/box intersection. The test
 * computes it for all of them, so a mis-oriented face, a gap between
 * faces, or a missing planar-to-radial depth conversion shows up as a
 * range error of metres rather than having to be inferred from a few
 * hand-picked directions.
 */
class CLidarTestController : public CCI_Controller {

public:

   CLidarTestController() {}
   virtual ~CLidarTestController() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy() {}

private:

   /** True range from the sensor to the room along c_direction, or the
    *  maximum range when the ray leaves through the open ceiling. */
   Real ExpectedRange(const CVector3& c_direction, Real f_max_range) const;

   CCI_PhotorealisticLidarSensor* m_pcLidar = nullptr;
   /** Room bounds: x in [0][1], y in [2][3], floor at 0, walls to [4] */
   Real m_fRoom[5] = {-4.0, 2.0, -5.0, 3.0, 3.0};
   /** Sensor origin in the arena frame */
   CVector3 m_cSensorPosition = CVector3(0.0, 0.0, 0.6);
   UInt32 m_unExpectedRings = 16;
   UInt32 m_unExpectedAzimuths = 1800;
   /** Every ray must land within this of its true range */
   Real m_fMaxError = 0.15;
   /** ... and the mean error must stay below this */
   Real m_fMeanError = 0.02;
   /** Lower bound on the mean range error. Zero disables the check; a
    *  positive value asserts that configured range noise is actually
    *  being applied, so an implementation that silently ignores
    *  range_noise_std_dev fails instead of passing quietly. */
   Real m_fMinMeanError = 0.0;

};

#endif
