#ifndef JOLT_DRONE_ODOMETRY_LOOP_FUNCTIONS_H
#define JOLT_DRONE_ODOMETRY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CDroneEntity;
}

using namespace argos;

/**
 * Checks that the drone takes off, flies at the XY velocity limit, stays
 * upright, and converges to a hover at the target position (same flight
 * profile as the drone_fly test), and additionally checks the drift-
 * injected odometry sensor added on top of it: early in the flight the
 * odometry estimate must still track ground truth closely (almost no
 * distance has been travelled yet, so almost no drift has accumulated),
 * while late in the flight it must have measurably, but boundedly,
 * diverged from ground truth, proving the per-tick drift both
 * accumulates and does not blow up or produce non-finite values.
 */
class CDroneOdometryLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CDroneEntity* m_pcDrone = nullptr;

};

#endif
