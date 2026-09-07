#ifndef JOLT_DRONE_ODOMETRY_LOOP_FUNCTIONS_H
#define JOLT_DRONE_ODOMETRY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>

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
 *
 * The odometry reading is START-RELATIVE, so comparing it against ground
 * truth means composing the drone's start pose onto it first. That is the
 * arithmetic every real consumer of this sensor has to do, and doing it here
 * keeps the test honest about the frame rather than measuring the spawn
 * offset and calling it drift.
 */
class CDroneOdometryLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CDroneEntity* m_pcDrone = nullptr;

   /* Where the drone started, captured on the first PostStep. The odometry
    * frame is pinned there, so this is what turns a reading into an arena
    * pose. */
   CVector3 m_cStartPosition;
   CQuaternion m_cStartOrientation;
   bool m_bHaveStart = false;

};

#endif
