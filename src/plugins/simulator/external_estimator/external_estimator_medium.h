/**
 * @file <argos3/plugins/simulator/external_estimator/external_estimator_medium.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef EXTERNAL_ESTIMATOR_MEDIUM_H
#define EXTERNAL_ESTIMATOR_MEDIUM_H

namespace argos {
   class CExternalEstimatorMedium;
   class CCI_IMUSensor;
   class CCI_PhotorealisticCameraSensor;
   class CCI_PhotorealisticLidarSensor;
   class CCI_DifferentialSteeringSensor;
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/rng.h>

#include <map>
#include <string>
#include <vector>

namespace argos {

   /**
    * Streams every robot's sensor data to a SLAM front-end running
    * outside ARGoS, and holds the pose estimates that come back for the
    * matching <odometry implementation="external"> sensors to report.
    *
    * ARGoS links no ROS, no Ceres and no OpenCV. The estimator lives
    * behind a Unix socket, in whatever container and ROS distribution it
    * needs, and this medium speaks a small binary protocol to it
    * (documented in Update()). That keeps the simulator free of the
    * estimator's dependency tree, and lets two estimators on two
    * incompatible ROS distributions run against the same simulation.
    *
    * This is a medium rather than a loop function on purpose: ARGoS
    * accepts exactly one <loop_functions>, so a bridge implemented there
    * would take the slot every real experiment needs for its own. Media
    * are a list, and are already how sensors find shared machinery.
    *
    * The medium is controller-agnostic: it resolves sensors through
    * CCI_Controller::GetSensor(), so any controller that declares the
    * sensors works, with no code shared between the two.
    *
    * TIMING
    * ======
    * Media are updated after the physics and before the sense phase
    * (see CSpace::Update()), so the readings this medium can see are the
    * ones the sensors produced during the previous tick's sense phase.
    * The frame it sends is therefore stamped with that tick, not the
    * current one, and the estimates that come back are picked up by the
    * odometry sensors during this tick's sense phase. One tick of
    * round-trip latency is the floor; a real estimator adds its own.
    */
   class CExternalEstimatorMedium : public CMedium {

   public:

      /** One pose estimate, as produced by the external estimator */
      struct SEstimate {
         CVector3 Position;
         CQuaternion Orientation;
         /** Body-frame twist; zero when the estimator reports none */
         CVector3 LinearVelocity;
         CVector3 AngularVelocity;
         /** The simulation tick this estimate refers to */
         UInt32 Tick = 0;
         /** False until the estimator has converged on a pose */
         bool Valid = false;
      };

   public:

      CExternalEstimatorMedium();

      virtual ~CExternalEstimatorMedium() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void PostSpaceInit();

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      /**
       * Copies the newest estimate for a robot into s_estimate. Returns
       * false when the estimator has never reported one, in which case
       * s_estimate is left untouched. Called by the odometry sensors.
       */
      bool GetEstimate(const std::string& str_robot_id,
                       SEstimate& s_estimate) const;

   public:

      /**
       * How a robot's estimator frame is placed in the shared frame.
       *
       * An estimator starts at identity in a frame of its own, so two
       * robots' poses are not comparable until something relates them.
       * Multi-robot back ends quietly assume they already are:
       * Swarm-SLAM initialises its joint optimisation by inserting each
       * robot's odometry values with NO relative transform applied
       * (decentralized_pgo.cpp, aggregate_pose_graphs), so per-robot
       * frames stack every trajectory at the origin while the truth has
       * them tens of metres apart, and the optimiser lands in a bad
       * local minimum.
       */
      enum class EAlignment {
         /** Report the estimator frame untouched. Honest about what the
          *  estimator knows, and what a multi-robot back end cannot use. */
         None,
         /** Place every robot using ground truth. Not realistic -- no
          *  robot is handed its own global pose -- but it is the upper
          *  bound: whatever a back end cannot achieve with this is not
          *  an alignment problem. */
         GroundTruth,
         /** Each robot places itself the first time it sees a surveyed
          *  fiducial marker, from a NOISY measurement of the marker's
          *  pose relative to its body. This is what a real deployment
          *  does, and the resulting inter-robot alignment error is real
          *  rather than assumed away. Until a robot has seen one, it has
          *  no shared frame and reports nothing. */
         Fiducial
      };

   private:

      /** A surveyed marker at a known pose in the world */
      struct SFiducial {
         std::string Id;
         CVector3 Position;
         CQuaternion Orientation;
      };

      /** One robot's sensors, resolved once after the space is built */
      struct SRobot {
         std::string Id;
         /** Required: the estimator needs inertial data */
         CCI_IMUSensor* IMU = nullptr;
         /** Optional channels; null when the robot does not carry them */
         CCI_PhotorealisticCameraSensor* Camera = nullptr;
         CCI_PhotorealisticLidarSensor* Lidar = nullptr;
         CCI_DifferentialSteeringSensor* Wheels = nullptr;
         /** Wheel-encoder dead reckoning, integrated by this medium (see
          *  IntegrateWheels). Planar: a differential drive observes
          *  neither height nor attitude. */
         CVector3 WheelPosition;
         CRadians WheelYaw;
         /** Ground truth, for visibility tests and alignment */
         CEmbodiedEntity* Body = nullptr;
         /** Estimator frame expressed in the shared frame, solved once
          *  when this robot first places itself */
         bool Aligned = false;
         CVector3 FramePosition;
         CQuaternion FrameOrientation;
      };

   private:

      void Connect();
      void Disconnect();

      /** Resolves one robot's sensors; throws if the IMU is missing */
      SRobot ResolveRobot(const std::string& str_id);

      /**
       * Advances a robot's wheel-encoder dead reckoning by one tick,
       * along the exact arc a differential drive traces.
       */
      void IntegrateWheels(SRobot& s_robot);

      /** Serialization helpers: append to m_vecOut */
      void Append(const void* pt_data, size_t un_size);
      template<typename T> void AppendPod(const T& t_value) {
         Append(&t_value, sizeof(t_value));
      }
      void AppendId(const std::string& str_id);

      void SerializeRobot(SRobot& s_robot);
      void SerializeCamera(SRobot& s_robot);
      void SerializeLidar(SRobot& s_robot);

      /** Socket helpers; both throw when the far side goes away */
      void SendAll(const void* pt_data, size_t un_size);
      void RecvAll(void* pt_data, size_t un_size);

      /** Reads the reply frame and files every pose it carries */
      void ReceiveEstimates();

      /**
       * Places s_robot's estimator frame in the shared frame if it can,
       * and maps s_estimate into that frame. Returns false while the
       * robot still has no shared frame, in which case it reports
       * nothing at all.
       */
      bool AlignEstimate(SRobot& s_robot, SEstimate& s_estimate);

      /** True when a fiducial is close enough and far enough forward to
       *  be seen from the robot's current pose */
      bool SeesFiducial(const SRobot& s_robot, const SFiducial& s_fiducial) const;

   private:

      std::vector<SRobot> m_vecRobots;

      /** Explicit robot list from the XML; empty means "every robot" */
      std::vector<std::string> m_vecRequestedRobots;

      std::map<std::string, SEstimate> m_mapEstimates;

      std::string m_strSocketPath;
      int m_nSocket;

      /** Seconds to wait for the far side to accept a connection */
      Real m_fConnectTimeout;
      /** Seconds to wait for a reply before declaring the far side dead */
      Real m_fTimeout;

      /** When true, the far side must hold its reply until it has a pose
       *  for the tick being sent; when false it replies with whatever is
       *  ready and the simulation moves on */
      bool m_bLockstepPose;

      UInt32 m_unTicksPerSecond;

      /** Reusable send buffer, so a tick costs no allocation */
      std::vector<UInt8> m_vecOut;

      /** Which streams are actually sent. Publishing a channel the
       *  estimator does not read is not free: a VLP-16 revolution is
       *  ~630 KB, and four robots' worth of unread cloud is enough to
       *  overrun the DDS buffers on the far side and starve the
       *  channels that ARE read. */
      bool m_bSendCamera;
      bool m_bSendLidar;
      bool m_bSendWheels;

      EAlignment m_eAlignment;
      std::vector<SFiducial> m_vecFiducials;
      /** How far, and how far off-axis, a marker can be read */
      Real m_fFiducialRange;
      CRadians m_cFiducialHalfFov;
      /** Measurement noise on the marker pose, metres and radians */
      Real m_fFiducialPositionNoise;
      CRadians m_cFiducialYawNoise;
      CRandom::CRNG* m_pcRNG;

   };

}

#endif
