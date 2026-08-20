/**
 * @file <argos3/plugins/simulator/external_estimator/external_estimator_medium.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "external_estimator_medium.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

namespace argos {

   /* Frame markers, so a protocol mismatch fails loudly instead of being
    * read as garbage numbers */
   static const char PROTOCOL_MAGIC[4] = {'A', 'E', 'B', 'R'};
   static const char PROTOCOL_ACK[4]   = {'A', 'C', 'K', '\0'};

   /* The differential steering sensor reports centimetres; everything
    * else in ARGoS, and everything in ROS, is metres */
   static const Real CM_TO_M = 0.01;

   /* Simulated returns carry no reflectance. A constant keeps the
    * intensity channel well-formed without inventing a radiometry the
    * renderer never computed. */
   static const float LIDAR_INTENSITY = 100.0f;

   /****************************************/
   /****************************************/

   CExternalEstimatorMedium::CExternalEstimatorMedium() :
      m_nSocket(-1),
      m_fConnectTimeout(120.0),
      m_fTimeout(120.0),
      m_bLockstepPose(false),
      m_unTicksPerSecond(0),
      m_bSendCamera(true),
      m_bSendLidar(true),
      m_bSendWheels(true),
      m_eAlignment(EAlignment::None),
      m_fFiducialRange(6.0),
      m_cFiducialHalfFov(ToRadians(CDegrees(30.0))),
      m_fFiducialPositionNoise(0.05),
      m_cFiducialYawNoise(ToRadians(CDegrees(2.0))),
      m_pcRNG(nullptr) {}

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         GetNodeAttribute(t_tree, "socket", m_strSocketPath);
         GetNodeAttributeOrDefault(t_tree, "connect_timeout",
                                   m_fConnectTimeout, m_fConnectTimeout);
         GetNodeAttributeOrDefault(t_tree, "timeout", m_fTimeout, m_fTimeout);
         GetNodeAttributeOrDefault(t_tree, "lockstep_pose",
                                   m_bLockstepPose, m_bLockstepPose);
         /* Robots: an explicit list, or (when absent) every robot */
         std::string strRobots;
         GetNodeAttributeOrDefault(t_tree, "robots", strRobots, strRobots);
         std::istringstream cRobots(strRobots);
         std::string strRobot;
         while(std::getline(cRobots, strRobot, ',')) {
            /* Tolerate "r0, r1, r2" as well as "r0,r1,r2" */
            size_t unFirst = strRobot.find_first_not_of(" \t");
            if(unFirst == std::string::npos) continue;
            size_t unLast = strRobot.find_last_not_of(" \t");
            m_vecRequestedRobots.push_back(
               strRobot.substr(unFirst, unLast - unFirst + 1));
         }
         /* Channels. Default is everything the robots carry. */
         std::string strChannels;
         GetNodeAttributeOrDefault(t_tree, "channels", strChannels, strChannels);
         if(!strChannels.empty()) {
            m_bSendCamera = m_bSendLidar = m_bSendWheels = false;
            std::istringstream cChannels(strChannels);
            std::string strChannel;
            while(std::getline(cChannels, strChannel, ',')) {
               size_t unFirst = strChannel.find_first_not_of(" \t");
               if(unFirst == std::string::npos) continue;
               size_t unLast = strChannel.find_last_not_of(" \t");
               strChannel = strChannel.substr(unFirst, unLast - unFirst + 1);
               if(strChannel == "camera")      m_bSendCamera = true;
               else if(strChannel == "lidar")  m_bSendLidar = true;
               else if(strChannel == "wheels") m_bSendWheels = true;
               else if(strChannel == "imu")    { /* always sent */ }
               else {
                  THROW_ARGOSEXCEPTION("Unknown channel \"" << strChannel
                                       << "\"; expected imu, camera, lidar "
                                       "or wheels");
               }
            }
         }
         /* Frame alignment */
         std::string strAlignment("none");
         GetNodeAttributeOrDefault(t_tree, "alignment", strAlignment, strAlignment);
         if(strAlignment == "none") {
            m_eAlignment = EAlignment::None;
         }
         else if(strAlignment == "ground_truth") {
            m_eAlignment = EAlignment::GroundTruth;
         }
         else if(strAlignment == "fiducial") {
            m_eAlignment = EAlignment::Fiducial;
         }
         else {
            THROW_ARGOSEXCEPTION("Unknown alignment \"" << strAlignment
                                 << "\"; expected none, ground_truth or fiducial");
         }
         if(m_eAlignment == EAlignment::Fiducial) {
            GetNodeAttributeOrDefault(t_tree, "fiducial_range",
                                      m_fFiducialRange, m_fFiducialRange);
            CDegrees cFov(60.0);
            GetNodeAttributeOrDefault(t_tree, "fiducial_fov", cFov, cFov);
            m_cFiducialHalfFov = ToRadians(cFov) * 0.5;
            GetNodeAttributeOrDefault(t_tree, "fiducial_position_noise",
                                      m_fFiducialPositionNoise,
                                      m_fFiducialPositionNoise);
            CDegrees cYawNoise(2.0);
            GetNodeAttributeOrDefault(t_tree, "fiducial_yaw_noise",
                                      cYawNoise, cYawNoise);
            m_cFiducialYawNoise = ToRadians(cYawNoise);
            if(NodeExists(t_tree, "fiducial")) {
               TConfigurationNodeIterator itFiducial("fiducial");
               for(itFiducial = itFiducial.begin(&t_tree);
                   itFiducial != itFiducial.end(); ++itFiducial) {
                  SFiducial sFiducial;
                  GetNodeAttribute(*itFiducial, "id", sFiducial.Id);
                  GetNodeAttribute(*itFiducial, "position", sFiducial.Position);
                  CVector3 cEuler;
                  GetNodeAttributeOrDefault(*itFiducial, "orientation",
                                            cEuler, cEuler);
                  sFiducial.Orientation.FromEulerAngles(
                     ToRadians(CDegrees(cEuler.GetX())),
                     ToRadians(CDegrees(cEuler.GetY())),
                     ToRadians(CDegrees(cEuler.GetZ())));
                  m_vecFiducials.push_back(sFiducial);
               }
            }
            if(m_vecFiducials.empty()) {
               THROW_ARGOSEXCEPTION("alignment=\"fiducial\" needs at least one "
                                    "<fiducial id=\"...\" position=\"x,y,z\" /> "
                                    "child; without one no robot can ever place "
                                    "itself and nothing is reported");
            }
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         m_unTicksPerSecond =
            UInt32(std::lround(CPhysicsEngine::GetInverseSimulationClockTick()));
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in the external "
                                     "estimator medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::PostSpaceInit() {
      /* Robots only exist once the space is built, which is why the
       * sensors are resolved here rather than in Init() */
      try {
         if(m_vecRequestedRobots.empty()) {
            CSpace::TMapPerType& mapControllers =
               GetSpace().GetEntitiesByType("controller");
            for(const auto& tEntity : mapControllers) {
               auto* pcControllable = any_cast<CControllableEntity*>(tEntity.second);
               m_vecRobots.push_back(
                  ResolveRobot(pcControllable->GetParent().GetId()));
            }
         }
         else {
            for(const std::string& strId : m_vecRequestedRobots) {
               m_vecRobots.push_back(ResolveRobot(strId));
            }
         }
         if(m_vecRobots.empty()) {
            THROW_ARGOSEXCEPTION("No robots to stream: the \"robots\" list is "
                                 "empty and the arena contains no controllable "
                                 "entity");
         }
         Connect();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error starting the external estimator "
                                     "medium \"" << GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   CExternalEstimatorMedium::SRobot
   CExternalEstimatorMedium::ResolveRobot(const std::string& str_id) {
      SRobot sRobot;
      sRobot.Id = str_id;
      auto& cEntity = dynamic_cast<CComposableEntity&>(GetSpace().GetEntity(str_id));
      sRobot.Body = &cEntity.GetComponent<CEmbodiedEntity>("body");
      auto& cControllable = cEntity.GetComponent<CControllableEntity>("controller");
      CCI_Controller& cController = cControllable.GetController();
      /* The estimator is inertial at its core: without an IMU there is
       * nothing to propagate between frames */
      if(!cController.HasSensor("imu")) {
         THROW_ARGOSEXCEPTION("Robot \"" << str_id << "\" has no \"imu\" sensor. "
                              "The external estimator needs one on every robot "
                              "it streams.");
      }
      sRobot.IMU = cController.GetSensor<CCI_IMUSensor>("imu");
      /* Everything else is optional: which channels a robot carries is
       * what decides the estimator's fusion mode */
      if(cController.HasSensor("photorealistic_camera")) {
         sRobot.Camera = cController.GetSensor<CCI_PhotorealisticCameraSensor>(
            "photorealistic_camera");
      }
      if(cController.HasSensor("photorealistic_lidar")) {
         sRobot.Lidar = cController.GetSensor<CCI_PhotorealisticLidarSensor>(
            "photorealistic_lidar");
      }
      if(cController.HasSensor("differential_steering")) {
         sRobot.Wheels = cController.GetSensor<CCI_DifferentialSteeringSensor>(
            "differential_steering");
      }
      LOG << "[EXTERNAL_ESTIMATOR] " << GetId() << ": " << str_id << " -> imu"
          << (sRobot.Camera != nullptr ? " camera" : "")
          << (sRobot.Lidar  != nullptr ? " lidar"  : "")
          << (sRobot.Wheels != nullptr ? " wheels" : "")
          << std::endl;
      return sRobot;
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::Connect() {
      m_nSocket = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if(m_nSocket < 0) {
         THROW_ARGOSEXCEPTION("Cannot create the estimator socket: "
                              << ::strerror(errno));
      }
      struct sockaddr_un tAddress;
      std::memset(&tAddress, 0, sizeof(tAddress));
      tAddress.sun_family = AF_UNIX;
      if(m_strSocketPath.size() >= sizeof(tAddress.sun_path)) {
         THROW_ARGOSEXCEPTION("Socket path \"" << m_strSocketPath
                              << "\" is too long");
      }
      std::strncpy(tAddress.sun_path, m_strSocketPath.c_str(),
                   sizeof(tAddress.sun_path) - 1);
      /* The far side usually lives in a container that takes a while to
       * load its models, so retry rather than fail on the first refusal */
      LOG << "[EXTERNAL_ESTIMATOR] Waiting for the estimator on "
          << m_strSocketPath << " ..." << std::endl;
      LOG.Flush();
      Real fWaited = 0.0;
      while(::connect(m_nSocket, reinterpret_cast<struct sockaddr*>(&tAddress),
                      sizeof(tAddress)) < 0) {
         if(fWaited >= m_fConnectTimeout) {
            THROW_ARGOSEXCEPTION("Could not connect to the external estimator at \""
                                 << m_strSocketPath << "\" after "
                                 << m_fConnectTimeout << " s: " << ::strerror(errno)
                                 << ". Is the estimator sidecar running?");
         }
         ::usleep(200000);
         fWaited += 0.2;
      }
      /* Bound the reply wait, so a crashed or wedged estimator stops the
       * run instead of hanging it forever */
      struct timeval tTimeout;
      tTimeout.tv_sec = time_t(m_fTimeout);
      tTimeout.tv_usec = 0;
      ::setsockopt(m_nSocket, SOL_SOCKET, SO_RCVTIMEO, &tTimeout, sizeof(tTimeout));
      LOG << "[EXTERNAL_ESTIMATOR] Connected; streaming " << m_vecRobots.size()
          << " robot(s), lockstep_pose="
          << (m_bLockstepPose ? "true" : "false") << std::endl;
      LOG.Flush();
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::Disconnect() {
      if(m_nSocket >= 0) {
         /* Tell the far side the run is over so it can flush its logs */
         ::shutdown(m_nSocket, SHUT_WR);
         ::close(m_nSocket);
         m_nSocket = -1;
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::Reset() {
      m_mapEstimates.clear();
      for(SRobot& sRobot : m_vecRobots) {
         sRobot.WheelPosition = CVector3::ZERO;
         sRobot.WheelYaw = CRadians::ZERO;
         sRobot.Aligned = false;
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::Destroy() {
      Disconnect();
   }

   /****************************************/
   /****************************************/

   bool CExternalEstimatorMedium::SeesFiducial(const SRobot& s_robot,
                                               const SFiducial& s_fiducial) const {
      const CVector3& cRobotPos = s_robot.Body->GetOriginAnchor().Position;
      const CQuaternion& cRobotOri = s_robot.Body->GetOriginAnchor().Orientation;
      CVector3 cToMarker = s_fiducial.Position - cRobotPos;
      if(cToMarker.Length() > m_fFiducialRange) {
         return false;
      }
      /* Bearing in the robot's own frame: a marker behind the robot is
       * not readable however close it is */
      cToMarker.Rotate(cRobotOri.Inverse());
      CRadians cBearing = ATan2(cToMarker.GetY(), cToMarker.GetX());
      return Abs(cBearing) <= m_cFiducialHalfFov;
   }

   /****************************************/
   /****************************************/

   bool CExternalEstimatorMedium::AlignEstimate(SRobot& s_robot,
                                                SEstimate& s_estimate) {
      if(m_eAlignment == EAlignment::None) {
         return true;
      }
      if(!s_robot.Aligned) {
         /*
          * Solve the estimator frame's pose in the shared frame, once:
          *
          *    T_shared_est = T_shared_body * T_est^-1
          *
          * Anchoring on the FIRST ESTIMATE rather than tick 0 matters:
          * an estimator initializes some way into the run and its origin
          * is wherever the robot had got to by then.
          */
         CVector3 cBodyPos;
         CQuaternion cBodyOri;
         if(m_eAlignment == EAlignment::GroundTruth) {
            cBodyPos = s_robot.Body->GetOriginAnchor().Position;
            cBodyOri = s_robot.Body->GetOriginAnchor().Orientation;
         }
         else {
            /* Fiducial: the robot must actually be looking at a surveyed
             * marker. It measures the marker's pose relative to itself,
             * with error, and infers its own pose from the marker's
             * known one:
             *
             *    T_world_body = T_world_marker * (T_body_marker)^-1
             *
             * so the marker's survey is trusted and the MEASUREMENT is
             * what carries the noise. Every robot that aligns off the
             * same marker inherits an independent error, which is
             * exactly the inter-robot misalignment a real deployment
             * has and a back end has to absorb. */
            const SFiducial* psSeen = nullptr;
            for(const SFiducial& sFiducial : m_vecFiducials) {
               if(SeesFiducial(s_robot, sFiducial)) {
                  psSeen = &sFiducial;
                  break;
               }
            }
            if(psSeen == nullptr) {
               /* No shared frame yet: this robot cannot place itself, so
                * it reports nothing at all rather than something in a
                * frame nobody else can use. */
               return false;
            }
            const CVector3& cTruePos = s_robot.Body->GetOriginAnchor().Position;
            const CQuaternion& cTrueOri = s_robot.Body->GetOriginAnchor().Orientation;
            /* True marker pose in the body frame */
            CVector3 cMarkerInBody = psSeen->Position - cTruePos;
            cMarkerInBody.Rotate(cTrueOri.Inverse());
            CQuaternion cMarkerOriInBody = cTrueOri.Inverse() * psSeen->Orientation;
            /* Perturb the MEASUREMENT */
            cMarkerInBody += CVector3(m_pcRNG->Gaussian(m_fFiducialPositionNoise),
                                      m_pcRNG->Gaussian(m_fFiducialPositionNoise),
                                      m_pcRNG->Gaussian(m_fFiducialPositionNoise));
            CQuaternion cYawError;
            cYawError.FromAngleAxis(
               CRadians(m_pcRNG->Gaussian(m_cFiducialYawNoise.GetValue())),
               CVector3::Z);
            cMarkerOriInBody = cYawError * cMarkerOriInBody;
            /* Invert to get the body in the marker frame, then compose
             * with the marker's surveyed world pose */
            CQuaternion cBodyInMarker = cMarkerOriInBody.Inverse();
            CVector3 cBodyPosInMarker = -cMarkerInBody;
            cBodyPosInMarker.Rotate(cBodyInMarker);
            cBodyOri = psSeen->Orientation * cBodyInMarker;
            CVector3 cOffset = cBodyPosInMarker;
            cOffset.Rotate(psSeen->Orientation);
            cBodyPos = psSeen->Position + cOffset;
            LOG << "[EXTERNAL_ESTIMATOR] " << GetId() << ": " << s_robot.Id
                << " aligned on fiducial \"" << psSeen->Id << "\" at tick "
                << GetSpace().GetSimulationClock() << std::endl;
            LOG.Flush();
         }
         s_robot.FrameOrientation = cBodyOri * s_estimate.Orientation.Inverse();
         CVector3 cRotated = s_estimate.Position;
         cRotated.Rotate(s_robot.FrameOrientation);
         s_robot.FramePosition = cBodyPos - cRotated;
         s_robot.Aligned = true;
      }
      CVector3 cPosition = s_estimate.Position;
      cPosition.Rotate(s_robot.FrameOrientation);
      s_estimate.Position = s_robot.FramePosition + cPosition;
      s_estimate.Orientation = s_robot.FrameOrientation * s_estimate.Orientation;
      /* The twist is body-frame, so it is frame independent */
      return true;
   }

   /****************************************/
   /****************************************/

   bool CExternalEstimatorMedium::GetEstimate(const std::string& str_robot_id,
                                              SEstimate& s_estimate) const {
      auto itEstimate = m_mapEstimates.find(str_robot_id);
      if(itEstimate == m_mapEstimates.end()) {
         return false;
      }
      s_estimate = itEstimate->second;
      return true;
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::Append(const void* pt_data, size_t un_size) {
      const auto* punData = static_cast<const UInt8*>(pt_data);
      m_vecOut.insert(m_vecOut.end(), punData, punData + un_size);
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::AppendId(const std::string& str_id) {
      if(str_id.size() > 255) {
         THROW_ARGOSEXCEPTION("Robot id \"" << str_id << "\" is longer than the "
                              "255 bytes the protocol allows");
      }
      AppendPod(UInt8(str_id.size()));
      Append(str_id.data(), str_id.size());
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::IntegrateWheels(SRobot& s_robot) {
      const CCI_DifferentialSteeringSensor::SReading& sWheels =
         s_robot.Wheels->GetReading();
      /* The sensor reports the distance covered during one tick, in
       * centimetres, derived from the commanded wheel velocities.
       * Reading the command rather than the achieved motion is what
       * makes this a real encoder: when a wheel spins against an
       * obstacle the encoder still counts, and the dead-reckoned pose
       * slips away from truth exactly as it would on hardware. */
      Real fLeft  = sWheels.CoveredDistanceLeftWheel  * CM_TO_M;
      Real fRight = sWheels.CoveredDistanceRightWheel * CM_TO_M;
      Real fAxis  = sWheels.WheelAxisLength * CM_TO_M;
      if(fAxis <= 0.0) {
         return;
      }
      Real fForward = 0.5 * (fRight + fLeft);
      CRadians cDeltaYaw((fRight - fLeft) / fAxis);
      /* Exact arc: integrate about the midpoint heading, which is what a
       * differential drive actually traces over the tick */
      CRadians cMidYaw = s_robot.WheelYaw + 0.5 * cDeltaYaw;
      s_robot.WheelPosition += CVector3(fForward * Cos(cMidYaw),
                                        fForward * Sin(cMidYaw),
                                        0.0);
      s_robot.WheelYaw += cDeltaYaw;
      s_robot.WheelYaw.SignedNormalize();
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::SerializeCamera(SRobot& s_robot) {
      bool bHasFrame = m_bSendCamera && s_robot.Camera != nullptr &&
                       s_robot.Camera->HasNewFrame();
      AppendPod(UInt8(bHasFrame ? 1 : 0));
      if(!bHasFrame) {
         return;
      }
      const CCI_PhotorealisticCameraSensor::SFrame& sFrame =
         s_robot.Camera->GetFrame();
      AppendPod(UInt32(sFrame.Width));
      AppendPod(UInt32(sFrame.Height));
      /* Vertical field of view in degrees. The far side derives the
       * pinhole intrinsics from it and must divide the HEIGHT by it:
       * fy = h / (2 tan(fov/2)), fx = fy. */
      AppendPod(float(sFrame.FieldOfView));
      Append(sFrame.RGB.data(), sFrame.RGB.size());
      UInt8 unHasDepth = sFrame.Depth.empty() ? 0 : 1;
      AppendPod(unHasDepth);
      if(unHasDepth != 0) {
         /* Depth is Real (double) inside ARGoS; ROS wants 32FC1 */
         std::vector<float> vecDepth(sFrame.Depth.begin(), sFrame.Depth.end());
         Append(vecDepth.data(), vecDepth.size() * sizeof(float));
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::SerializeLidar(SRobot& s_robot) {
      bool bHasScan = m_bSendLidar && s_robot.Lidar != nullptr &&
                      s_robot.Lidar->HasNewScan();
      AppendPod(UInt8(bHasScan ? 1 : 0));
      if(!bHasScan) {
         return;
      }
      const CCI_PhotorealisticLidarSensor::SScan& sScan = s_robot.Lidar->GetScan();
      /* Count the returns first: misses are dropped, exactly as a real
       * lidar reports nothing for a ray that hit nothing */
      UInt32 unPoints = 0;
      for(const auto& sReading : sScan.Readings) {
         if(sReading.Hit) ++unPoints;
      }
      AppendPod(unPoints);
      if(unPoints == 0) {
         return;
      }
      /* An ARGoS scan is a snapshot: every ray depicts the same tick,
       * whereas a real spinning lidar smears one revolution. Estimators
       * still want a monotonic per-point time to motion-compensate
       * against, so one is synthesized from the azimuth below.
       *
       * It deliberately spans ONE TICK, not one revolution. The cloud
       * carries no skew, so whatever interval we claim is an interval
       * the estimator will de-skew across, injecting an error of
       * (robot speed x interval). The tick is both the truthful answer
       * -- it is the interval the snapshot depicts -- and the smaller
       * one: at a foot-bot's 0.7 m/s a 10 ms tick costs ~7 mm, where
       * the 100 ms revolution of a 10 Hz lidar would cost ~7 cm.
       *
       * The consequence is that the sweep does not tile the time
       * between scans, as a real spinning lidar's would. If an
       * estimator ever objects to that, this is the knob. */
      Real fSweep = CPhysicsEngine::GetSimulationClockTick();
      double fNanosPerAzimuth = sScan.NumAzimuths > 0
         ? 1.0e9 * double(fSweep) / double(sScan.NumAzimuths)
         : 0.0;
      UInt32 unRings = sScan.NumRings > 0 ? sScan.NumRings : 1;
      for(size_t i = 0; i < sScan.Readings.size(); ++i) {
         const auto& sReading = sScan.Readings[i];
         if(!sReading.Hit) continue;
         /* Readings are azimuth-major: index = azimuth * rings + ring */
         double fOffsetNanos = double(i / unRings) * fNanosPerAzimuth;
         /* The sensor frame is +x forward, +y left, +z up, the same
          * convention ROS uses, so the endpoint needs no rotation */
         AppendPod(float(sReading.Position.GetX()));
         AppendPod(float(sReading.Position.GetY()));
         AppendPod(float(sReading.Position.GetZ()));
         AppendPod(LIDAR_INTENSITY);
         AppendPod(fOffsetNanos);
         AppendPod(UInt8(0));                    /* Livox tag: normal return */
         AppendPod(UInt8(sReading.Ring & 0xFF)); /* scan line */
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::SerializeRobot(SRobot& s_robot) {
      AppendId(s_robot.Id);
      SerializeCamera(s_robot);
      SerializeLidar(s_robot);
      /* Wheel-encoder dead reckoning: planar, since a differential drive
       * observes neither height nor attitude */
      UInt8 unHasWheels = (m_bSendWheels && s_robot.Wheels != nullptr) ? 1 : 0;
      AppendPod(unHasWheels);
      if(unHasWheels != 0) {
         IntegrateWheels(s_robot);
         CQuaternion cWheelOrientation(s_robot.WheelYaw, CVector3::Z);
         double pfWheel[7] = {
            double(s_robot.WheelPosition.GetX()),
            double(s_robot.WheelPosition.GetY()),
            double(s_robot.WheelPosition.GetZ()),
            double(cWheelOrientation.GetW()), double(cWheelOrientation.GetX()),
            double(cWheelOrientation.GetY()), double(cWheelOrientation.GetZ())};
         Append(pfWheel, sizeof(pfWheel));
      }
      const CCI_IMUSensor::SReading& sImu = s_robot.IMU->GetReading();
      double pfImu[6] = {
         double(sImu.AngularVelocity.GetX()),
         double(sImu.AngularVelocity.GetY()),
         double(sImu.AngularVelocity.GetZ()),
         double(sImu.LinearAcceleration.GetX()),
         double(sImu.LinearAcceleration.GetY()),
         double(sImu.LinearAcceleration.GetZ())};
      Append(pfImu, sizeof(pfImu));
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::Update() {
      /*
       * One frame per tick, little-endian throughout:
       *
       *   "AEBR", u32 tick, u32 ticks_per_second, u8 lockstep,
       *   u32 robot_count
       *   per robot:
       *     u8 id_len, id bytes
       *     u8 has_frame; if set: u32 w, u32 h, f32 vertical_fov_deg,
       *                           u8 rgb[w*h*3],
       *                           u8 has_depth, if set f32 depth[w*h]
       *     u8 has_scan;  if set: u32 n_points, then per point
       *                           f32 x, f32 y, f32 z, f32 intensity,
       *                           f64 offset_ns, u8 tag, u8 line
       *     u8 has_wheels; if set: f64 wheel_pose[7]  (x y z qw qx qy qz)
       *     f64 imu[6]                                (wx wy wz ax ay az)
       *
       * The reply is described in ReceiveEstimates().
       *
       * The tick sent is the one the READINGS belong to, which is the
       * previous one: media update before the sense phase, so the newest
       * readings available here are the ones produced last tick. Tick 1
       * carries nothing yet, so it is skipped entirely.
       */
      UInt32 unClock = UInt32(GetSpace().GetSimulationClock());
      if(unClock < 2) {
         return;
      }
      m_vecOut.clear();
      Append(PROTOCOL_MAGIC, sizeof(PROTOCOL_MAGIC));
      AppendPod(UInt32(unClock - 1));
      AppendPod(m_unTicksPerSecond);
      AppendPod(UInt8(m_bLockstepPose ? 1 : 0));
      AppendPod(UInt32(m_vecRobots.size()));
      for(SRobot& sRobot : m_vecRobots) {
         SerializeRobot(sRobot);
      }
      SendAll(m_vecOut.data(), m_vecOut.size());
      /* Blocks here: this is what keeps the simulation from running
       * ahead of the estimator and dropping data on the floor */
      ReceiveEstimates();
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::ReceiveEstimates() {
      /*
       *   "ACK\0", u32 pose_count
       *   per pose:
       *     u8 id_len, id bytes, u32 tick,
       *     f64 pose[7]   (x y z qw qx qy qz)
       *     f64 twist[6]  (vx vy vz wx wy wz, body frame)
       *     u8 valid
       *
       * When the request set the lockstep flag, the far side is the one
       * that waits: it holds the reply back until the estimator has a
       * pose for that tick. ARGoS only bounds the wait with its receive
       * timeout, which keeps the waiting in the process that already
       * runs the estimator's event loop.
       */
      char pchAck[4];
      RecvAll(pchAck, sizeof(pchAck));
      if(std::memcmp(pchAck, PROTOCOL_ACK, sizeof(PROTOCOL_ACK)) != 0) {
         THROW_ARGOSEXCEPTION("Unexpected reply from the external estimator: "
                              "protocol mismatch");
      }
      UInt32 unPoses = 0;
      RecvAll(&unPoses, sizeof(unPoses));
      for(UInt32 i = 0; i < unPoses; ++i) {
         UInt8 unIdLength = 0;
         RecvAll(&unIdLength, sizeof(unIdLength));
         std::string strId(unIdLength, '\0');
         if(unIdLength > 0) {
            RecvAll(&strId[0], unIdLength);
         }
         UInt32 unTick = 0;
         double pfPose[7];
         double pfTwist[6];
         UInt8 unValid = 0;
         RecvAll(&unTick, sizeof(unTick));
         RecvAll(pfPose, sizeof(pfPose));
         RecvAll(pfTwist, sizeof(pfTwist));
         RecvAll(&unValid, sizeof(unValid));
         if(unValid == 0) {
            continue;
         }
         SEstimate sEstimate;
         sEstimate.Position.Set(pfPose[0], pfPose[1], pfPose[2]);
         sEstimate.Orientation.Set(pfPose[3], pfPose[4], pfPose[5], pfPose[6]);
         sEstimate.LinearVelocity.Set(pfTwist[0], pfTwist[1], pfTwist[2]);
         sEstimate.AngularVelocity.Set(pfTwist[3], pfTwist[4], pfTwist[5]);
         sEstimate.Tick = unTick;
         sEstimate.Valid = true;
         /* Place the estimate in the shared frame before storing it, so
          * every consumer sees one convention. A robot that cannot place
          * itself yet reports nothing rather than a pose in a private
          * frame that no other robot can use. */
         SRobot* psRobot = nullptr;
         for(SRobot& sCandidate : m_vecRobots) {
            if(sCandidate.Id == strId) {
               psRobot = &sCandidate;
               break;
            }
         }
         if(psRobot != nullptr && !AlignEstimate(*psRobot, sEstimate)) {
            continue;
         }
         m_mapEstimates[strId] = sEstimate;
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::SendAll(const void* pt_data, size_t un_size) {
      const auto* punData = static_cast<const UInt8*>(pt_data);
      size_t unSent = 0;
      while(unSent < un_size) {
         ssize_t nWritten = ::send(m_nSocket, punData + unSent,
                                   un_size - unSent, MSG_NOSIGNAL);
         if(nWritten <= 0) {
            if(errno == EINTR) continue;
            THROW_ARGOSEXCEPTION("Lost the external estimator while sending: "
                                 << ::strerror(errno));
         }
         unSent += size_t(nWritten);
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorMedium::RecvAll(void* pt_data, size_t un_size) {
      auto* punData = static_cast<UInt8*>(pt_data);
      size_t unRead = 0;
      while(unRead < un_size) {
         ssize_t nGot = ::recv(m_nSocket, punData + unRead, un_size - unRead, 0);
         if(nGot == 0) {
            THROW_ARGOSEXCEPTION("The external estimator closed the connection");
         }
         if(nGot < 0) {
            if(errno == EINTR) continue;
            if(errno == EAGAIN || errno == EWOULDBLOCK) {
               THROW_ARGOSEXCEPTION("The external estimator did not reply within "
                                    << m_fTimeout << " s; it is either stuck or "
                                    "has died");
            }
            THROW_ARGOSEXCEPTION("Lost the external estimator while receiving: "
                                 << ::strerror(errno));
         }
         unRead += size_t(nGot);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CExternalEstimatorMedium,
                   "external_estimator",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "Streams sensor data to a SLAM front-end outside ARGoS and "
                   "collects its pose estimates.",

                   "This medium connects to a Unix socket, sends every robot's IMU, camera,\n"
                   "lidar and wheel-encoder data once per tick, and receives pose estimates\n"
                   "back. The <odometry implementation=\"external\"> sensors then report those\n"
                   "poses to controllers, so a controller sees the drift a real algorithm\n"
                   "produced rather than a statistical model of drift.\n\n"

                   "ARGoS links no ROS, no Ceres and no OpenCV: the estimator runs in its own\n"
                   "process, in whatever container and ROS distribution it needs, and only the\n"
                   "socket protocol crosses the boundary. Two estimators built against\n"
                   "incompatible ROS distributions can therefore run against one simulation.\n\n"

                   "The exchange is blocking, so the simulation runs in lockstep with the\n"
                   "estimator and drops no data, however slow the estimator is.\n\n"

                   "Which sensors a robot carries decides what the estimator can fuse: the\n"
                   "\"imu\" sensor is required, while \"photorealistic_camera\",\n"
                   "\"photorealistic_lidar\" and \"differential_steering\" are each optional.\n"
                   "The wheel-encoder pose is dead-reckoned here from the differential steering\n"
                   "sensor's covered distances, so it slips when the wheels do.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <media>\n"
                   "    ...\n"
                   "    <external_estimator id=\"uf\" socket=\"/tmp/argos_uf.sock\" />\n"
                   "    ...\n"
                   "  </media>\n\n"

                   "and, on every robot whose estimate you want to read:\n\n"

                   "  <odometry implementation=\"external\" medium=\"uf\" />\n\n"

                   "OPTIONAL XML CONFIGURATION\n\n"

                   "Attribute 'robots' is a comma-separated list of the robots to stream; when\n"
                   "absent, every robot in the arena is streamed. Attribute 'connect_timeout'\n"
                   "(default 120 s) bounds the wait for the sidecar to come up, and 'timeout'\n"
                   "(default 120 s) bounds the wait for each reply, so a dead estimator stops\n"
                   "the run instead of hanging it. Attribute 'lockstep_pose' (default false)\n"
                   "asks the far side to hold its reply until the estimator has a pose for the\n"
                   "tick just sent: slower, but it removes the estimator's variable latency\n"
                   "from the run. With it false, the newest pose available is used, which lags\n"
                   "by a few ticks exactly as on a real robot.\n\n"

                   "  <media>\n"
                   "    ...\n"
                   "    <external_estimator id=\"uf\"\n"
                   "                        socket=\"/tmp/argos_uf.sock\"\n"
                   "                        robots=\"r0,r1,r2,r3\"\n"
                   "                        lockstep_pose=\"false\"\n"
                   "                        connect_timeout=\"120\"\n"
                   "                        timeout=\"120\" />\n"
                   "    ...\n"
                   "  </media>\n\n",

                   "Usable"
                  );

   /****************************************/
   /****************************************/

}
