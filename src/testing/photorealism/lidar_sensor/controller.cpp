#include "controller.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/string_utilities.h>

#include <algorithm>
#include <cmath>
#include <vector>

/****************************************/
/****************************************/

void CLidarTestController::Init(TConfigurationNode& t_tree) {
   m_pcLidar = GetSensor<CCI_PhotorealisticLidarSensor>("photorealistic_lidar");
   std::string strRoom;
   GetNodeAttributeOrDefault(t_tree, "room", strRoom, strRoom);
   if(!strRoom.empty()) {
      ParseValues<Real>(strRoom, 5, m_fRoom, ',');
   }
   GetNodeAttributeOrDefault(t_tree, "sensor_position",
                             m_cSensorPosition, m_cSensorPosition);
   GetNodeAttributeOrDefault(t_tree, "expected_rings",
                             m_unExpectedRings, m_unExpectedRings);
   GetNodeAttributeOrDefault(t_tree, "expected_azimuths",
                             m_unExpectedAzimuths, m_unExpectedAzimuths);
   GetNodeAttributeOrDefault(t_tree, "max_error", m_fMaxError, m_fMaxError);
   GetNodeAttributeOrDefault(t_tree, "mean_error", m_fMeanError, m_fMeanError);
}

/****************************************/
/****************************************/

Real CLidarTestController::ExpectedRange(const CVector3& c_direction,
                                         Real f_max_range) const {
   const Real fOx = m_cSensorPosition.GetX();
   const Real fOy = m_cSensorPosition.GetY();
   const Real fOz = m_cSensorPosition.GetZ();
   const Real fDx = c_direction.GetX();
   const Real fDy = c_direction.GetY();
   const Real fDz = c_direction.GetZ();
   Real fBest = f_max_range;
   /* The four walls and the floor. The room has no ceiling, so a ray
    * steep enough to clear the walls escapes and reports a miss. */
   const Real pfPlane[5]   = {m_fRoom[0], m_fRoom[1], m_fRoom[2], m_fRoom[3], 0.0};
   /* 0 = x, 1 = y, 2 = z */
   const UInt32 punAxis[5] = {0, 0, 1, 1, 2};
   for(UInt32 i = 0; i < 5; ++i) {
      const Real fOrigin = punAxis[i] == 0 ? fOx : (punAxis[i] == 1 ? fOy : fOz);
      const Real fDir    = punAxis[i] == 0 ? fDx : (punAxis[i] == 1 ? fDy : fDz);
      if(std::abs(fDir) < 1e-12) continue;
      const Real fT = (pfPlane[i] - fOrigin) / fDir;
      if(fT <= 0.0 || fT >= fBest) continue;
      /* The hit has to lie within the face, not on its infinite plane */
      const Real fHitX = fOx + fT * fDx;
      const Real fHitY = fOy + fT * fDy;
      const Real fHitZ = fOz + fT * fDz;
      if(punAxis[i] != 0 && (fHitX < m_fRoom[0] || fHitX > m_fRoom[1])) continue;
      if(punAxis[i] != 1 && (fHitY < m_fRoom[2] || fHitY > m_fRoom[3])) continue;
      if(punAxis[i] != 2 && (fHitZ < 0.0 || fHitZ > m_fRoom[4])) continue;
      fBest = fT;
   }
   return fBest;
}

/****************************************/
/****************************************/

void CLidarTestController::ControlStep() {
   if(!m_pcLidar->HasNewScan()) {
      return;
   }
   const CCI_PhotorealisticLidarSensor::SScan& sScan = m_pcLidar->GetScan();
   /* Shape */
   if(sScan.NumRings != m_unExpectedRings ||
      sScan.NumAzimuths != m_unExpectedAzimuths) {
      THROW_ARGOSEXCEPTION("Unexpected scan shape " << sScan.NumRings << " rings x "
                           << sScan.NumAzimuths << " azimuths");
   }
   const size_t unExpected = size_t(sScan.NumRings) * sScan.NumAzimuths;
   if(sScan.Readings.size() != unExpected) {
      THROW_ARGOSEXCEPTION("Expected " << unExpected << " readings, got "
                           << sScan.Readings.size());
   }
   /* Ordering: azimuth-major, ring fastest, as a VLP-16 emits */
   for(size_t i = 0; i < sScan.Readings.size(); ++i) {
      const CCI_PhotorealisticLidarSensor::SReading& sReading = sScan.Readings[i];
      if(sReading.Ring != UInt32(i % sScan.NumRings)) {
         THROW_ARGOSEXCEPTION("Reading " << i << " has ring " << sReading.Ring
                              << ", expected " << (i % sScan.NumRings));
      }
   }

   /*
    * Every ray against the room geometry.
    */
   std::vector<Real> vecErrors;
   vecErrors.reserve(sScan.Readings.size());
   Real fTotal = 0.0;
   Real fWorst = 0.0;
   size_t unWorst = 0;
   size_t unMisses = 0;
   for(size_t i = 0; i < sScan.Readings.size(); ++i) {
      const CCI_PhotorealisticLidarSensor::SReading& sReading = sScan.Readings[i];
      const Real fCosElevation = std::cos(sReading.Elevation.GetValue());
      const CVector3 cDirection(fCosElevation * std::cos(sReading.Azimuth.GetValue()),
                                fCosElevation * std::sin(sReading.Azimuth.GetValue()),
                                std::sin(sReading.Elevation.GetValue()));
      const Real fExpected = ExpectedRange(cDirection, sScan.MaxRange);
      const Real fError = std::abs(sReading.Range - fExpected);
      vecErrors.push_back(fError);
      fTotal += fError;
      if(fError > fWorst) {
         fWorst = fError;
         unWorst = i;
      }
      if(!sReading.Hit) ++unMisses;
      /* The reported endpoint must agree with the reported range */
      if(sReading.Hit) {
         const Real fEndpointError =
            std::abs(sReading.Position.Length() - sReading.Range);
         if(fEndpointError > 1e-6) {
            THROW_ARGOSEXCEPTION("Reading " << i << " endpoint is "
                                 << sReading.Position.Length()
                                 << " from the origin but reports range "
                                 << sReading.Range);
         }
      }
   }
   const Real fMean = fTotal / Real(vecErrors.size());
   std::vector<Real> vecSorted(vecErrors);
   std::sort(vecSorted.begin(), vecSorted.end());
   const Real fMedian = vecSorted[vecSorted.size() / 2];
   const Real fP99 = vecSorted[size_t(0.99 * vecSorted.size())];

   LOG << "[LIDAR] tick=" << sScan.Tick
       << " rays=" << sScan.Readings.size()
       << " misses=" << unMisses << std::endl;
   LOG << "[LIDAR] range error: mean=" << fMean
       << " median=" << fMedian
       << " p99=" << fP99
       << " max=" << fWorst << std::endl;
   {
      /* The cardinal directions and the diagonal, on the ring nearest
       * the horizontal. The diagonal is the interesting one: with four
       * faces it falls exactly on a seam, and it is the direction where
       * the difference between depth along the optical axis and true
       * range is largest. */
      const UInt32 unRing = sScan.NumRings / 2;
      for(UInt32 unDegrees = 0; unDegrees < 360; unDegrees += 45) {
         const auto unAzimuth =
            UInt32(std::lround(Real(unDegrees) / 360.0 * sScan.NumAzimuths));
         const size_t unIndex =
            (size_t(unAzimuth) % sScan.NumAzimuths) * sScan.NumRings + unRing;
         const CCI_PhotorealisticLidarSensor::SReading& sReading =
            sScan.Readings[unIndex];
         const Real fCosElevation = std::cos(sReading.Elevation.GetValue());
         const CVector3 cDirection(fCosElevation * std::cos(sReading.Azimuth.GetValue()),
                                   fCosElevation * std::sin(sReading.Azimuth.GetValue()),
                                   std::sin(sReading.Elevation.GetValue()));
         LOG << "[LIDAR] azimuth " << unDegrees << " deg: range="
             << sReading.Range << " expected="
             << ExpectedRange(cDirection, sScan.MaxRange) << std::endl;
      }
   }
   LOG.Flush();

   if(fWorst > m_fMaxError) {
      const CCI_PhotorealisticLidarSensor::SReading& sReading = sScan.Readings[unWorst];
      THROW_ARGOSEXCEPTION("Reading " << unWorst << " (azimuth "
                           << ToDegrees(sReading.Azimuth) << ", elevation "
                           << ToDegrees(sReading.Elevation) << ") is off by "
                           << fWorst << " m, tolerance " << m_fMaxError
                           << ". Reported " << sReading.Range << " m.");
   }
   if(fMean > m_fMeanError) {
      THROW_ARGOSEXCEPTION("Mean range error " << fMean << " m exceeds "
                           << m_fMeanError << " m");
   }
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CLidarTestController, "lidar_test_controller");
