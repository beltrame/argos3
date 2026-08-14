/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>
 *
 * Control interface for the photorealistic lidar sensor. The sensor
 * delivers a spinning-lidar scan of the rendered scene: a full ring of
 * azimuths, each carrying one range per laser channel, in the order a
 * Velodyne VLP-16 emits them.
 *
 * The defaults model a VLP-16: 16 channels evenly spaced over a 30
 * degree vertical field of view (+15 to -15), a full 360 degree
 * azimuth sweep, and a 20 m maximum range.
 *
 * A scan is a snapshot: every ray in it depicts the same simulation
 * tick. Real spinning lidars smear a revolution over its duration, and
 * this sensor does not model that.
 *
 * Like the camera, scans are pipelined by default: the scan available
 * during a control step depicts the previous simulation tick. The
 * <photorealism> medium can be configured with latency="immediate"
 * for same-tick scans.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef CCI_PHOTOREALISTIC_LIDAR_SENSOR_H
#define CCI_PHOTOREALISTIC_LIDAR_SENSOR_H

namespace argos {
   class CCI_PhotorealisticLidarSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

#include <vector>

namespace argos {

   class CCI_PhotorealisticLidarSensor : public CCI_Sensor {

   public:

      /** One laser return. */
      struct SReading {
         /** Distance from the sensor origin along the ray, in meters.
          *  Rays that hit nothing carry MaxRange and Hit == false. */
         Real Range = 0.0;
         /** Endpoint in the sensor frame: +x forward, +y left, +z up.
          *  Meaningful only when Hit is true. */
         CVector3 Position;
         /** Laser channel, 0 at the lowest elevation */
         UInt32 Ring = 0;
         /** Ray azimuth in the sensor frame, 0 along +x, growing
          *  towards +y (counter-clockwise seen from above) */
         CRadians Azimuth;
         /** Ray elevation, positive upwards */
         CRadians Elevation;
         /** Entity the ray hit; 0 when it hit nothing */
         UInt16 EntityId = 0;
         /** Class of the entity the ray hit; 0 when it hit nothing */
         UInt8 ClassId = 0;
         /** False when the ray reached the maximum range without
          *  hitting anything. Consumers building an occupancy map need
          *  this: a miss still clears the space it travelled through,
          *  but must not mark an obstacle at its endpoint. */
         bool Hit = false;
      };

      struct SScan {
         /** The simulation tick this scan depicts */
         UInt32 Tick = 0;
         /** Number of laser channels */
         UInt32 NumRings = 0;
         /** Number of azimuth steps in a revolution */
         UInt32 NumAzimuths = 0;
         /** The range a ray that hits nothing reports */
         Real MaxRange = 0.0;
         /** Readings ordered azimuth-major, the order a VLP-16 emits
          *  them: index = azimuth * NumRings + ring. Size is
          *  NumAzimuths * NumRings. */
         std::vector<SReading> Readings;
      };

   public:

      virtual ~CCI_PhotorealisticLidarSensor() {}

      /**
       * Returns the most recent scan. Contents are valid only once a
       * scan has been delivered; check HasNewScan() or SScan::Tick.
       */
      virtual const SScan& GetScan() const = 0;

      /** Whether a new scan arrived during this control step. */
      virtual bool HasNewScan() const = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   };

}

#endif
