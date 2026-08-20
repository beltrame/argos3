/**
 * @file <argos3/plugins/simulator/photorealism/sensors/photorealistic_lidar_default_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PHOTOREALISTIC_LIDAR_DEFAULT_SENSOR_H
#define PHOTOREALISTIC_LIDAR_DEFAULT_SENSOR_H

namespace argos {
   class CPhotorealisticLidarDefaultSensor;
   class CPhotorealismMedium;
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_camera_pool.h>

#include <vector>

namespace argos {

   /**
    * A spinning lidar built out of depth cameras.
    *
    * The renderer only draws pinhole views, and a pinhole cannot see a
    * full turn: its horizontal field of view is
    *
    *    hfov = 2 * atan(aspect * tan(vfov / 2))
    *
    * which approaches but never reaches 180 degrees. The sensor
    * therefore renders a ring of faces covering 360 degrees between
    * them, and resamples them into the lidar's ray pattern. Faces are
    * ordinary pool cameras rendering the aux pass only, so they cost
    * no more than the equivalent depth cameras and are batched into
    * the same atlas.
    */
   class CPhotorealisticLidarDefaultSensor : public CSimulatedSensor,
                                             public CCI_PhotorealisticLidarSensor {

   public:

      CPhotorealisticLidarDefaultSensor() {}
      virtual ~CPhotorealisticLidarDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();
      virtual void Destroy();

      virtual const SScan& GetScan() const { return m_sScan; }

      virtual bool HasNewScan() const { return m_bNewScan; }

   private:

      /** One rendered view covering a slice of the azimuth range */
      struct SFace {
         UInt32 Handle = 0;
         UInt32 Width = 0;
         UInt32 Height = 0;
      };

      /** Where a ray reads its depth from. Built once in Init(),
       *  because the pattern is fixed in the sensor frame: per tick
       *  the sensor only looks pixels up and scales them. */
      struct SSample {
         /** Index into m_vecFaces */
         UInt32 Face = 0;
         /** Pixel index into that face's aux buffer */
         UInt32 Pixel = 0;
         /** Planar depth to range: 1 / cos(angle off the optical
          *  axis). The aux pass stores depth along the optical axis,
          *  not distance from the camera. */
         Real DepthToRange = 1.0;
         /** Unit ray direction in the sensor frame */
         CVector3 Direction;
      };

      /** Reads the ray pattern and builds m_vecFaces / m_vecSamples. */
      void BuildRayTable(TConfigurationNode& t_tree,
                         const SPRCameraConfig& s_base,
                         UInt32 un_faces,
                         UInt32 un_face_width,
                         UInt32 un_face_height);

      CEmbodiedEntity* m_pcEmbodiedEntity = nullptr;
      CPhotorealismMedium* m_pcMedium = nullptr;
      std::vector<SFace> m_vecFaces;
      std::vector<SSample> m_vecSamples;
      SScan m_sScan;
      bool m_bNewScan = false;

      /** Standard deviation of the Gaussian noise added to the range of
       *  every return, in metres. Zero (the default) leaves ranges
       *  geometrically exact, which no real lidar is: a VLP-16 is
       *  specified at about +/-3 cm. */
      Real m_fRangeNoiseStdDev = 0.0;
      /** Only created when noise is actually configured */
      CRandom::CRNG* m_pcRNG = nullptr;
      /** Closest range the optics can report; noise is clamped to it */
      Real m_fNearPlane = 0.0;

   };

}

#endif
