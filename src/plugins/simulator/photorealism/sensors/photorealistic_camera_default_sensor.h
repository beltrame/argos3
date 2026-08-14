/**
 * @file <argos3/plugins/simulator/photorealism/sensors/photorealistic_camera_default_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PHOTOREALISTIC_CAMERA_DEFAULT_SENSOR_H
#define PHOTOREALISTIC_CAMERA_DEFAULT_SENSOR_H

namespace argos {
   class CPhotorealisticCameraDefaultSensor;
   class CPhotorealismMedium;
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_camera_pool.h>

#include <string>
#include <vector>

namespace argos {

   class CPhotorealisticCameraDefaultSensor : public CSimulatedSensor,
                                              public CCI_PhotorealisticCameraSensor {

   public:

      CPhotorealisticCameraDefaultSensor() {}
      virtual ~CPhotorealisticCameraDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();
      virtual void Destroy();

      virtual const SFrame& GetFrame() const {
         return m_vecMounts.front().Frame;
      }

      virtual bool HasNewFrame() const {
         for(const SMount& sMount : m_vecMounts) {
            if(sMount.NewFrame) return true;
         }
         return false;
      }

      virtual size_t GetNumCameras() const { return m_vecMounts.size(); }

      virtual const SFrame& GetFrame(size_t un_index) const {
         return m_vecMounts.at(un_index).Frame;
      }

      virtual bool HasNewFrame(size_t un_index) const {
         return m_vecMounts.at(un_index).NewFrame;
      }

   private:

      /** One camera mounted on the robot. ARGoS keys sensors by type, so a
       *  robot cannot carry two <photorealistic_camera> entries: the second
       *  would overwrite the first in the controller's sensor map. Multiple
       *  viewpoints are therefore mounts of a single sensor. */
      struct SMount {
         std::string Id = "default";
         UInt32 Handle = 0;
         SFrame Frame;
         bool NewFrame = false;
         bool RGBEnabled = true;
         bool DepthEnabled = true;
         bool SegEnabled = true;
         Real FarPlane = 20.0;
      };

      /** Reads one camera's attributes from `t_node` into a pool config. */
      void ParseMount(TConfigurationNode& t_node, const std::string& str_id);

      CEmbodiedEntity* m_pcEmbodiedEntity = nullptr;
      CPhotorealismMedium* m_pcMedium = nullptr;
      std::vector<SMount> m_vecMounts;

   };

}

#endif
