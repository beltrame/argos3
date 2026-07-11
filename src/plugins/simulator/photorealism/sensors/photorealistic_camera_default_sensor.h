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
         return m_sFrame;
      }

      virtual bool HasNewFrame() const {
         return m_bNewFrame;
      }

   private:

      CEmbodiedEntity* m_pcEmbodiedEntity = nullptr;
      CPhotorealismMedium* m_pcMedium = nullptr;
      UInt32 m_unCameraHandle = 0;
      SFrame m_sFrame;
      bool m_bNewFrame = false;
      bool m_bRGBEnabled = true;
      bool m_bDepthEnabled = true;
      bool m_bSegEnabled = true;
      Real m_fFarPlane = 20.0;

   };

}

#endif
