/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.h>
 *
 * Control interface for the photorealistic camera sensor. The sensor
 * delivers rendered images from a camera mounted on the robot: RGB
 * color, metric depth along the optical axis, and per-pixel entity/
 * class segmentation ids.
 *
 * By default frames are pipelined: the frame available during a
 * control step depicts the state of the previous simulation tick
 * (matching the latency of a real camera and keeping the GPU busy).
 * The <photorealism> medium can be configured with
 * latency="immediate" to deliver same-tick frames instead.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef CCI_PHOTOREALISTIC_CAMERA_SENSOR_H
#define CCI_PHOTOREALISTIC_CAMERA_SENSOR_H

namespace argos {
   class CCI_PhotorealisticCameraSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/datatypes/datatypes.h>

#include <vector>

namespace argos {

   class CCI_PhotorealisticCameraSensor : public CCI_Sensor {

   public:

      struct SFrame {
         /** Image width in pixels */
         UInt32 Width = 0;
         /** Image height in pixels */
         UInt32 Height = 0;
         /** The simulation tick this frame depicts */
         UInt32 Tick = 0;
         /** RGB pixels, row-major from the top-left, Width*Height*3 */
         std::vector<UInt8> RGB;
         /** Metric depth along the optical axis in meters, Width*Height;
          *  pixels with no geometry hold the far-plane distance.
          *  Empty when the "depth" modality is disabled */
         std::vector<Real> Depth;
         /** Per-pixel entity id, Width*Height; 0 means no entity.
          *  Empty when the "seg" modality is disabled */
         std::vector<UInt16> EntityId;
         /** Per-pixel class id, Width*Height; 0 means no entity.
          *  Empty when the "seg" modality is disabled */
         std::vector<UInt8> ClassId;
      };

   public:

      virtual ~CCI_PhotorealisticCameraSensor() {}

      /**
       * Returns the most recent frame.
       * Contents are valid only when a frame has been delivered at
       * least once; check HasNewFrame() or SFrame::Tick.
       */
      virtual const SFrame& GetFrame() const = 0;

      /**
       * Returns true when a new frame was delivered during the
       * current control step.
       */
      virtual bool HasNewFrame() const = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   };

}

#endif
