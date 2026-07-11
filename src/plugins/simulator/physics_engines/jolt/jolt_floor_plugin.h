/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_floor_plugin.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_FLOOR_PLUGIN_H
#define JOLT_FLOOR_PLUGIN_H

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_plugin.h>

namespace argos {

   /**
    * Adds a static floor (a large box whose top face lies at the
    * configured height) to the Jolt world.
    */
   class CJoltFloorPlugin : public CJoltPlugin {

   public:

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Destroy();

      virtual void RegisterModel(CJoltModel& c_model) {}

      virtual void UnregisterModel(CJoltModel& c_model) {}

      virtual void Update() {}

   private:

      JPH::BodyID m_cFloorId;

   };

}

#endif
