/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_gravity_plugin.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_GRAVITY_PLUGIN_H
#define JOLT_GRAVITY_PLUGIN_H

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_plugin.h>

namespace argos {

   /**
    * Enables gravity in the Jolt world (Jolt integrates gravity
    * itself; this plugin only configures it).
    */
   class CJoltGravityPlugin : public CJoltPlugin {

   public:

      virtual void Init(TConfigurationNode& t_tree);

      virtual void RegisterModel(CJoltModel& c_model) {}

      virtual void UnregisterModel(CJoltModel& c_model) {}

      virtual void Update() {}

   };

}

#endif
