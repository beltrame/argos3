/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_gravity_plugin.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_gravity_plugin.h"

namespace argos {

   /****************************************/
   /****************************************/

   void CJoltGravityPlugin::Init(TConfigurationNode& t_tree) {
      Real fAcceleration = 9.81;
      GetNodeAttributeOrDefault(t_tree, "g", fAcceleration, fAcceleration);
      m_pcEngine->GetSystem().SetGravity(
         JPH::Vec3(0.0f, 0.0f, -float(fAcceleration)));
   }

   /****************************************/
   /****************************************/

   REGISTER_JOLT_PLUGIN(CJoltGravityPlugin,
                        "gravity",
                        "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                        "1.0",
                        "Enables gravity in the Jolt engine",
                        "For a description on how to use this plugin, please consult the documentation\n"
                        "for the jolt physics engine plugin",
                        "Usable");

   /****************************************/
   /****************************************/

}
