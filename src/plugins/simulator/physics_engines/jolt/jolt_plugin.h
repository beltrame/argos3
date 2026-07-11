/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_plugin.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_PLUGIN_H
#define JOLT_PLUGIN_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/plugins/factory.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CJoltPlugin {

   public:

      using TMap = std::map<std::string, CJoltPlugin*>;

   public:

      CJoltPlugin() :
         m_pcEngine(nullptr) {}

      virtual ~CJoltPlugin() {}

      virtual void Init(TConfigurationNode& t_tree) {}

      virtual void Reset() {}

      virtual void Destroy() {}

      virtual void SetEngine(CJoltEngine& c_engine) {
         m_pcEngine = &c_engine;
      }

      virtual void RegisterModel(CJoltModel& c_model) = 0;

      virtual void UnregisterModel(CJoltModel& c_model) = 0;

      /** Called once per physics sub-step, before the step */
      virtual void Update() = 0;

   protected:

      CJoltEngine* m_pcEngine;

   };

   /****************************************/
   /****************************************/

}

#define REGISTER_JOLT_PLUGIN(CLASSNAME,         \
                             LABEL,             \
                             AUTHOR,            \
                             VERSION,           \
                             BRIEF_DESCRIPTION, \
                             LONG_DESCRIPTION,  \
                             STATUS)            \
   REGISTER_SYMBOL(CJoltPlugin,                 \
                   CLASSNAME,                   \
                   LABEL,                       \
                   AUTHOR,                      \
                   VERSION,                     \
                   BRIEF_DESCRIPTION,           \
                   LONG_DESCRIPTION,            \
                   STATUS)

#endif
