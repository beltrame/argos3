/**
 * @file <argos3/plugins/simulator/photorealism/pr_overlay.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_overlay.h"
#include "photorealism_medium.h"

#include <argos3/core/simulator/simulator.h>

namespace argos {

   /****************************************/
   /****************************************/

   CPROverlay& GetPhotorealismOverlay(const std::string& str_medium_id) {
      /* The Filament-dependent lookup happens here, inside the plugin, so the
       * caller never sees a Filament type */
      return CSimulator::GetInstance()
         .GetMedium<CPhotorealismMedium>(str_medium_id)
         .GetDebugDraw();
   }

   /****************************************/
   /****************************************/

}
