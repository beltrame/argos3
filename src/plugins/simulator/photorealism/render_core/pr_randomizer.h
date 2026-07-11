/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_randomizer.h>
 *
 * Domain randomization for sim-to-real transfer: draws sunlight,
 * sky color, and material parameters from configured ranges using
 * the ARGoS RNG, so randomization is deterministic per seed.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_RANDOMIZER_H
#define PR_RANDOMIZER_H

namespace argos {
   class CPRSceneSync;
}

namespace filament {
   class Skybox;
}

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_id_scene.h>

#include <set>

namespace argos {

   class CPRRandomizer {

   public:

      /**
       * Parses the <randomization> configuration node.
       */
      void Init(TConfigurationNode& t_tree);

      inline bool IsEnabled() const {
         return m_bEnabled;
      }

      inline bool AppliesOnReset() const {
         return m_bOnReset;
      }

      /**
       * Draws a new random environment and applies it to the scene.
       */
      void Apply(CRandom::CRNG& c_rng,
                 CPRSceneSync& c_scene_sync,
                 filament::Skybox* pc_skybox);

   private:

      bool m_bEnabled = false;
      bool m_bOnReset = true;
      /* Sunlight: intensity in lux, angles in degrees */
      bool m_bSun = false;
      CRange<Real> m_cSunIntensity = CRange<Real>(60000.0, 140000.0);
      CRange<Real> m_cSunElevation = CRange<Real>(25.0, 80.0);
      CRange<Real> m_cSunAzimuth = CRange<Real>(0.0, 360.0);
      /* Sky color, per-channel uniform between the two colors */
      bool m_bSky = false;
      CVector3 m_cSkyColorMin = CVector3(0.3, 0.4, 0.55);
      CVector3 m_cSkyColorMax = CVector3(0.7, 0.8, 1.0);
      /* Materials */
      bool m_bMaterials = false;
      std::set<EPRClass> m_setClasses;
      CRange<Real> m_cRoughness = CRange<Real>(0.0, 0.0);
      Real m_fColorJitter = 0.0;

   };

}

#endif
