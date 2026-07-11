/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_randomizer.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "pr_randomizer.h"
#include "pr_scene_sync.h"

#include <argos3/core/utility/string_utilities.h>

#include <filament/Skybox.h>

#include <cmath>

namespace argos {

   /****************************************/
   /****************************************/

   static EPRClass ParseClass(const std::string& str_target) {
      if(str_target == "box")      return EPRClass::Box;
      if(str_target == "cylinder") return EPRClass::Cylinder;
      if(str_target == "floor")    return EPRClass::Floor;
      if(str_target == "foot-bot") return EPRClass::FootBot;
      if(str_target == "drone")    return EPRClass::Drone;
      THROW_ARGOSEXCEPTION("Unknown material randomization target \""
                           << str_target << "\"; use box, cylinder, "
                           "floor, foot-bot, or drone");
   }

   /****************************************/
   /****************************************/

   void CPRRandomizer::Init(TConfigurationNode& t_tree) {
      m_bEnabled = true;
      GetNodeAttributeOrDefault(t_tree, "on_reset", m_bOnReset, m_bOnReset);
      if(NodeExists(t_tree, "sun")) {
         m_bSun = true;
         TConfigurationNode& tSun = GetNode(t_tree, "sun");
         GetNodeAttributeOrDefault(tSun, "intensity", m_cSunIntensity, m_cSunIntensity);
         GetNodeAttributeOrDefault(tSun, "elevation", m_cSunElevation, m_cSunElevation);
         GetNodeAttributeOrDefault(tSun, "azimuth", m_cSunAzimuth, m_cSunAzimuth);
      }
      if(NodeExists(t_tree, "sky")) {
         m_bSky = true;
         TConfigurationNode& tSky = GetNode(t_tree, "sky");
         GetNodeAttributeOrDefault(tSky, "color_min", m_cSkyColorMin, m_cSkyColorMin);
         GetNodeAttributeOrDefault(tSky, "color_max", m_cSkyColorMax, m_cSkyColorMax);
      }
      if(NodeExists(t_tree, "materials")) {
         m_bMaterials = true;
         TConfigurationNode& tMaterials = GetNode(t_tree, "materials");
         std::string strTargets("box,cylinder,floor");
         GetNodeAttributeOrDefault(tMaterials, "targets", strTargets, strTargets);
         std::vector<std::string> vecTargets;
         Tokenize(strTargets, vecTargets, ",");
         for(const std::string& str_target : vecTargets) {
            m_setClasses.insert(ParseClass(str_target));
         }
         GetNodeAttributeOrDefault(tMaterials, "roughness", m_cRoughness, m_cRoughness);
         GetNodeAttributeOrDefault(tMaterials, "color_jitter", m_fColorJitter, m_fColorJitter);
      }
   }

   /****************************************/
   /****************************************/

   void CPRRandomizer::Apply(CRandom::CRNG& c_rng,
                             CPRSceneSync& c_scene_sync,
                             filament::Skybox* pc_skybox) {
      if(m_bSun) {
         Real fIntensity = c_rng.Uniform(m_cSunIntensity);
         Real fElevation = c_rng.Uniform(m_cSunElevation) * M_PI / 180.0;
         Real fAzimuth = c_rng.Uniform(m_cSunAzimuth) * M_PI / 180.0;
         /* The direction points from the sun towards the scene */
         CVector3 cDirection(-std::cos(fElevation) * std::cos(fAzimuth),
                             -std::cos(fElevation) * std::sin(fAzimuth),
                             -std::sin(fElevation));
         c_scene_sync.SetSunlight(cDirection, fIntensity);
      }
      if(m_bSky && pc_skybox != nullptr) {
         Real fR = c_rng.Uniform(CRange<Real>(m_cSkyColorMin.GetX(),
                                              m_cSkyColorMax.GetX()));
         Real fG = c_rng.Uniform(CRange<Real>(m_cSkyColorMin.GetY(),
                                              m_cSkyColorMax.GetY()));
         Real fB = c_rng.Uniform(CRange<Real>(m_cSkyColorMin.GetZ(),
                                              m_cSkyColorMax.GetZ()));
         pc_skybox->setColor({float(fR), float(fG), float(fB), 1.0f});
      }
      if(m_bMaterials) {
         c_scene_sync.RandomizeMaterials(c_rng, m_setClasses,
                                         m_cRoughness, m_fColorJitter);
      }
   }

   /****************************************/
   /****************************************/

}
