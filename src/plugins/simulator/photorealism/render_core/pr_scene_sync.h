/**
 * @file <argos3/plugins/simulator/photorealism/render_core/pr_scene_sync.h>
 *
 * Keeps the Filament scene in sync with the ARGoS space: creates and
 * destroys renderables as entities appear and disappear, and copies
 * anchor poses, LED colors and the floor texture into the scene every
 * tick.
 *
 * An entity's visual is a set of parts. Regular parts carry a local
 * transform composed with the entity's origin anchor; LED parts are
 * small emissive cubes that follow the global position and color of
 * their CLEDEntity, which makes LED rendering work for any robot
 * without robot-specific code.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef PR_SCENE_SYNC_H
#define PR_SCENE_SYNC_H

namespace argos {
   class CSpace;
   class CEmbodiedEntity;
   class CComposableEntity;
   class CLEDEquippedEntity;
   class CDirectionalLEDEquippedEntity;
   class CFloorEntity;
   class CPRRenderEngine;
   class CPRIdScene;
}

namespace filament {
   class MaterialInstance;
   class IndirectLight;
   class Texture;
   class Skybox;
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_mesh_builder.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_id_scene.h>
#include <argos3/plugins/simulator/photorealism/render_core/pr_asset_registry.h>

#include <utils/Entity.h>
#include <math/mat4.h>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace argos {

   class CPRSceneSync {

   public:

      struct SSunlight {
         CVector3 Direction = CVector3(0.6, 0.2, -1.0);
         Real Intensity = 100000.0; /* lux */
         bool CastShadows = true;
      };

      CPRSceneSync() {}
      ~CPRSceneSync() {}

      /**
       * Builds the shared meshes, the sunlight, and the floor plane.
       * The id scene must be initialized already; it receives a paired
       * renderable for every visual this class creates.
       */
      void Init(CPRRenderEngine& c_engine,
                CPRIdScene& c_id_scene,
                CPRAssetRegistry& c_assets,
                const SSunlight& s_sunlight,
                const CVector3& c_arena_size,
                const CVector3& c_arena_center);

      void Destroy();

      /**
       * Diffs the space against the tracked instances and updates all
       * transforms, LED emissives, and the floor texture. Must be
       * called on the render thread once per tick.
       */
      void Sync(CSpace& c_space);

      /**
       * Returns the numeric id used in segmentation images for the
       * given embodied entity, or 0 if the entity has no visual.
       * Ids are stable within a run.
       */
      UInt16 GetEntityId(const CEmbodiedEntity& c_entity) const;

      /**
       * Replaces the constant ambient light and the sky with an HDR
       * environment: a prefiltered reflections cubemap and a skybox
       * cubemap in KTX1 format, as produced by Filament's cmgen tool
       * (<name>_ibl.ktx and <name>_skybox.ktx). The skybox path may
       * be empty to keep the current sky.
       */
      void LoadEnvironment(const std::string& str_ibl_file,
                           const std::string& str_skybox_file,
                           Real f_intensity);

      /**
       * Changes the sunlight direction and intensity at runtime; the
       * ambient light intensity follows the sun unless an HDR
       * environment is loaded.
       */
      void SetSunlight(const CVector3& c_direction, Real f_intensity);

      /**
       * Sets a float material parameter ("roughness" or "metallic")
       * on all non-LED parts of the given entity's visual.
       */
      void SetMaterialParam(const CEmbodiedEntity& c_entity,
                            const std::string& str_name,
                            Real f_value);

      /**
       * Sets the base color of all non-LED parts of the given
       * entity's visual.
       */
      void SetMaterialColor(const CEmbodiedEntity& c_entity,
                            const CVector3& c_color);

      /**
       * Randomizes the materials of all parts whose segmentation
       * class is in set_classes (LED emissives are never touched).
       * Roughness is drawn uniformly from c_roughness (when the range
       * is non-empty); each base color channel is jittered around the
       * authored color by a factor uniform in [1-j, 1+j]. Including
       * EPRClass::Floor randomizes the floor roughness (its colors
       * come from the floor entity).
       */
      void RandomizeMaterials(CRandom::CRNG& c_rng,
                              const std::set<EPRClass>& set_classes,
                              const CRange<Real>& c_roughness,
                              Real f_color_jitter);

      /**
       * Entities whose root id starts with one of these prefixes are
       * never added to the render scene (any modality). Meant for
       * collision proxies that approximate scenery geometry which is
       * already rendered as a glTF prop. Set before the first Sync().
       */
      void SetHiddenIdPrefixes(const std::vector<std::string>& vec_prefixes);

      /**
       * When false, the arena floor plane is never created: scenery
       * props that model their own ground would otherwise be occluded
       * wherever their surface dips below z=0. Set before the first
       * Sync().
       */
      void SetDrawFloor(bool b_draw_floor);

      /**
       * Reserves the next free numeric entity id. Every renderable that
       * reaches the id scene must have one, including geometry this
       * class does not own (scenery props, created by the medium), so
       * that id 0 keeps meaning "nothing was drawn here". Ids are
       * handed out monotonically and are never reused, so allocating
       * before the first Sync() is safe.
       */
      UInt16 AllocateEntityId();

   private:

      struct SPart {
         utils::Entity Renderable;
         filament::MaterialInstance* Material = nullptr;
         /* Local transform (scale included), composed with the origin
          * anchor; ignored except for scale when LEDIndex >= 0 */
         filament::math::mat4f Local;
         /* When >= 0, this part follows LED number LEDIndex */
         SInt32 LEDIndex = -1;
         /* Authored material values, the stable baseline that
          * randomization jitters from */
         CVector3 BaseColor;
         Real BaseRoughness = 0.6;
      };

      struct SInstance {
         std::vector<SPart> Parts;
         UInt16 EntityId = 0;
         EPRClass Class = EPRClass::None;
         /* LED components, when the entity has them */
         CLEDEquippedEntity* LEDs = nullptr;
         CDirectionalLEDEquippedEntity* DirectionalLEDs = nullptr;
         /* glTF visual from the asset registry, when the entity type
          * has a descriptor; Parts then only holds the LED emissives */
         std::string Type;
         CPRAssetRegistry::SInstance Gltf;
         filament::math::mat4f GltfOffset;
      };

      void AddEntity(CEmbodiedEntity& c_entity);
      bool IsHidden(const CEmbodiedEntity& c_entity) const;
      void RemoveInstance(SInstance& s_instance);
      void UpdateInstance(CEmbodiedEntity& c_entity, SInstance& s_instance);

      /**
       * Creates a renderable part plus its aux-scene pair.
       */
      SPart MakePart(const SPRMesh& s_mesh,
                     const filament::math::mat4f& c_local,
                     float f_r, float f_g, float f_b,
                     float f_roughness,
                     UInt16 un_entity_id,
                     EPRClass e_class);

      /* Placeholder robot bodies with correct overall dimensions;
       * replaced by glTF assets when the asset pipeline lands */
      void BuildFootBot(SInstance& s_instance);
      void BuildDrone(SInstance& s_instance);

      void InitFloor(CSpace& c_space,
                     const CVector3& c_arena_size,
                     const CVector3& c_arena_center);
      void SyncFloorTexture();

   private:

      CPRRenderEngine* m_pcEngine = nullptr;
      CPRIdScene* m_pcIdScene = nullptr;
      CPRAssetRegistry* m_pcAssets = nullptr;
      /* Root entity id prefixes excluded from rendering */
      std::vector<std::string> m_vecHiddenIdPrefixes;
      /* When false, the arena floor plane is not rendered */
      bool m_bDrawFloor = true;
      /* Next numeric entity id to assign (0 = none, 1 = floor) */
      UInt16 m_unNextEntityId = 2;
      SPRMesh m_sBoxMesh;
      SPRMesh m_sCylinderMesh;
      SPRMesh m_sPlaneMesh;
      utils::Entity m_cSunlight;
      filament::IndirectLight* m_pcAmbientLight = nullptr;
      /* HDR environment, when loaded */
      bool m_bHasEnvironment = false;
      filament::Texture* m_pcEnvironmentReflections = nullptr;
      filament::Texture* m_pcEnvironmentSkyboxTexture = nullptr;
      filament::Skybox* m_pcEnvironmentSkybox = nullptr;
      /* Floor */
      utils::Entity m_cFloor;
      filament::MaterialInstance* m_pcFloorMaterial = nullptr;
      filament::Texture* m_pcFloorTexture = nullptr;
      filament::Material* m_pcTexturedMaterial = nullptr;
      CFloorEntity* m_pcFloorEntity = nullptr;
      CVector3 m_cArenaSize;
      CVector3 m_cArenaCenter;
      UInt32 m_unFloorTextureSize = 512;
      std::map<CEmbodiedEntity*, SInstance> m_mapInstances;
      /* Entity types already reported as not renderable */
      std::set<std::string> m_setWarnedTypes;

   };

}

#endif
