/**
 * @file <argos3/plugins/robots/bunker-mini/simulator/bunker_mini_entity.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "bunker_mini_entity.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* Physical dimensions of AgileX Bunker Mini */
   static const Real BUNKER_MINI_LENGTH             = 0.660f;
   static const Real BUNKER_MINI_WIDTH              = 0.584f;
   static const Real BUNKER_MINI_HEIGHT             = 0.281f;
   static const Real BUNKER_MINI_HALF_HEIGHT        = BUNKER_MINI_HEIGHT * 0.5f;
   static const Real BUNKER_MINI_TRACK_GAUGE        = 0.412f;
   static const Real BUNKER_MINI_HALF_TRACK_GAUGE   = BUNKER_MINI_TRACK_GAUGE * 0.5f;
   static const Real BUNKER_MINI_SPROCKET_RADIUS    = 0.090f;

   /****************************************/
   /****************************************/

   CBunkerMiniEntity::CBunkerMiniEntity() :
      CComposableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
   }

   /****************************************/
   /****************************************/

   CBunkerMiniEntity::CBunkerMiniEntity(const std::string& str_id,
                                        const std::string& str_controller_id,
                                        const CVector3& c_position,
                                        const CQuaternion& c_orientation,
                                        const std::string& str_bat_model) :
      CComposableEntity(nullptr, str_id),
      m_pcEmbodiedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
      try {
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);
         /* Anchors */
         m_pcEmbodiedEntity->AddAnchor("body",
                                       CVector3(0.0f, 0.0f, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_track",
                                       CVector3(0.0f, BUNKER_MINI_HALF_TRACK_GAUGE, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_track",
                                       CVector3(0.0f, -BUNKER_MINI_HALF_TRACK_GAUGE, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(0.0f, 0.0f, BUNKER_MINI_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.30f, 0.0f, 0.25f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();

         /* Wheeled entity (2 tracks/wheels) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  BUNKER_MINI_HALF_TRACK_GAUGE, 0.0f), BUNKER_MINI_SPROCKET_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -BUNKER_MINI_HALF_TRACK_GAUGE, 0.0f), BUNKER_MINI_SPROCKET_RADIUS);

         /* Battery entity */
         m_pcBatteryEquippedEntity = new CBatteryEquippedEntity(this, "battery_0", str_bat_model);
         AddComponent(*m_pcBatteryEquippedEntity);

         /* Controllable entity */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);

         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBunkerMiniEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Initialize the base class */
         CComposableEntity::Init(t_tree);

         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

         /* Anchors */
         m_pcEmbodiedEntity->AddAnchor("body",
                                       CVector3(0.0f, 0.0f, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_track",
                                       CVector3(0.0f, BUNKER_MINI_HALF_TRACK_GAUGE, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_track",
                                       CVector3(0.0f, -BUNKER_MINI_HALF_TRACK_GAUGE, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(0.0f, 0.0f, BUNKER_MINI_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.30f, 0.0f, 0.25f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, BUNKER_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();

         /* Wheeled entity */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  BUNKER_MINI_HALF_TRACK_GAUGE, 0.0f), BUNKER_MINI_SPROCKET_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -BUNKER_MINI_HALF_TRACK_GAUGE, 0.0f), BUNKER_MINI_SPROCKET_RADIUS);

         /* Battery entity */
         m_pcBatteryEquippedEntity = new CBatteryEquippedEntity(this, "battery_0");
         if(NodeExists(t_tree, "battery")) {
            m_pcBatteryEquippedEntity->Init(GetNode(t_tree, "battery"));
         }
         AddComponent(*m_pcBatteryEquippedEntity);

         /* Controllable entity */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));

         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBunkerMiniEntity::Reset() {
      /* Reset base and components */
      CComposableEntity::Reset();
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CBunkerMiniEntity::UpdateComponents() {
      if(m_pcBatteryEquippedEntity != nullptr) {
         m_pcBatteryEquippedEntity->Update();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CBunkerMiniEntity,
                   "bunker_mini",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "The AgileX Bunker Mini tracked mobile robot.",
                   "The AgileX Bunker Mini is a compact tracked mobile robot equipped with differential\n"
                   "drive kinematics, photorealistic lidar, camera, and IMU.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <bunker_mini id=\"bm0\">\n"
                   "      <body position=\"0,0,0\" orientation=\"0,0,0\" />\n"
                   "      <controller config=\"bm0_controller\" />\n"
                   "    </bunker_mini>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  None.\n",
                   "Usable"
   );

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CBunkerMiniEntity);

}
