/* Exercise Spot's real surface-velocity callback at known body-local contacts. */
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_model.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;

class CSupportContacts : public CLoopFunctions {
public:
   void PostStep() override {
      if(GetSpace().GetSimulationClock() < 2) return;
      auto& cEntity = dynamic_cast<CComposableEntity&>(GetSpace().GetEntity("test"));
      auto& cModel = dynamic_cast<CJoltModel&>(
         cEntity.GetComponent<CEmbodiedEntity>("body").GetPhysicsModel("jolt"));
      JPH::BodyLockRead cLock(cModel.GetJoltEngine().GetSystem().GetBodyLockInterface(),
                              cModel.GetBodies()[0].Id);
      if(!cLock.Succeeded()) THROW_ARGOSEXCEPTION("Cannot lock test body");
      const JPH::Body& cBody = cLock.GetBody();
      const JPH::Quat cRotation = cBody.GetRotation();
      auto Check = [&](const char* pchName, JPH::Vec3 cNormal,
                       JPH::Vec3 cPoint, bool bDrive) {
         JPH::ContactPoints cPoints;
         cPoints.push_back(cRotation * cPoint);
         const auto sVelocity = cModel.GetContactSurfaceVelocity(
            cBody, cRotation * cNormal, cBody.GetCenterOfMassPosition(), cPoints);
         const bool bDriven = sVelocity.Linear.LengthSq() > 0.01f;
         if(bDriven != bDrive || (!bDrive && sVelocity.Angular.LengthSq() > 0.0f)) {
            THROW_ARGOSEXCEPTION("Unexpected Spot traction at " << pchName);
         }
      };
      Check("sole", JPH::Vec3(0, 0, 1), JPH::Vec3(0.4f, 0.2f, -0.31f), true);
      Check("sole edge", JPH::Vec3(-0.2f, 0, 0.9798f), JPH::Vec3(0.53f, 0, -0.29f), true);
      Check("wall", JPH::Vec3(-0.99f, 0, 0.1411f), JPH::Vec3(0.55f, 0, 0), false);
      Check("rounded foot edge", JPH::Vec3(-0.99f, 0, 0.1411f), JPH::Vec3(0.55f, 0, -0.29f), true);
      Check("lower leg on rough ground", JPH::Vec3(-0.9987f, 0, 0.05f), JPH::Vec3(0.53f, 0.25f, -0.21f), true);
      Check("body with upward normal", JPH::Vec3(0, 0, 1), JPH::Vec3(0.55f, 0, 0), false);
      Check("roof", JPH::Vec3(0, 0, 1), JPH::Vec3(0, 0, 0.31f), false);
      Check("inverted sole", JPH::Vec3(0, 0, -1), JPH::Vec3(0, 0, -0.31f), false);
      JPH::ContactPoints cPoints;
      auto CheckPassiveManifold = [&]() {
         const auto sVelocity = cModel.GetContactSurfaceVelocity(
            cBody, cRotation * JPH::Vec3::sAxisZ(),
            cBody.GetCenterOfMassPosition(), cPoints);
         if(sVelocity.Linear.LengthSq() != 0.0f || sVelocity.Angular.LengthSq() != 0.0f) {
            THROW_ARGOSEXCEPTION("Empty or mixed Spot manifold drives the body");
         }
      };
      CheckPassiveManifold();
      cPoints.push_back(cRotation * JPH::Vec3(0.4f, 0, -0.31f));
      cPoints.push_back(cRotation * JPH::Vec3(0.55f, 0, 0));
      CheckPassiveManifold();
      LOG << "[mesh] Spot support contact checks passed" << std::endl;
   }
};

REGISTER_LOOP_FUNCTIONS(CSupportContacts, "support_contacts");
