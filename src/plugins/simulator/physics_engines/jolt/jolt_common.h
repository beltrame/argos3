/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_common.h>
 *
 * Includes every Jolt header used by this plugin in one place.
 * ARGoS's <argos3/core/utility/math/general.h> defines Log, Sqrt, Exp
 * and Mod as macros, which break the Jolt headers (JPH::Vec3::Sqrt()
 * and friends); the macros are suspended while Jolt is parsed. Always
 * include this header instead of Jolt headers directly, and before
 * any ARGoS header.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_COMMON_H
#define JOLT_COMMON_H

#pragma push_macro("Log")
#pragma push_macro("Sqrt")
#pragma push_macro("Exp")
#pragma push_macro("Mod")
#undef Log
#undef Sqrt
#undef Exp
#undef Mod

#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>

#pragma pop_macro("Mod")
#pragma pop_macro("Exp")
#pragma pop_macro("Sqrt")
#pragma pop_macro("Log")

#endif
