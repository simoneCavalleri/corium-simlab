#pragma once

// =============================================================================
// Corium SimLab — High-Performance Physical Agent Incubator Engine
// Umbrella Header
// =============================================================================

// -----------------------------------------------------------------------------
// 1. Core Engine Foundations & Events
// -----------------------------------------------------------------------------
#include "corium_sim/App.hpp"
#include "corium_sim/AssetResolver.hpp"
#include "corium_sim/Log.hpp"
#include "corium_sim/SimConfig.hpp"
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/math/Math.hpp"

// -----------------------------------------------------------------------------
// 2. Physical Agent Incubator Framework (Compile-Time C++20)
// -----------------------------------------------------------------------------
#include "corium_sim/agent/Agent.hpp"

// -----------------------------------------------------------------------------
// 3. Simulation Environments & Task Objectives
// -----------------------------------------------------------------------------
#include "corium_sim/environment/Environment.hpp"

// -----------------------------------------------------------------------------
// 4. Pluggable Physics, Kinematics & WebGPU Graphics Rendering
// -----------------------------------------------------------------------------
#include "corium_sim/kinematics/JointKinematics.hpp"
#include "corium_sim/kinematics/SimJoint.hpp"
#include "corium_sim/physics/KinematicPhysicsEngine.hpp"
#include "corium_sim/physics/PhysicsConcept.hpp"
#include "corium_sim/physics/PhysicsEngine.hpp"
#include "corium_sim/physics/Raycast.hpp"
#include "corium_sim/renderer/Camera.hpp"
#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/renderer/Mesh.hpp"
#include "corium_sim/renderer/MeshLoader.hpp"
#include "corium_sim/renderer/SensorCamera.hpp"
#include "corium_sim/renderer/Texture.hpp"
#include "corium_sim/renderer/Uniforms.hpp"
#include "corium_sim/renderer/WebGpuBackend.hpp"
#include "corium_sim/scene/SceneBuilder.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"
#include "corium_sim/scene/UrdfLoader.hpp"
