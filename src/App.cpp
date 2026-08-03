#include "corium_sim/App.hpp"
#include "corium_sim/Log.hpp"
#include "corium_sim/renderer/MeshLoader.hpp"

namespace corium_sim {

using namespace math;

SimLabApp::SimLabApp()
    : BaseApp({ .title = "Corium SimLab — Physical Agent Incubator & Simulation Engine", .width = 1280, .height = 720, .noApi = true })
{}

SimLabApp::~SimLabApp() = default;

void SimLabApp::onSetContext(corium::AppCoreContextT<SimRuntime::EventBusType> ctx)
{
    BaseApp::onSetContext(ctx);

    // Initialize WebGPU context with GLFW native handle
    GLFWwindow* nativeWin = window().backend().nativeHandle();
    _gpuBackend.initialize(
        nativeWin,
        windowConfig().width,
        windowConfig().height
    );

    _camera.setViewportSize(windowConfig().width, windowConfig().height);

    // Automatically update WebGPU surface viewport on window resize
    on([this](const corium::ui::WindowResizeEvent& evt) {
        _gpuBackend.resize(static_cast<uint32_t>(evt.width), static_cast<uint32_t>(evt.height));
        _camera.setViewportSize(static_cast<uint32_t>(evt.width), static_cast<uint32_t>(evt.height));
    });

    // Mouse Movement -> Orbit / Pan
    on([this](const corium::ui::MouseMoveEvent& evt) {
        _camera.onMouseMove(evt.x, evt.y);
    });

    // Mouse Button -> Drag Start/Stop
    on([this](const corium::ui::MouseButtonEvent& evt) {
        _camera.onMouseButton(evt.button, evt.pressed, evt.x, evt.y);
    });

    // Mouse Wheel Scroll -> Zoom
    on([this](const corium::ui::MouseScrollEvent& evt) {
        _camera.onScroll(evt.xoffset, evt.yoffset);
    });

    // Key Press / Release -> Directional Navigation & Controls
    on([this](const corium::ui::KeyEvent& evt) {
        _camera.onKey(evt.key, evt.pressed);

        if (evt.key == 256 && evt.pressed) { // GLFW_KEY_ESCAPE
            CORIUM_LOG_INFO("SimLab", "ESC key pressed, shutting down simulation environment...");
            requestQuit();
        }
    });
}

void SimLabApp::onConfigureServices(corium::ServiceRegistryT<8, DefaultSimEvents>& registry)
{
    (void)registry;
}

void SimLabApp::onRegisterHandlers()
{
    // Handler for 3D Mesh Load Request Events
    on([this](const MeshLoadEvent& evt) {
        std::cout << "[SimLab] Received MeshLoadEvent for asset: " << evt.assetPath << "\n";
        loadSceneMesh(evt.assetPath);
    });

    // Handler for Camera Positioning Command Events
    on([this](const CameraCommandEvent& evt) {
        _camera.setEyeAndTarget(
            Vec3{evt.eyeX, evt.eyeY, evt.eyeZ},
            Vec3{evt.targetX, evt.targetY, evt.targetZ}
        );
        _camera.setFov(evt.fovDegrees);
    });

    // Handler for Agent Action Commands
    on([this](const AgentActionCommand& evt) {
        if (evt.resetEpisode) {
            resetEnvironment();
            return;
        }

        scene::SimEntity* agent = scene().findEntity("robot_agent");
        if (!agent) agent = scene().findEntity("agent_robot");
        if (!agent) agent = scene().findEntity("agent");
        if (!agent) {
            for (auto& entity : scene().entities()) {
                if (!entity.isStatic) {
                    agent = &entity;
                    break;
                }
            }
        }

        if (agent) {
            float yawRad = agent->rotation.y * DEG2RAD;
            Vec3 forward{std::sin(yawRad), 0.0f, std::cos(yawRad)};

            agent->velocity += forward * (evt.moveForward * 5.0f);
            agent->angularVelocity.y = evt.turnYaw * 90.0f;
            agent->velocity.y += evt.moveUp * 5.0f;
        }
    });

    // Handler for Agent Observation Events
    on([this](const AgentObservationEvent& evt) {
        if (evt.stepIndex % 100 == 0 || evt.isTerminated || evt.isTruncated) {
            std::cout << "[SimLab Agent #" << evt.agentId << "] Step " << evt.stepIndex
                      << " | Pos: (" << evt.posX << ", " << evt.posZ << ")"
                      << " | Dist to Target: " << evt.distanceToTarget
                      << " | Reward: " << evt.reward
                      << (evt.isTerminated ? " [TARGET REACHED]" : "")
                      << (evt.isTruncated ? " [MAX STEPS EXCEEDED]" : "") << "\n";
        }
    });

    // Handler for Simulation Reset Events
    on([this](const SimResetEvent& evt) {
        (void)evt;
        resetEnvironment();
    });

    // Handler for Agent Joint Commands
    on([this](const AgentJointCommand& cmd) {
        if (auto* joint = scene().findJoint(cmd.jointName)) {
            joint->position = cmd.targetPosition;
            joint->targetVelocity = cmd.targetVelocity;
        }
    });

    // Handler for Simulation Physics / Logic Step Events
    on([this](const SimStepEvent& evt) {
        _simStepCount++;
        (void)evt;
    });
}

void SimLabApp::onInitialize()
{
    _camera.setTarget(Vec3{0.0f, 0.5f, 0.0f});
    _camera.setDistance(12.0f);

    _sensorTarget = _gpuBackend.createOffscreenTarget(128, 128);
    CORIUM_LOG_INFO("SimLab", "3D WebGPU Engine & ", _config.sensorWidth, "x", _config.sensorHeight, " Offscreen Sensor Camera Target Initialized.");

    WGPUDevice device = _gpuBackend.device();
    WGPUQueue queue = _gpuBackend.queue();
    if (device && queue) {
        if (_sceneSetupFn) {
            _sceneSetupFn(*this, device, queue);
        }

        for (auto& entity : scene().entities()) {

            if (!entity.mesh.isValid()) {
                if (entity.name.find("ground") != std::string::npos) {
                    entity.mesh = renderer::Mesh::createPlane(device, queue, 60.0f, 60.0f, 60);
                    entity.texture = renderer::Texture::createGridPattern(device, queue, 512, 512);
                    entity.material = renderer::Material::Matte({0.3f, 0.35f, 0.40f, 1.0f});
                    entity.localBounds = renderer::BoundingBox{{-30.0f, 0.0f, -30.0f}, {30.0f, 0.0f, 30.0f}};
                } else if (entity.name.find("goal") != std::string::npos || entity.name.find("target") != std::string::npos) {
                    entity.mesh = renderer::Mesh::createCube(device, queue, 1.2f);
                    entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                    entity.material = renderer::Material::Metallic({0.95f, 0.15f, 0.15f, 1.0f}, 0.2f);
                    entity.localBounds = renderer::BoundingBox{{-0.6f, -0.6f, -0.6f}, {0.6f, 0.6f, 0.6f}};
                } else {
                    entity.mesh = renderer::Mesh::createCube(device, queue, 1.0f);
                    entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                    if (entity.material.albedo.x == 0.0f && entity.material.albedo.y == 0.0f && entity.material.albedo.z == 0.0f) {

                        entity.material = renderer::Material::Metallic({0.2f, 0.6f, 0.9f, 1.0f}, 0.3f);
                    }
                    entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                }
            }
        }
        if (scene().entityCount() > 0) {
            renderer::BoundingBox bounds = scene().sceneBounds();
            _camera.focusOnBounds(bounds.min, bounds.max);
        }
    }
}

void SimLabApp::setScene(scene::SimScene scene) noexcept
{
    _environment = environment::SimEnvironment(std::move(scene));
    _jointKinematics.updateKinematics(this->scene(), 0.0f);

    WGPUDevice device = _gpuBackend.device();
    WGPUQueue queue = _gpuBackend.queue();
    if (device && queue) {
        for (auto& entity : this->scene().entities()) {
            if (!entity.mesh.isValid()) {
                if (entity.name.find("ground") != std::string::npos) {
                    entity.mesh = renderer::Mesh::createPlane(device, queue, 60.0f, 60.0f, 60);
                    entity.texture = renderer::Texture::createGridPattern(device, queue, 512, 512);
                    entity.material = renderer::Material::Matte({0.3f, 0.35f, 0.40f, 1.0f});
                    entity.localBounds = renderer::BoundingBox{{-30.0f, 0.0f, -30.0f}, {30.0f, 0.0f, 30.0f}};
                } else if (entity.name.find("goal") != std::string::npos || entity.name.find("target") != std::string::npos) {
                    entity.mesh = renderer::Mesh::createCube(device, queue, 1.2f);
                    entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                    entity.material = renderer::Material::Metallic({0.95f, 0.15f, 0.15f, 1.0f}, 0.2f);
                    entity.localBounds = renderer::BoundingBox{{-0.6f, -0.6f, -0.6f}, {0.6f, 0.6f, 0.6f}};
                } else {
                    entity.mesh = renderer::Mesh::createCube(device, queue, 1.0f);
                    entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                    if (entity.material.albedo.x == 0.0f && entity.material.albedo.y == 0.0f && entity.material.albedo.z == 0.0f) {

                        entity.material = renderer::Material::Metallic({0.2f, 0.6f, 0.9f, 1.0f}, 0.3f);
                    }
                    entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                }
            }
        }
        if (this->scene().entityCount() > 0) {
            renderer::BoundingBox bounds = this->scene().sceneBounds();
            _camera.focusOnBounds(bounds.min, bounds.max);
        }
    }
}


void SimLabApp::resetEnvironment() noexcept
{
    _simStepCount = 0;
    _episodeCount++;
    _environment.reset();

    scene::SimEntity* agent = scene().findEntity("robot_agent");
    if (!agent) agent = scene().findEntity("agent_robot");
    if (!agent) agent = scene().findEntity("agent");
    if (!agent) {
        for (auto& entity : scene().entities()) {
            if (!entity.isStatic) {
                agent = &entity;
                break;
            }
        }
    }

    if (agent) {
        agent->position = Vec3{0.0f, 0.0f, 0.0f};
        agent->rotation = Vec3{0.0f, 0.0f, 0.0f};
        agent->velocity = Vec3{0.0f, 0.0f, 0.0f};
        agent->angularVelocity = Vec3{0.0f, 0.0f, 0.0f};
    }

    CORIUM_LOG_INFO("SimLab", "Environment reset to Episode #", _episodeCount);
}

bool SimLabApp::loadSceneMesh(const std::string& filePath)
{
    WGPUDevice device = _gpuBackend.device();
    WGPUQueue queue = _gpuBackend.queue();

    if (!device || !queue) return false;

    scene().addEntity(
        scene::SimEntity{
            .id = static_cast<uint32_t>(scene().entityCount() + 1),
            .name = filePath,
            .mesh = renderer::Mesh::createFromOBJ(device, queue, filePath),
            .texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32),
            .position = Vec3{0.0f, 0.0f, 0.0f}
        }
    );

    renderer::BoundingBox bounds = scene().sceneBounds();
    _camera.focusOnBounds(bounds.min, bounds.max);
    CORIUM_LOG_INFO("SimLab", "3D Mesh asset successfully added to simulation scene!");
    return true;
}

void SimLabApp::onRender(double deltaTime)
{
    float dt = static_cast<float>(deltaTime);
    _renderFramesCount++;

    // 1. Advance Physical Environment & Joint Kinematics Step
    _environment.step(dt);
    _jointKinematics.updateKinematics(scene(), dt);
    _simStepCount++;

    // 2. Evaluate Agent State & Animate 3D Motion
    scene::SimEntity* agent = scene().findEntity("amr_leader");
    if (!agent) agent = scene().findEntity("amr_robot");
    if (!agent) agent = scene().findEntity("robot_agent");
    if (!agent) agent = scene().findEntity("agent_robot");
    if (!agent) agent = scene().findEntity("agent");
    if (!agent) {
        for (auto& entity : scene().entities()) {
            if (!entity.isStatic) {
                agent = &entity;
                break;
            }
        }
    }

    scene::SimEntity* target = scene().findEntity("target_goal");
    if (!target) target = scene().findEntity("target_box");
    if (!target) target = scene().findEntity("target_workpiece");
    if (!target) target = scene().findEntity("target_waypoint");

    if (agent && !agent->isStatic) {
        Vec3 targetPos = target ? target->position : Vec3{4.0f, 0.75f, -2.0f};
        Vec3 dir = targetPos - agent->position;
        float dist = dir.length();
        if (dist > 0.15f) {
            Vec3 moveDir = dir.normalized();
            agent->position += moveDir * (2.0f * dt);
            agent->rotation.y = std::atan2(moveDir.x, moveDir.z) * RAD2DEG;
        }
    }


    if (agent) {
        Vec3 agentPos = agent->position;
        Vec3 targetPos = target ? target->position : Vec3{4.0f, 0.75f, -2.0f};
        float dist = (agentPos - targetPos).length();

        bool isReached = (dist < _config.reachThreshold);
        bool isTruncated = (_simStepCount >= _config.maxEpisodeSteps);
        float reward = -dist + (isReached ? _config.reachBonus : 0.0f);

        // Update onboard agent visual sensor camera pose
        _sensorCamera.updateMountPose(agentPos, agent->rotation);

        // Emit zero-heap MPSC observation event
        this->eventSink().post(AgentObservationEvent{
            .agentId = agent->id,
            .posX = agentPos.x,
            .posY = agentPos.y,
            .posZ = agentPos.z,
            .velX = agent->velocity.x,
            .velY = agent->velocity.y,
            .velZ = agent->velocity.z,
            .targetX = targetPos.x,
            .targetY = targetPos.y,
            .targetZ = targetPos.z,
            .distanceToTarget = dist,
            .reward = reward,
            .isTerminated = isReached,
            .isTruncated = isTruncated,
            .stepIndex = _simStepCount
        });

        // Render onboard agent visual sensor 3D view to offscreen target
        _gpuBackend.renderOffscreen(_sensorTarget, _sensorCamera.camera(), scene());

        // Extract visual sensor frame from WebGPU staging buffer
        std::vector<uint8_t> sensorPixels = _gpuBackend.readOffscreenPixels(_sensorTarget);

        // Emit visual sensor frame event
        this->eventSink().post(SensorFrameEvent{
            .sensorId = 101,
            .width = _sensorCamera.width(),
            .height = _sensorCamera.height(),
            .stepIndex = _simStepCount,
            .rgbData = std::move(sensorPixels)
        });

        if (isReached || isTruncated) {
            std::cout << "[SimLab Agent #1] Episode completed ("
                      << (isReached ? "TARGET REACHED!" : "MAX STEPS EXCEEDED")
                      << ") -> Resetting environment...\n";
            resetEnvironment();
        }
    }

    // 3. Update camera WASD keyboard navigation
    _camera.update(dt);

    if (!_gpuBackend.beginFrame(_camera)) {
        return;
    }

    // 4. Render all active 3D entities in simulation scene
    for (const auto& entity : scene().entities()) {
        if (entity.mesh.isValid()) {
            _gpuBackend.drawMesh(entity.mesh, entity.texture, entity.transformMatrix(), entity.material);
        }
    }

    _gpuBackend.endFrame();
}

void SimLabApp::onShutdown()
{
    _gpuBackend.destroyOffscreenTarget(_sensorTarget);
    scene().destroy();
    _gpuBackend.shutdown();

    CORIUM_LOG_INFO("SimLab", "Application Shutdown Summary:");
    CORIUM_LOG_INFO("SimLab", "  - Total Simulation Episodes: ", _episodeCount);
    CORIUM_LOG_INFO("SimLab", "  - Total Simulation Steps:    ", _simStepCount);
    CORIUM_LOG_INFO("SimLab", "  - Total Render Frames:       ", _renderFramesCount);
}

} // namespace corium_sim
