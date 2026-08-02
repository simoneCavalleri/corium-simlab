#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <fstream>

#include "corium_sim/App.hpp"
#include "corium_sim/SimConfig.hpp"
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/scene/SceneBuilder.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace py = pybind11;
using namespace corium_sim;
using namespace corium_sim::math;
using namespace corium_sim::renderer;
using namespace corium_sim::scene;

PYBIND11_MODULE(corium_sim_py, m) {
    m.doc() = "Corium SimLab — High-Performance 3D Agent Simulation & WebGPU Visualizer Engine Extension";

    // 1. Math Bindings
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<float, float, float>(), py::arg("x") = 0.0f, py::arg("y") = 0.0f, py::arg("z") = 0.0f)
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("length", &Vec3::length)
        .def("__repr__", [](const Vec3& v) {
            return "<corium_sim.Vec3 (" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")>";
        });

    py::class_<Vec4>(m, "Vec4")
        .def(py::init<float, float, float, float>(), py::arg("x") = 0.0f, py::arg("y") = 0.0f, py::arg("z") = 0.0f, py::arg("w") = 1.0f)
        .def_readwrite("x", &Vec4::x)
        .def_readwrite("y", &Vec4::y)
        .def_readwrite("z", &Vec4::z)
        .def_readwrite("w", &Vec4::w);

    // 2. Material System Bindings
    py::class_<Material>(m, "Material")
        .def(py::init<Vec4, float, float, float>(),
             py::arg("albedo") = Vec4{0.8f, 0.8f, 0.8f, 1.0f},
             py::arg("metallic") = 0.1f,
             py::arg("roughness") = 0.5f,
             py::arg("emissive") = 0.0f)
        .def_readwrite("albedo", &Material::albedo)
        .def_readwrite("metallic", &Material::metallic)
        .def_readwrite("roughness", &Material::roughness)
        .def_readwrite("emissive", &Material::emissive)
        .def_static("Metallic", &Material::Metallic, py::arg("color") = Vec4{0.9f, 0.91f, 0.92f, 1.0f}, py::arg("roughness") = 0.25f)
        .def_static("Matte", &Material::Matte, py::arg("color"))
        .def_static("Glossy", &Material::Glossy, py::arg("color"));

    // 3. Joint Kinematics Bindings
    py::enum_<kinematics::JointType>(m, "JointType")
        .value("Revolute", kinematics::JointType::Revolute)
        .value("Prismatic", kinematics::JointType::Prismatic)
        .value("Fixed", kinematics::JointType::Fixed);

    py::class_<kinematics::SimJoint>(m, "SimJoint")
        .def(py::init<>())
        .def_readwrite("id", &kinematics::SimJoint::id)
        .def_readwrite("name", &kinematics::SimJoint::name)
        .def_readwrite("parent_name", &kinematics::SimJoint::parentName)
        .def_readwrite("child_name", &kinematics::SimJoint::childName)
        .def_readwrite("type", &kinematics::SimJoint::type)
        .def_readwrite("anchor", &kinematics::SimJoint::anchor)
        .def_readwrite("axis", &kinematics::SimJoint::axis)
        .def_readwrite("position", &kinematics::SimJoint::position)
        .def_readwrite("min_limit", &kinematics::SimJoint::minLimit)
        .def_readwrite("max_limit", &kinematics::SimJoint::maxLimit)
        .def_readwrite("target_velocity", &kinematics::SimJoint::targetVelocity);

    // 4. SimEntity & SimScene Bindings
    py::class_<SimEntity>(m, "SimEntity")
        .def(py::init<>())
        .def_readwrite("id", &SimEntity::id)
        .def_readwrite("name", &SimEntity::name)
        .def_readwrite("position", &SimEntity::position)
        .def_readwrite("rotation", &SimEntity::rotation)
        .def_readwrite("scale", &SimEntity::scale)
        .def_readwrite("velocity", &SimEntity::velocity)
        .def_readwrite("angular_velocity", &SimEntity::angularVelocity)
        .def_readwrite("mass", &SimEntity::mass)
        .def_readwrite("is_static", &SimEntity::isStatic)
        .def_readwrite("has_physics", &SimEntity::hasPhysics)
        .def_readwrite("material", &SimEntity::material);

    py::class_<SimScene>(m, "SimScene")
        .def(py::init<>())
        .def("entity_count", &SimScene::entityCount)
        .def("destroy", &SimScene::destroy);

    // 5. SimConfig Bindings
    py::class_<SimConfig>(m, "SimConfig")
        .def(py::init<>())
        .def_readwrite("max_episode_steps", &SimConfig::maxEpisodeSteps)
        .def_readwrite("sensor_width", &SimConfig::sensorWidth)
        .def_readwrite("sensor_height", &SimConfig::sensorHeight)
        .def_readwrite("reach_threshold", &SimConfig::reachThreshold)
        .def_readwrite("reach_bonus", &SimConfig::reachBonus)
        .def_readwrite("enable_gravity", &SimConfig::enableGravity)
        .def_readwrite("gravity", &SimConfig::gravity)
        .def_readwrite("fixed_timestep", &SimConfig::fixedTimestep);

    // 6. Main Simulation App & Gymnasium Helper Bindings
    py::class_<SimLabApp>(m, "SimLabApp")
        .def(py::init<>())
        .def("reset", &SimLabApp::resetEnvironment)
        .def_property("config",
            [](SimLabApp& self) -> SimConfig& { return self.config(); },
            [](SimLabApp& self, const SimConfig& cfg) { self.setConfig(cfg); })
        .def("setup_default_scene", [](SimLabApp& self) {
            if (!self.renderer().isInitialized()) {
                self.renderer().initialize(nullptr, 1280, 720);
            }
            WGPUDevice device = self.renderer().device();
            WGPUQueue queue = self.renderer().queue();
            self.setScene(
                scene::SimScene::builder(device, queue)
                    .addGroundGrid(50.0f, 50.0f, 50)
                    .addModel("agent_robot", "assets/models/sample_robot.obj", Vec3{0.0f, 0.0f, 0.0f})
                    .addCube("target_goal", Vec3{4.0f, 0.75f, -2.0f}, Vec3{1.2f, 1.2f, 1.2f})
                    .addSphere("obstacle_ball", Vec3{-3.0f, 1.0f, 2.0f}, 1.0f)
                    .build()
            );
        }, "Set up the default 3D RL environment scene (ground grid, agent, target, obstacle).")
        .def("setup_robotic_arm_scene", [](SimLabApp& self) {
            if (!self.renderer().isInitialized()) {
                self.renderer().initialize(nullptr, 1280, 720);
            }
            WGPUDevice device = self.renderer().device();
            WGPUQueue queue = self.renderer().queue();
            self.setScene(
                scene::SimScene::builder(device, queue)
                    .addGroundGrid(60.0f, 60.0f, 60)
                    .addCube("workstation_table", Vec3{0.0f, 0.4f, 0.0f}, Vec3{3.0f, 0.8f, 2.0f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Metallic({0.4f, 0.45f, 0.50f, 1.0f}, 0.35f), true)
                    .addModel("agent_robot", "assets/models/sample_robot.obj", Vec3{0.0f, 0.8f, 0.0f}, Vec3{0.8f, 0.8f, 0.8f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Metallic({0.95f, 0.95f, 0.98f, 1.0f}, 0.15f), true)
                    .addCube("shoulder_link", Vec3{0.0f, 1.4f, 0.0f}, Vec3{0.4f, 1.0f, 0.4f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Glossy({0.15f, 0.55f, 0.95f, 1.0f}))
                    .addCube("elbow_link", Vec3{0.0f, 2.3f, 0.0f}, Vec3{0.3f, 0.8f, 0.3f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Glossy({0.95f, 0.85f, 0.10f, 1.0f}))
                    .addCube("wrist_link", Vec3{0.0f, 2.9f, 0.0f}, Vec3{0.25f, 0.4f, 0.25f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Metallic({0.85f, 0.88f, 0.90f, 1.0f}, 0.25f))
                    .addCube("gripper_finger_l", Vec3{-0.15f, 3.2f, 0.0f}, Vec3{0.08f, 0.3f, 0.12f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Matte({0.2f, 0.2f, 0.25f, 1.0f}))
                    .addCube("gripper_finger_r", Vec3{0.15f, 3.2f, 0.0f}, Vec3{0.08f, 0.3f, 0.12f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Matte({0.2f, 0.2f, 0.25f, 1.0f}))
                    .addJoint("joint_shoulder_yaw", "agent_robot", "shoulder_link", kinematics::JointType::Revolute, Vec3{0.0f, 0.6f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, -3.14159f, 3.14159f)
                    .addJoint("joint_elbow_pitch", "shoulder_link", "elbow_link", kinematics::JointType::Revolute, Vec3{0.0f, 0.9f, 0.0f}, Vec3{1.0f, 0.0f, 0.0f}, -2.0944f, 2.0944f)
                    .addJoint("joint_wrist_roll", "elbow_link", "wrist_link", kinematics::JointType::Revolute, Vec3{0.0f, 0.6f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, -3.14159f, 3.14159f)
                    .addJoint("joint_gripper_l", "wrist_link", "gripper_finger_l", kinematics::JointType::Prismatic, Vec3{-0.15f, 0.3f, 0.0f}, Vec3{-1.0f, 0.0f, 0.0f}, 0.0f, 0.1f)
                    .addJoint("joint_gripper_r", "wrist_link", "gripper_finger_r", kinematics::JointType::Prismatic, Vec3{0.15f, 0.3f, 0.0f}, Vec3{1.0f, 0.0f, 0.0f}, 0.0f, 0.1f)
                    .addCube("inspection_platform", Vec3{3.5f, 0.5f, -1.5f}, Vec3{2.0f, 1.0f, 1.5f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Metallic({0.5f, 0.52f, 0.55f, 1.0f}, 0.40f), true)
                    .addCube("target_workpiece", Vec3{3.5f, 1.25f, -1.5f}, Vec3{0.6f, 0.5f, 0.6f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Metallic({0.95f, 0.10f, 0.15f, 1.0f}, 0.10f))
                    .addCube("safety_barrier", Vec3{-2.5f, 0.6f, 2.5f}, Vec3{3.5f, 1.2f, 0.2f}, Vec3{0.0f, 0.0f, 0.0f}, Material::Matte({0.95f, 0.80f, 0.05f, 1.0f}), true)
                    .build()
            );
        }, "Set up the 3D industrial robotic manipulator workstation scene.")
        .def("load_scene_mesh", &SimLabApp::loadSceneMesh, py::arg("obj_file_path"), "Load an OBJ 3D mesh model into the scene.")
        .def("set_joint_position", [](SimLabApp& self, const std::string& name, float pos) {
            if (auto* j = self.scene().findJoint(name)) {
                j->position = pos;
            }
        }, py::arg("joint_name"), py::arg("position"))
        .def("apply_action", [](SimLabApp& self, float moveForward, float turnYaw, float moveUp) {
            scene::SimEntity* agent = self.scene().findEntity("agent_robot");
            if (!agent) agent = self.scene().findEntity("robot_agent");

            if (agent) {
                float yawRad = agent->rotation.y * DEG2RAD;
                Vec3 forward{std::sin(yawRad), 0.0f, std::cos(yawRad)};

                agent->velocity += forward * (moveForward * 5.0f);
                agent->angularVelocity.y = turnYaw * 90.0f;
                agent->velocity.y += moveUp * 5.0f;
            }
        }, py::arg("move_forward") = 0.0f, py::arg("turn_yaw") = 0.0f, py::arg("move_up") = 0.0f,
        "Apply movement action to the agent: forward/backward, yaw rotation, vertical.")
        .def("sim_step", [](SimLabApp& self, float dt) {
            self.physics().step(self.scene(), dt);
            self.jointKinematics().updateKinematics(self.scene(), dt);
        }, py::arg("dt") = 0.016667f,
        "Advance physics and kinematics simulation by one timestep.")
        .def("solve_ik", [](SimLabApp& self, const std::string& endEffectorName, float targetX, float targetY, float targetZ, uint32_t maxIterations, float tolerance) {
            return self.jointKinematics().solveIK(self.scene(), endEffectorName, Vec3{targetX, targetY, targetZ}, maxIterations, tolerance);
        }, py::arg("end_effector_name"), py::arg("target_x"), py::arg("target_y"), py::arg("target_z"), py::arg("max_iterations") = 50, py::arg("tolerance") = 0.01f,
        "Solve Inverse Kinematics to move specified end-effector link to target (x, y, z) position.")
        .def("get_sensor_frame", [](SimLabApp& self) {
            auto fetch_pixels = [](SimLabApp& app) -> std::vector<uint8_t> {
                if (!app.renderer().isInitialized()) {
                    app.renderer().initialize(nullptr, 1280, 720);
                }
                scene::SimEntity* agent = app.scene().findEntity("agent_robot");
                if (!agent) agent = app.scene().findEntity("robot_agent");
                if (agent) {
                    app.sensorCamera().updateMountPose(agent->position, agent->rotation);
                }

                auto target = app.renderer().createOffscreenTarget(128, 128);
                app.renderer().renderOffscreen(target, app.sensorCamera().camera(), app.scene());
                auto pixels = app.renderer().readOffscreenPixels(target);
                app.renderer().destroyOffscreenTarget(target);
                return pixels;
            };

            auto pixels = fetch_pixels(self);
            uint32_t w = self.sensorCamera().width();
            uint32_t h = self.sensorCamera().height();

            if (pixels.empty()) {
                pixels.resize(w * h * 4, 0);
            }

            return py::bytes(reinterpret_cast<const char*>(pixels.data()), pixels.size());
        })
        .def("save_sensor_frame_ppm", [](SimLabApp& self, const std::string& filename) -> bool {
            if (!self.renderer().isInitialized()) {
                self.renderer().initialize(nullptr, 1280, 720);
            }
            scene::SimEntity* agent = self.scene().findEntity("agent_robot");
            if (!agent) agent = self.scene().findEntity("robot_agent");
            if (agent) {
                self.sensorCamera().updateMountPose(agent->position, agent->rotation);
            }

            auto target = self.renderer().createOffscreenTarget(128, 128);
            self.renderer().renderOffscreen(target, self.sensorCamera().camera(), self.scene());
            auto pixels = self.renderer().readOffscreenPixels(target);
            self.renderer().destroyOffscreenTarget(target);

            uint32_t w = self.sensorCamera().width();
            uint32_t h = self.sensorCamera().height();

            std::ofstream out(filename, std::ios::binary);
            if (!out.is_open()) return false;

            out << "P6\n" << w << " " << h << "\n255\n";
            for (size_t i = 0; i + 3 < pixels.size(); i += 4) {
                out.put(static_cast<char>(pixels[i]));     // R
                out.put(static_cast<char>(pixels[i + 1])); // G
                out.put(static_cast<char>(pixels[i + 2])); // B
            }
            return true;
        }, py::arg("filename"), "Save current onboard camera view directly as a PPM image file.")
        .def("get_observation", [](SimLabApp& self) {
            scene::SimEntity* agent = self.scene().findEntity("agent_robot");
            if (!agent) agent = self.scene().findEntity("robot_agent");

            scene::SimEntity* target = self.scene().findEntity("target_goal");
            if (!target) target = self.scene().findEntity("target_box");
            if (!target) target = self.scene().findEntity("target_workpiece");

            py::dict obs;
            if (agent && target) {
                float dist = (agent->position - target->position).length();
                bool isReached = (dist < 1.5f);
                float reward = -dist + (isReached ? 100.0f : 0.0f);

                obs["agent_pos"] = std::vector<float>{agent->position.x, agent->position.y, agent->position.z};
                obs["agent_vel"] = std::vector<float>{agent->velocity.x, agent->velocity.y, agent->velocity.z};
                obs["target_pos"] = std::vector<float>{target->position.x, target->position.y, target->position.z};
                obs["distance"] = dist;
                obs["reward"] = reward;
                obs["terminated"] = isReached;
            }
            return obs;
        });
}

