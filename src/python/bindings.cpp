#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <fstream>

#include "corium_sim/App.hpp"
#include "corium_sim/SimConfig.hpp"
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/events/SimEventTracer.hpp"
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/scene/SceneBuilder.hpp"
#include "corium_sim/scene/SimScene.hpp"
#include "corium_sim/scene/UrdfLoader.hpp"
#include "corium_sim/physics/Raycast.hpp"
#include "corium_sim/environment/Environment.hpp"
#include "corium_sim/environment/SimEnvironment.hpp"
#include "corium_sim/environment/Task.hpp"


namespace py = pybind11;
using namespace corium_sim;
using namespace corium_sim::math;
using namespace corium_sim::renderer;
using namespace corium_sim::scene;
using namespace corium_sim::environment;
using namespace corium_sim::physics;


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

    py::class_<SceneBuilder>(m, "SceneBuilder")
        .def("add_ground_grid", &SceneBuilder::addGroundGrid,
             py::arg("width") = 50.0f, py::arg("depth") = 50.0f, py::arg("subdivisions") = 50,
             py::return_value_policy::reference)
        .def("add_cube", &SceneBuilder::addCube,
             py::arg("name"), py::arg("position") = Vec3{0,0,0}, py::arg("scale") = Vec3{1,1,1},
             py::arg("rotation") = Vec3{0,0,0}, py::arg("material") = Material{},
             py::arg("is_static") = false, py::arg("has_physics") = true,
             py::return_value_policy::reference)
        .def("add_sphere", &SceneBuilder::addSphere,
             py::arg("name"), py::arg("position") = Vec3{0,0,0}, py::arg("radius") = 0.5f,
             py::arg("scale") = Vec3{1,1,1}, py::arg("material") = Material{},
             py::arg("is_static") = false, py::arg("has_physics") = true,
             py::return_value_policy::reference)
        .def("add_model", &SceneBuilder::addModel,
             py::arg("name"), py::arg("obj_file_path"), py::arg("position") = Vec3{0,0,0},
             py::arg("scale") = Vec3{1,1,1}, py::arg("rotation") = Vec3{0,0,0},
             py::arg("material") = Material{}, py::arg("is_static") = false, py::arg("has_physics") = true,
             py::return_value_policy::reference)
        .def("add_joint", &SceneBuilder::addJoint,
             py::arg("name"), py::arg("parent_name"), py::arg("child_name"),
             py::arg("type") = kinematics::JointType::Revolute,
             py::arg("anchor") = Vec3{0,0,0}, py::arg("axis") = Vec3{0,1,0},
             py::arg("min_limit") = -3.14159f, py::arg("max_limit") = 3.14159f,
             py::return_value_policy::reference)
        .def("add_urdf", &SceneBuilder::addURDF,
             py::arg("urdf_file_path"), py::arg("base_position") = Vec3{0,0,0}, py::arg("base_scale") = Vec3{1,1,1},
             py::return_value_policy::reference)
        .def("build", &SceneBuilder::build);



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
        .def("set_scene", [](SimLabApp& self, SimScene& scene) {
            self.setScene(std::move(scene));
        }, py::arg("scene"), "Set active 3D simulation environment scene.")

        .def("create_scene_builder", [](SimLabApp& self) {
            if (!self.renderer().isInitialized()) {
                self.renderer().initialize(nullptr, 1280, 720);
            }
            return SimScene::builder(self.renderer().device(), self.renderer().queue());
        }, "Create a fluent SceneBuilder to construct user-defined 3D environment scenes.")
        .def("load_scene_mesh", &SimLabApp::loadSceneMesh, py::arg("obj_file_path"), "Load an OBJ 3D mesh model into the scene.")


        .def("load_urdf", [](SimLabApp& self, const std::string& urdfFilePath) {
            if (!self.renderer().isInitialized()) {
                self.renderer().initialize(nullptr, 1280, 720);
            }
            return scene::UrdfLoader::loadURDF(urdfFilePath, self.scene(), self.renderer().device(), self.renderer().queue());
        }, py::arg("urdf_file_path"), "Parse and import a URDF XML robot specification model file.")
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
            // Note: physics stepping is handled by SimEnvironment / CoriumEnv.
            // SimLabApp is a pure rendering shell — only kinematics (joint FK) is advanced here.
            self.jointKinematics().updateKinematics(self.scene(), dt);
        }, py::arg("dt") = 0.016667f,
        "Advance joint kinematics simulation by one timestep (physics is owned by CoriumEnv).")

        .def("solve_ik", [](SimLabApp& self, const std::string& endEffectorName, float targetX, float targetY, float targetZ, uint32_t maxIterations, float tolerance) {
            return self.jointKinematics().solveIK(self.scene(), endEffectorName, Vec3{targetX, targetY, targetZ}, maxIterations, tolerance);
        }, py::arg("end_effector_name"), py::arg("target_x"), py::arg("target_y"), py::arg("target_z"), py::arg("max_iterations") = 50, py::arg("tolerance") = 0.01f,
        "Solve Inverse Kinematics to move specified end-effector link to target (x, y, z) position.")
        .def("cast_ray", [](SimLabApp& self, float origX, float origY, float origZ, float dirX, float dirY, float dirZ, float maxDist) {
            auto hit = physics::Raycast::castRay(self.scene(), Vec3{origX, origY, origZ}, Vec3{dirX, dirY, dirZ}, maxDist);
            py::dict res;
            res["hit"] = hit.hit;
            res["distance"] = hit.distance;
            res["point"] = std::vector<float>{hit.point.x, hit.point.y, hit.point.z};
            res["normal"] = std::vector<float>{hit.normal.x, hit.normal.y, hit.normal.z};
            res["entity"] = hit.entityName;
            return res;
        }, py::arg("orig_x"), py::arg("orig_y"), py::arg("orig_z"), py::arg("dir_x"), py::arg("dir_y"), py::arg("dir_z"), py::arg("max_distance") = 50.0f,
        "Cast a 3D ray into the scene and return hit dictionary (hit, distance, point, normal, entity).")
        .def("cast_lidar_scan", [](SimLabApp& self, float origX, float origY, float origZ, uint32_t numRays, float fovDeg, float maxDist) {
            auto hits = physics::Raycast::castLidarScan(self.scene(), Vec3{origX, origY, origZ}, Vec3{0.0f, 0.0f, 1.0f}, numRays, fovDeg, maxDist);
            py::list results;
            for (const auto& hit : hits) {
                py::dict item;
                item["hit"] = hit.hit;
                item["distance"] = hit.distance;
                item["point"] = std::vector<float>{hit.point.x, hit.point.y, hit.point.z};
                item["normal"] = std::vector<float>{hit.normal.x, hit.normal.y, hit.normal.z};
                item["entity"] = hit.entityName;
                results.append(item);
            }
            return results;
        }, py::arg("orig_x") = 0.0f, py::arg("orig_y") = 1.0f, py::arg("orig_z") = 0.0f, py::arg("num_rays") = 36, py::arg("fov_deg") = 360.0f, py::arg("max_distance") = 20.0f,
        "Perform a 360-degree 3D LiDAR point cloud scan around sensor origin and return list of hit dicts.")
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
            if (!agent) agent = self.scene().findEntity("agent");
            if (!agent) {
                for (auto& entity : self.scene().entities()) {
                    if (!entity.isStatic) {
                        agent = &entity;
                        break;
                    }
                }
            }

            scene::SimEntity* target = self.scene().findEntity("target_goal");
            if (!target) target = self.scene().findEntity("target_box");
            if (!target) target = self.scene().findEntity("target_workpiece");

            py::dict obs;
            if (agent) {
                Vec3 agentPos = agent->position;
                Vec3 targetPos = target ? target->position : Vec3{4.0f, 0.75f, -2.0f};
                float dist = (agentPos - targetPos).length();
                bool isReached = (dist < 1.5f);
                float reward = -dist + (isReached ? 100.0f : 0.0f);

                obs["agent_pos"] = std::vector<float>{agentPos.x, agentPos.y, agentPos.z};
                obs["agent_vel"] = std::vector<float>{agent->velocity.x, agent->velocity.y, agent->velocity.z};
                obs["target_pos"] = std::vector<float>{targetPos.x, targetPos.y, targetPos.z};
                obs["distance"] = dist;
                obs["reward"] = reward;
                obs["terminated"] = isReached;
            }
            return obs;
        });

    // 6. Physical Environment & Raycast LiDAR Bindings
    py::class_<TaskStepResult>(m, "TaskStepResult")
        .def(py::init<>())
        .def_readwrite("reward", &TaskStepResult::reward)
        .def_readwrite("done", &TaskStepResult::done)
        .def_readwrite("truncated", &TaskStepResult::truncated)
        .def_readwrite("info", &TaskStepResult::info);

    py::class_<DefaultSimEnvironment>(m, "SimEnvironment")
        .def(py::init<>())
        .def("step", &DefaultSimEnvironment::step, py::arg("dt") = 0.01667f)
        .def("reset", &DefaultSimEnvironment::reset)
        .def("evaluate_task", &DefaultSimEnvironment::evaluateTask)
        .def("current_step", &DefaultSimEnvironment::currentStep)
        .def("elapsed_time", &DefaultSimEnvironment::elapsedTime);


    py::class_<DomainRandomizationConfig>(m, "DomainRandomizationConfig")
        .def(py::init<>())
        .def_readwrite("enable_pose_randomization", &DomainRandomizationConfig::enablePoseRandomization)
        .def_readwrite("position_std_dev", &DomainRandomizationConfig::positionStdDev)
        .def_readwrite("rotation_std_dev", &DomainRandomizationConfig::rotationStdDev)
        .def_readwrite("enable_physics_randomization", &DomainRandomizationConfig::enablePhysicsRandomization)
        .def_readwrite("mass_scale_range", &DomainRandomizationConfig::massScaleRange)
        .def_readwrite("friction_scale_range", &DomainRandomizationConfig::frictionScaleRange)
        .def_readwrite("enable_sensor_noise", &DomainRandomizationConfig::enableSensorNoise)
        .def_readwrite("sensor_noise_std_dev", &DomainRandomizationConfig::sensorNoiseStdDev);

    py::class_<DomainRandomizer>(m, "DomainRandomizer")
        .def(py::init<>())
        .def(py::init<DomainRandomizationConfig>(), py::arg("config"))
        .def("randomize_scene", &DomainRandomizer::randomizeScene, py::arg("scene"))
        .def_property("config",
            [](DomainRandomizer& self) -> DomainRandomizationConfig& { return self.config(); },
            [](DomainRandomizer& self, const DomainRandomizationConfig& cfg) { self.config() = cfg; });

    py::class_<RewardBuilder>(m, "RewardBuilder")
        .def(py::init<>())
        .def("add_distance_penalty", [](RewardBuilder& self, float weight) {
            return self.addTerm<DistanceToGoalPenalty>(weight);
        }, py::arg("weight") = 1.0f, py::return_value_policy::reference)
        .def("add_goal_bonus", [](RewardBuilder& self, float bonus, float threshold) {
            return self.addTerm<GoalReachedBonus>(bonus, threshold);
        }, py::arg("bonus") = 100.0f, py::arg("threshold") = 0.5f, py::return_value_policy::reference)
        .def("add_action_smoothing_penalty", [](RewardBuilder& self, float weight) {
            return self.addTerm<ActionSmoothingPenalty>(weight);
        }, py::arg("weight") = 0.01f, py::return_value_policy::reference)
        .def("add_collision_penalty", [](RewardBuilder& self, float penalty) {
            return self.addTerm<CollisionPenalty>(penalty);
        }, py::arg("penalty") = 50.0f, py::return_value_policy::reference)
        .def("compute_reward", [](RewardBuilder& self, const Vec3& agentPos, const Vec3& targetPos, const std::vector<float>& action, bool isCollided) {
            return self.computeTotalReward(agentPos, targetPos, action, isCollided);
        }, py::arg("agent_pos"), py::arg("target_pos"), py::arg("action"), py::arg("is_collided") = false);

    py::class_<RaycastHit>(m, "RaycastHit")
        .def(py::init<>())
        .def_readwrite("hit", &RaycastHit::hit)
        .def_readwrite("distance", &RaycastHit::distance)
        .def_readwrite("point", &RaycastHit::point)
        .def_readwrite("normal", &RaycastHit::normal)
        .def_readwrite("entity_name", &RaycastHit::entityName);

    m.def("cast_ray", &Raycast::castRay,
          py::arg("scene"), py::arg("origin"), py::arg("direction"), py::arg("max_distance") = 50.0f);


    m.def("cast_lidar_scan", &Raycast::castLidarScan,
          py::arg("scene"), py::arg("origin"), py::arg("forward_dir") = Vec3{0.0f, 0.0f, 1.0f},
          py::arg("num_rays") = 36, py::arg("fov_degrees") = 360.0f, py::arg("max_distance") = 20.0f);

    // 7. Event Tracing & Telemetry Bindings
    py::class_<events::TraceEntry>(m, "TraceEntry")
        .def(py::init<>())
        .def_readwrite("step_index", &events::TraceEntry::stepIndex)
        .def_readwrite("timestamp_ms", &events::TraceEntry::timestampMs)
        .def_readwrite("event_type", &events::TraceEntry::eventType)
        .def_readwrite("summary", &events::TraceEntry::summary)
        .def_readwrite("details_json", &events::TraceEntry::detailsJson);

    py::class_<events::SimEventTracer>(m, "SimEventTracer")
        .def(py::init<>())
        .def("start_tracing", &events::SimEventTracer::startTracing)
        .def("stop_tracing", &events::SimEventTracer::stopTracing)
        .def("is_tracing", &events::SimEventTracer::isTracing)
        .def("clear", &events::SimEventTracer::clear)
        .def("trace_count", &events::SimEventTracer::traceCount)
        .def("traces", &events::SimEventTracer::traces)
        .def("log_summary", &events::SimEventTracer::logSummary)
        .def("export_to_json", &events::SimEventTracer::exportToJson, py::arg("file_path"))
        .def("register_with_app", [](events::SimEventTracer& self, SimLabApp& app) {
            self.registerWith(app);
        }, py::arg("app"));
}


