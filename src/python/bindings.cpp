#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "corium_sim/App.hpp"
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

    // 4. Main Simulation App & Gymnasium Helper Bindings
    py::class_<SimLabApp>(m, "SimLabApp")
        .def(py::init<>())
        .def("reset", &SimLabApp::resetEnvironment)
        .def("set_joint_position", [](SimLabApp& self, const std::string& name, float pos) {
            if (auto* j = self.scene().findJoint(name)) {
                j->position = pos;
            }
        }, py::arg("joint_name"), py::arg("position"))
        .def("get_sensor_frame", [](SimLabApp& self) {
            auto pixels = self.renderer().readOffscreenPixels(self.renderer().createOffscreenTarget(128, 128));
            uint32_t w = self.sensorCamera().width();
            uint32_t h = self.sensorCamera().height();

            if (pixels.empty()) {
                pixels.resize(w * h * 4, 0);
            }

            return py::bytes(reinterpret_cast<const char*>(pixels.data()), pixels.size());
        })
        .def("get_observation", [](SimLabApp& self) {
            scene::SimEntity* agent = self.scene().findEntity("agent_robot");
            if (!agent) agent = self.scene().findEntity("robot_agent");

            scene::SimEntity* target = self.scene().findEntity("target_goal");
            if (!target) target = self.scene().findEntity("target_box");

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
