#include "corium_sim/scene/UrdfLoader.hpp"
#include "corium_sim/Log.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace corium_sim::scene {

using namespace math;

static Vec3 parseVec3(const std::string& str) {
    Vec3 res{0.0f, 0.0f, 0.0f};
    std::stringstream ss(str);
    ss >> res.x >> res.y >> res.z;
    return res;
}

static Vec4 parseVec4(const std::string& str) {
    Vec4 res{1.0f, 1.0f, 1.0f, 1.0f};
    std::stringstream ss(str);
    ss >> res.x >> res.y >> res.z >> res.w;
    return res;
}

static std::string extractAttribute(const std::string& tag, const std::string& attrName) {
    std::string pattern = attrName + "=\"";
    size_t pos = tag.find(pattern);
    if (pos == std::string::npos) return "";
    size_t start = pos + pattern.length();
    size_t end = tag.find("\"", start);
    if (end == std::string::npos) return "";
    return tag.substr(start, end - start);
}

bool UrdfLoader::loadURDF(
    const std::string& urdfFilePath,
    SimScene& scene,
    WGPUDevice device,
    WGPUQueue queue,
    const Vec3& basePosition,
    const Vec3& baseScale
) {
    std::ifstream file(urdfFilePath);
    if (!file.is_open()) {
        CORIUM_LOG_ERROR("UrdfLoader", "Failed to open URDF file: ", urdfFilePath);
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string xml = buffer.str();

    uint32_t entityId = static_cast<uint32_t>(scene.entityCount() + 1);

    // 1. Parse Links
    size_t linkPos = 0;
    while ((linkPos = xml.find("<link", linkPos)) != std::string::npos) {
        size_t linkEnd = xml.find("</link>", linkPos);
        if (linkEnd == std::string::npos) break;

        std::string linkBlock = xml.substr(linkPos, linkEnd - linkPos + 7);
        std::string name = extractAttribute(linkBlock, "name");

        Vec3 pos = basePosition;
        Vec3 boxScale{1.0f, 1.0f, 1.0f};
        Vec4 color{0.8f, 0.8f, 0.85f, 1.0f};

        // Origin
        size_t originPos = linkBlock.find("<origin");
        if (originPos != std::string::npos) {
            std::string xyzStr = extractAttribute(linkBlock.substr(originPos), "xyz");
            if (!xyzStr.empty()) {
                pos = basePosition + parseVec3(xyzStr);
            }
        }

        // Geometry (Box)
        size_t boxPos = linkBlock.find("<box");
        if (boxPos != std::string::npos) {
            std::string sizeStr = extractAttribute(linkBlock.substr(boxPos), "size");
            if (!sizeStr.empty()) {
                boxScale = parseVec3(sizeStr);
            }
        }

        // Color
        size_t colorPos = linkBlock.find("<color");
        if (colorPos != std::string::npos) {
            std::string rgbaStr = extractAttribute(linkBlock.substr(colorPos), "rgba");
            if (!rgbaStr.empty()) {
                color = parseVec4(rgbaStr);
            }
        }

        if (!name.empty()) {
            SimEntity entity{};
            entity.id = entityId++;
            entity.name = name;
            if (device && queue) {
                entity.mesh = renderer::Mesh::createCube(device, queue, 1.0f);
                entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
            }
            entity.material = renderer::Material::Metallic(color, 0.25f);
            entity.position = pos;
            entity.scale = Vec3{boxScale.x * baseScale.x, boxScale.y * baseScale.y, boxScale.z * baseScale.z};
            entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};

            scene.addEntity(std::move(entity));
        }

        linkPos = linkEnd + 7;
    }

    // 2. Parse Joints
    size_t jointPos = 0;
    while ((jointPos = xml.find("<joint", jointPos)) != std::string::npos) {
        size_t jointEnd = xml.find("</joint>", jointPos);
        if (jointEnd == std::string::npos) break;

        std::string jointBlock = xml.substr(jointPos, jointEnd - jointPos + 8);
        std::string jointName = extractAttribute(jointBlock, "name");
        std::string typeStr = extractAttribute(jointBlock, "type");

        std::string parentName;
        std::string childName;
        Vec3 origin{0.0f, 0.0f, 0.0f};
        Vec3 axis{0.0f, 1.0f, 0.0f};
        float minLimit = -3.14159f;
        float maxLimit = 3.14159f;

        size_t parentPos = jointBlock.find("<parent");
        if (parentPos != std::string::npos) {
            parentName = extractAttribute(jointBlock.substr(parentPos), "link");
        }

        size_t childPos = jointBlock.find("<child");
        if (childPos != std::string::npos) {
            childName = extractAttribute(jointBlock.substr(childPos), "link");
        }

        size_t jOriginPos = jointBlock.find("<origin");
        if (jOriginPos != std::string::npos) {
            std::string xyzStr = extractAttribute(jointBlock.substr(jOriginPos), "xyz");
            if (!xyzStr.empty()) origin = parseVec3(xyzStr);
        }

        size_t axisPos = jointBlock.find("<axis");
        if (axisPos != std::string::npos) {
            std::string xyzStr = extractAttribute(jointBlock.substr(axisPos), "xyz");
            if (!xyzStr.empty()) axis = parseVec3(xyzStr);
        }

        size_t limitPos = jointBlock.find("<limit");
        if (limitPos != std::string::npos) {
            std::string lowerStr = extractAttribute(jointBlock.substr(limitPos), "lower");
            std::string upperStr = extractAttribute(jointBlock.substr(limitPos), "upper");
            if (!lowerStr.empty()) minLimit = std::stof(lowerStr);
            if (!upperStr.empty()) maxLimit = std::stof(upperStr);
        }

        kinematics::JointType type = kinematics::JointType::Revolute;
        if (typeStr == "prismatic") type = kinematics::JointType::Prismatic;
        else if (typeStr == "fixed") type = kinematics::JointType::Fixed;

        if (!jointName.empty() && !childName.empty()) {
            kinematics::SimJoint joint{};
            joint.name = jointName;
            joint.parentName = parentName;
            joint.childName = childName;
            joint.type = type;
            joint.anchor = origin;
            joint.axis = axis;
            joint.minLimit = minLimit;
            joint.maxLimit = maxLimit;

            scene.addJoint(std::move(joint));
        }

        jointPos = jointEnd + 8;
    }

    CORIUM_LOG_INFO("UrdfLoader", "Successfully loaded URDF robot specification: ", urdfFilePath);
    return true;
}

} // namespace corium_sim::scene
