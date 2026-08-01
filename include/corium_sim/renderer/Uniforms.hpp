#pragma once

#include "corium_sim/math/Math.hpp"

namespace corium_sim::renderer {

/// @brief WebGPU Uniform Buffer Object layout (208 bytes, 16-byte aligned for WGSL mat4/vec4).
struct alignas(16) UniformBufferObject {
    math::Mat4 model{math::Mat4::identity()};       // Offset 0   (64 bytes)
    math::Mat4 viewProj{math::Mat4::identity()};    // Offset 64  (64 bytes)
    math::Vec4 lightDir{0.577f, 0.577f, 0.577f, 0.0f}; // Offset 128 (16 bytes)
    math::Vec4 lightColor{1.0f, 0.95f, 0.9f, 1.0f}; // Offset 144 (16 bytes)
    math::Vec4 cameraPos{0.0f, 5.0f, 10.0f, 1.0f};  // Offset 160 (16 bytes)
    math::Vec4 ambientColor{0.2f, 0.22f, 0.25f, 1.0f}; // Offset 176 (16 bytes)
    float time{0.0f};                               // Offset 192 (4 bytes)
    float padding[3]{0.0f, 0.0f, 0.0f};             // Offset 196 (12 bytes padding)
};

static_assert(sizeof(UniformBufferObject) % 16 == 0, "UniformBufferObject size must be a multiple of 16 bytes for WebGPU WGSL alignment!");

} // namespace corium_sim::renderer
