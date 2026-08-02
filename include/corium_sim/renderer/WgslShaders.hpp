#pragma once

namespace corium_sim::renderer {

/// @brief Embedded WGSL 3D PBR (Physically Based Rendering) Shader Source Code.
inline constexpr const char* WGSL_PBR_SHADER_SOURCE = R"(
struct Uniforms {
    model: mat4x4<f32>,
    viewProj: mat4x4<f32>,
    lightDir: vec4<f32>,
    lightColor: vec4<f32>,
    cameraPos: vec4<f32>,
    ambientColor: vec4<f32>,
    albedoColor: vec4<f32>,
    materialParams: vec4<f32>, // x=metallic, y=roughness, z=emissive
    time: f32,
};

@group(0) @binding(0) var<uniform> ubo: Uniforms;
@group(0) @binding(1) var textureSampler: sampler;
@group(0) @binding(2) var textureData: texture_2d<f32>;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clipPosition: vec4<f32>,
    @location(0) worldPos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) color: vec4<f32>,
};

const PI: f32 = 3.14159265359;

fn distributionGGX(N: vec3<f32>, H: vec3<f32>, roughness: f32) -> f32 {
    let a = roughness * roughness;
    let a2 = a * a;
    let NdotH = max(dot(N, H), 0.0);
    let NdotH2 = NdotH * NdotH;
    let num = a2;
    let denom = (NdotH2 * (a2 - 1.0) + 1.0);
    return num / (PI * denom * denom);
}

fn geometrySchlickGGX(NdotV: f32, roughness: f32) -> f32 {
    let r = (roughness + 1.0);
    let k = (r * r) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}

fn geometrySmith(N: vec3<f32>, V: vec3<f32>, L: vec3<f32>, roughness: f32) -> f32 {
    let NdotV = max(dot(N, V), 0.0);
    let NdotL = max(dot(N, L), 0.0);
    let ggx2 = geometrySchlickGGX(NdotV, roughness);
    let ggx1 = geometrySchlickGGX(NdotL, roughness);
    return ggx1 * ggx2;
}

fn fresnelSchlick(cosTheta: f32, F0: vec3<f32>) -> vec3<f32> {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let worldPos4 = ubo.model * vec4<f32>(in.position, 1.0);
    out.worldPos = worldPos4.xyz;
    out.clipPosition = ubo.viewProj * worldPos4;
    
    let normalMatrix = mat3x3<f32>(ubo.model[0].xyz, ubo.model[1].xyz, ubo.model[2].xyz);
    out.normal = normalize(normalMatrix * in.normal);
    
    out.uv = in.uv;
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let N = normalize(in.normal);
    let V = normalize(ubo.cameraPos.xyz - in.worldPos);
    let L = normalize(ubo.lightDir.xyz);
    let H = normalize(V + L);

    let texColor = textureSample(textureData, textureSampler, in.uv);
    let albedo = in.color.rgb * ubo.albedoColor.rgb * texColor.rgb;
    let metallic = clamp(ubo.materialParams.x, 0.0, 1.0);
    let roughness = clamp(ubo.materialParams.y, 0.05, 1.0);
    let emissive = ubo.materialParams.z;

    var F0 = vec3<f32>(0.04);
    F0 = mix(F0, albedo, metallic);

    // Cook-Torrance BRDF
    let NDF = distributionGGX(N, H, roughness);
    let G = geometrySmith(N, V, L, roughness);
    let F = fresnelSchlick(max(dot(H, V), 0.0), F0);

    let numerator = NDF * G * F;
    let denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
    let specular = numerator / denominator;

    let kS = F;
    var kD = vec3<f32>(1.0) - kS;
    kD *= 1.0 - metallic;

    let NdotL = max(dot(N, L), 0.0);
    let radiance = ubo.lightColor.rgb;

    let Lo = (kD * albedo / PI + specular) * radiance * NdotL;
    let ambient = ubo.ambientColor.rgb * albedo;
    let emissiveCol = albedo * emissive;

    let color = ambient + Lo + emissiveCol;

    // Tone mapping and gamma correction
    let mapped = color / (color + vec3<f32>(1.0));
    let finalColor = pow(mapped, vec3<f32>(1.0 / 2.2));

    return vec4<f32>(finalColor, in.color.a * texColor.a * ubo.albedoColor.a);
}
)";

} // namespace corium_sim::renderer
