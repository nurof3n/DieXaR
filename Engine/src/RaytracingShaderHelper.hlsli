//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#ifndef RAYTRACINGSHADERHELPER_H
#define RAYTRACINGSHADERHELPER_H

#include "RayTracingHlslCompat.h"

#define INFINITY (1.0 / 0.0)
#define PI 3.14159265359f
#define INV_PI 0.31830988618f

struct Ray
{
    float3 origin;
    float3 direction;
};

// Returns a 32-bit hash of a float2 and a seed.
uint hash(uint2 p, uint seed)
{
    p = (p >> 16u) ^ p;
    p = (p * 0x45d9f3b) ^ seed;
    p = (p >> 16u) ^ p;
    p = (p * 0x45d9f3b) ^ seed;
    p = (p >> 16u) ^ p;
    return p.x;
}

uint hash(float3 vec)
{
    return hash(uint2(asuint(vec.x), asuint(vec.y)), asuint(vec.z));
}

// PRNG
// https://www.reedbeta.com/blog/hash-functions-for-gpu-rendering/
uint rand_pcg(inout uint rng_state)
{
    uint state = rng_state;
    rng_state = rng_state * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

float random(inout uint rng_state)
{
    return float(rand_pcg(rng_state)) / float(0xFFFFFFFFu);
}

inline float sq(float x)
{
    return x * x;
}

float length_toPow2(float2 p)
{
    return dot(p, p);
}

float length_toPow2(float3 p)
{
    return dot(p, p);
}

// Returns a cycling <0 -> 1 -> 0> animation interpolant
float CalculateAnimationInterpolant(in float elapsedTime, in float cycleDuration)
{
    float curLinearCycleTime = fmod(elapsedTime, cycleDuration) / cycleDuration;
    curLinearCycleTime = (curLinearCycleTime <= 0.5f) ? 2 * curLinearCycleTime : 1 - 2 * (curLinearCycleTime - 0.5f);
    return smoothstep(0, 1, curLinearCycleTime);
}

void swap(inout float a, inout float b)
{
    float temp = a;
    a = b;
    b = temp;
}

bool IsInRange(in float val, in float min, in float max)
{
    return (val >= min && val <= max);
}

// Load three 16 bit indices.
static uint3 Load3x16BitIndices(uint offsetBytes, ByteAddressBuffer Indices)
{
    uint3 indices;

    // ByteAdressBuffer loads must be aligned at a 4 byte boundary.
    // Since we need to read three 16 bit indices: { 0, 1, 2 }
    // aligned at a 4 byte boundary as: { 0 1 } { 2 0 } { 1 2 } { 0 1 } ...
    // we will load 8 bytes (~ 4 indices { a b | c d }) to handle two possible index triplet layouts,
    // based on first index's offsetBytes being aligned at the 4 byte boundary or not:
    //  Aligned:     { 0 1 | 2 - }
    //  Not aligned: { - 0 | 1 2 }
    const uint dwordAlignedOffset = offsetBytes & ~3;
    const uint2 four16BitIndices = Indices.Load2(dwordAlignedOffset);

    // Aligned: { 0 1 | 2 - } => retrieve first three 16bit indices
    if (dwordAlignedOffset == offsetBytes)
    {
        indices.x = four16BitIndices.x & 0xffff;
        indices.y = (four16BitIndices.x >> 16) & 0xffff;
        indices.z = four16BitIndices.y & 0xffff;
    }
    else // Not aligned: { - 0 | 1 2 } => retrieve last three 16bit indices
    {
        indices.x = (four16BitIndices.x >> 16) & 0xffff;
        indices.y = four16BitIndices.y & 0xffff;
        indices.z = (four16BitIndices.y >> 16) & 0xffff;
    }

    return indices;
}

// Retrieve hit world position.
float3 HitWorldPosition()
{
    return WorldRayOrigin() + RayTCurrent() * WorldRayDirection();
}

// Retrieve attribute at a hit position interpolated from vertex attributes using the hit's barycentrics.
float3 HitAttribute(float3 vertexAttribute[3], float2 barycentrics)
{
    return vertexAttribute[0] +
           barycentrics.x * (vertexAttribute[1] - vertexAttribute[0]) +
           barycentrics.y * (vertexAttribute[2] - vertexAttribute[0]);
}

// Generate a ray in world space for a camera pixel corresponding to an index from the dispatched 2D grid.
inline Ray GenerateCameraRay(uint2 index, in float3 cameraPosition, in float4x4 projectionToWorld, in float2 offset)
{
    // Apply offset to the pixel.
    float2 xy = index + offset;

    // Normalize the pixel coordinate to [-1, 1].
    float2 screenPos = xy / DispatchRaysDimensions().xy * 2.0f - 1.0f;

    // Invert Y for DirectX-style coordinates.
    screenPos.y = -screenPos.y;

    // Unproject the pixel coordinate into a world positon.
    float4 world = mul(float4(screenPos, 0, 1), projectionToWorld);
    world.xyz /= world.w;

    Ray ray;
    ray.origin = cameraPosition;
    ray.direction = normalize(world.xyz - ray.origin);

    return ray;
}

// Test if a hit is culled based on specified RayFlags.
bool IsCulled(in Ray ray, in float3 hitSurfaceNormal)
{
    float rayDirectionNormalDot = dot(ray.direction, hitSurfaceNormal);

    bool isCulled =
        ((RayFlags() & RAY_FLAG_CULL_BACK_FACING_TRIANGLES) && (rayDirectionNormalDot > 0)) ||
        ((RayFlags() & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES) && (rayDirectionNormalDot < 0));

    return isCulled;
}

// Test if a hit is valid based on specified RayFlags and <RayTMin, RayTCurrent> range.
bool IsAValidHit(in Ray ray, in float thit, in float3 hitSurfaceNormal)
{
    return IsInRange(thit, RayTMin(), RayTCurrent()) && !IsCulled(ray, hitSurfaceNormal);
}

// Texture coordinates on a horizontal plane.
float2 TexCoords(in float3 position)
{
    return position.xz;
}

void ComputeLocalSpace(in float3 normal, out float3 tangent, out float3 bitangent)
{
    if (abs(normal.y) < 0.999f)
    {
        tangent = normalize(cross(float3(0.0f, 1.0f, 0.0f), normal));
        bitangent = cross(normal, tangent);
    }
    else
    {
        tangent = normalize(cross(float3(1.0f, 0.0f, 0.0f), normal));
        bitangent = cross(normal, tangent);
    }
}

float R0FromIOR(float ior)
{
    return sq((1.0f - ior) / (1.0f + ior));
}

// The Fresnel reflectance for a dielectric material.
float FresnelDielectric(in float dotHL, in float dotHV, float eta)
{
    float Rs = (dotHV - eta * dotHL) / (dotHV + eta * dotHL);
    float Rp = (eta * dotHV - dotHL) / (eta * dotHV + dotHL);
    return 0.5f * (sq(Rs) + sq(Rp));
}

// Fresnel reflectance - schlick approximation.
float3 FresnelReflectanceSchlick(in float3 I, in float3 N, in float3 f0)
{
    float cosi = saturate(dot(-I, N));
    return f0 + (1 - f0) * pow(saturate(1.0f - cosi), 5);
}

// Fresnel reflectance - schlick approximation.
float FresnelReflectanceSchlick(in float3 I, in float3 N, in float f0)
{
    float cosi = saturate(dot(-I, N));
    return f0 + (1 - f0) * pow(saturate(1.0f - cosi), 5);
}

// Fresnel reflectance - schlick approximation (but with dot product given).
float3 FresnelReflectanceSchlick(in float dotNL, in float3 f0)
{
    return f0 + (1 - f0) * pow(saturate(1.0f - dotNL), 5);
}

// Fresnel reflectance - schlick approximation (but with dot product given).
float FresnelReflectanceSchlick(in float dotNL, in float f0)
{
    return f0 + (1.0f - f0) * pow(saturate(1.0f - dotNL), 5);
}

float3 DisneyFresnel(in float dotLH, in float3 albedo, in float specularTint, in float specular, in float eta, in float metallic)
{
    // compute Fresnel achromatic component (to account for dielectric specular reflection)
    // C0 can be tinted to the base color
    float luminance = dot(albedo, float3(0.3f, 0.6f, 1.0f));
    float3 Ks = luminance > 0.0f ? lerp(float3(1.0f, 1.0f, 1.0f), albedo / luminance, specularTint) : float3(1.0f, 1.0f, 1.0f); // same as sheen but different tint parameter
    float3 C0 = lerp(Ks * specular * R0FromIOR(eta), albedo, metallic);

    // compute Fresnel term
    return FresnelReflectanceSchlick(dotLH, C0);
}

// Used to influence the grazing angle reflectance.
float FD90(in float roughness, in float dotWH)
{
    return 0.5f + 2.0f * roughness * sq(dotWH);
}

// Modified Fresnel equation for the diffuse model.
float FD(in float fd90, in float dotWN)
{
    return 1.0f + (fd90 - 1.0f) * pow(1.0f - abs(dotWN), 5);
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//
// This represents the microfacet distribution function used for the clear coat layer.
// This is an outdated version (GTR2 comes with improvements) but the clear coat
// is an artistic effect anyway, so it's fine to use this.
//
// The alpha term is a roughness parameter that controls the distribution.
// A value of 0 produces a smooth distribution (a Dirac delta function),
// while a value of 1 produces a perfectly uniform (rough) distribution.
//
// Note: gamma is set to 1.
// Note: this distribution is normalized.
float DGTR1(in float dotNH, in float a)
{
    // clamp the roughness
    if (abs(a) >= 1.0f)
        return INV_PI;

    float a2 = a * a;
    return INV_PI * (a2 - 1.0f) / (log(a2) * (1.0f + (a2 - 1.0f) * sq(dotNH)));
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//
// This computes the alpha x and y values for the anisotropic distribution of GTR2.
// (the variations of the roughness in the x and y directions in tangent space).
void ComputeAnisotropicAlphas(in float roughness, in float anisotropic, out float ax, out float ay)
{
    float aspect = sqrt(1.0f - 0.9f * anisotropic); // limits the aspect ratio to 10:1
    float roughness2 = roughness * roughness;
    ax = max(0.0001f, roughness2 / aspect);
    ay = max(0.0001f, roughness2 * aspect);
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//
// This computes the GTR2 distribution function for anisotropic materials.
// The alpha x and y values are used to control the roughness in the x and y directions (tangent space).
// Note: gamma is set to 2.
// Note: this distribution is normalized.
float DGTR2Anisotropic(in float dotHX, in float dotHY, in float dotHN, in float ax, in float ay)
{
    return INV_PI / (ax * ay * sq(sq(dotHX / ax * ax) + sq(dotHY / ay) + sq(dotHN)));
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//      and https://www.jcgt.org/published/0003/02/03/paper.pdf
//      and https://sayan1an.github.io/pdfs/references/disneyBrdf.pdf
//
// This computes the unidirectional (separable) G1 masking function for anisotropic materials.
// The alpha x and y values are the aniostropic roughness parameters.
// This function is used to account for shadowing and masking effects in the microfacet model,
// to prevent energy conservation issues.
float SmithG1Anisotropic(in float dotWX, in float dotWY, in float dotWN, float ax, float ay)
{
    float inv_a2 = (sq(dotWX * ax) + sq(dotWY * ay)) / sq(dotWN);
    float lambda = -0.5f + 0.5f * sqrt(1.0f + inv_a2);
    return 1.0f / (1.0f + lambda);
}

// Computes the lobes' probability distribution functions for the microfacet model.
// Ignores the sheen lobe because its influence is minimal.
void ComputePdfs(bool inside, in float metallic, in float specularTransmission, in float clearcoat,
                 out float pMetal, out float pDiffuse, out float pClearcoat, out float pGlass,
                 out float wMetal, out float wDiffuse, out float wClearcoat, out float wGlass)
{
    if (!inside)
    {
        wDiffuse = (1.0f - metallic) * (1.0f - specularTransmission);
        wMetal = 1.0f - specularTransmission * (1.0f - metallic);
        wGlass = specularTransmission * (1.0f - metallic);
        wClearcoat = 0.25f * clearcoat;
    }
    else
    {
        wDiffuse = 0.0f;
        wMetal = 0.0f;
        wGlass = 1.0f;
        wClearcoat = 0.0f;
    }

    float totalWeight = wDiffuse + wMetal + wClearcoat + wGlass;

    pMetal = wMetal / totalWeight;
    pDiffuse = wDiffuse / totalWeight;
    pClearcoat = wClearcoat / totalWeight;
    pGlass = wGlass / totalWeight;
}

// Uniformly samples a hemisphere.
// eps0 and eps1 are random numbers in [0, 1).
// phi = 2 * PI * eps1
// theta = acos(1 - eps0)
// pdf = 1 / (2 * PI)
// y is the up vector.
float3 UniformSampleHemisphere(in float eps0, in float eps1, in float3 normal, in float3 T, float3 B, out float pdf)
{
    float sinTheta = sqrt(1.0f - eps0 * eps0);
    float phi = 2.0f * PI * eps1;
    float3 direction;
    direction.x = sinTheta * cos(phi);
    direction.y = eps0;
    direction.z = sinTheta * sin(phi);

    // Compute the pdf.
    pdf = 0.5f * INV_PI;

    // Transform the direction to the hemisphere's normal.
    return normalize(direction.x * T + direction.y * normal + direction.z * B);
}

// Cosine-weighted hemisphere sampling.
// eps0 and eps1 are random numbers in [0, 1).
// phi = 2 * PI * eps1
// theta = acos(sqrt(eps0))
// pdf = cos(theta) / PI
// y is the up vector.
float3 CosineSampleHemisphere(in float eps0, in float eps1, in float3 normal, in float3 T, in float3 B, out float pdf)
{
    float cosTheta = sqrt(eps0);
    float sinTheta = sqrt(1.0f - eps0);
    float phi = 2.0f * PI * eps1;
    float3 direction;
    direction.x = sinTheta * cos(phi);
    direction.y = cosTheta;
    direction.z = sinTheta * sin(phi);

    // Compute the pdf.
    pdf = INV_PI * cosTheta;

    // Transform the direction to the hemisphere's normal.
    return normalize(direction.x * T + direction.y * normal + direction.z * B);
}

// https://hal.science/hal-01509746/document
float3 VisibleNormalsSampling(in float eps0, in float eps1, in float ax, in float ay, in float3 V)
{
    // Stretch the view vector to match the roughness 1.
    float3 stretchedV = normalize(float3(ax * V.x, V.y, ay * V.z));

    // Compute an orthonormal basis.
    float3 T, B;
    ComputeLocalSpace(stretchedV, T, B);

    // Sample point with polar coordinates (r, phi).
    float a = 1.0f / (1.0f + stretchedV.y);
    float r = sqrt(eps0);
    float phi = eps1 < a ? eps1 / a * PI : PI + (eps1 - a) / (1.0f - a) * PI;
    float p1 = r * cos(phi);
    float p2 = r * sin(phi) * (eps1 < a ? 1.0f : stretchedV.y);

    // Compute the normal in the tangent space.
    float3 normal = normalize(p1 * T + p2 * B + sqrt(max(0.0f, 1.0f - sq(p1) - sq(p2))) * stretchedV);
    return float3(ax * normal.x, normal.y, ay * normal.z);
}

float VisibleNormalsPdf(in float ax, in float ay, in float dotLH, in float dotVX, in float dotVY, in float dotVN,
    in float dotHX, in float dotHY, in float dotHN)
{
    float G1 = SmithG1Anisotropic(dotVX, dotVY, dotVN, ax, ay);
    float D = DGTR2Anisotropic(dotHX, dotHY, dotHN, ax, ay);
    return D * G1 * abs(dotLH);
}

float PowerHeuristic(float nf, float fPdf, float ng, float gPdf)
{
    float f = nf * fPdf;
    float g = ng * gPdf;
    return sq(f) / (sq(f) + sq(g));
}

#endif // RAYTRACINGSHADERHELPER_H