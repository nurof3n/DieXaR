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
#define INV_PI 0.31830988618f

struct Ray
{
    float3 origin;
    float3 direction;
};

inline float sq(float x)
{
    return x * x;
}

// Hash function
uint hash(uint x)
{
    x ^= x >> 16;
    x *= 0x7feb352d;
    x ^= x >> 15;
    x *= 0x846ca68b;
    x ^= x >> 16;
    return x;
}

// Convert hash to float in range [0, 1)
float uintToFloat(uint x)
{
    return x * (1.0 / 4294967296.0); // 1/2^32
}

// returns a random float in [0, 1) based on the input seed
float random(uint seed)
{
    return uintToFloat(hash(seed));
}

// returns a random float in [0, 1) using pixel coordinates and sample index
float random(uint2 xy, uint sampleIndex)
{
    uint seed = xy.x * 73856093u ^ xy.y * 19349663u ^ sampleIndex * 83492791u;
    return random(seed);
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
    return 0.5f * (Rs * Rs + Rp * Rp);
}

// Fresnel reflectance - schlick approximation.
float3 FresnelReflectanceSchlick(in float3 I, in float3 N, in float3 f0)
{
    float cosi = saturate(dot(-I, N));
    return f0 + (1 - f0) * pow(1 - cosi, 5);
}

// Fresnel reflectance - schlick approximation.
float FresnelReflectanceSchlick(in float3 I, in float3 N, in float f0)
{
    float cosi = saturate(dot(-I, N));
    return f0 + (1 - f0) * pow(1 - cosi, 5);
}

// Fresnel reflectance - schlick approximation (but with dot product given).
float3 FresnelReflectanceSchlick(in float dotNL, in float3 f0)
{
    return f0 + (1 - f0) * pow(1 - dotNL, 5);
}

// Fresnel reflectance - schlick approximation (but with dot product given).
float FresnelReflectanceSchlick(in float dotNL, in float f0)
{
    return f0 + (1 - f0) * pow(1 - dotNL, 5);
}

// Used to influence the grazing angle reflectance.
float FD90(in float roughness, in float dotWH)
{
    return 0.5f + 2.0f * roughness * sq(dotWH);
}

// Modified Fresnel equation for the diffuse model.
float FD(in float fd90, in float dotWN)
{
    return 1.0f + (fd90 - 1.0f) * pow(1.0f - abs(dotWN), 5.0f);
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
    if (a >= 1.0f || a <= -1.0f)
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
    ax = max(0.001f, roughness2 / aspect);
    ay = max(0.001f, roughness2 * aspect);
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//
// This computes the GTR2 distribution function for anisotropic materials.
// The alpha x and y values are used to control the roughness in the x and y directions (tangent space).
// Note: gamma is set to 2.
// Note: this distribution is normalized.
float DGTR2Anisotropic(in float dotHX, in float dotHY, in float dotHN, in float ax, in float ay)
{
    return INV_PI / (ax * ay * sq(sq(dotHX) / (ax * ax) + sq(dotHY) / (ay * ay) + sq(dotHN)));
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
void ComputePdfs(in float metallic, in float specularTransmission, in float clearcoat,
                 out float pMetal, out float pDiffuse, out float pClearcoat, out float pGlass,
                 out float wMetal, out float wDiffuse, out float wClearcoat, out float wGlass)
{
    wDiffuse = (1.0f - metallic) * (1.0f - specularTransmission);
    wMetal = 1.0f - specularTransmission * (1.0f - metallic);
    wClearcoat = specularTransmission * (1.0f - metallic);
    wGlass = 0.25f * clearcoat;
    float totalWeight = wDiffuse + wMetal + wClearcoat + wGlass;

    pMetal = wMetal / totalWeight;
    pDiffuse = wDiffuse / totalWeight;
    pClearcoat = wClearcoat / totalWeight;
    pGlass = wGlass / totalWeight;
}

#endif // RAYTRACINGSHADERHELPER_H