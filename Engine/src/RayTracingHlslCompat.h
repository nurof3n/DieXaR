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

#ifndef RAYTRACINGHLSLCOMPAT_H
#define RAYTRACINGHLSLCOMPAT_H

//**********************************************************************************************
//
// RaytracingHLSLCompat.h
//
// A header with shared definitions for C++ and HLSL source files.
//
//**********************************************************************************************

#ifdef HLSL
#include "util\HlslCompat.h"
#else
using namespace DirectX;

// Shader will use byte encoding to access vertex indices.
typedef UINT16 Index;
#endif

// Number of metaballs to use within an AABB.
#define N_METABALLS 3  // = {3, 5}

// Limiting calculations only to metaballs a ray intersects can speed up raytracing
// dramatically particularly when there is a higher number of metaballs used.
// Use of dynamic loops can have detrimental effects to performance for low iteration counts
// and outweighing any potential gains from avoiding redundant calculations.
// Requires: USE_DYNAMIC_LOOPS set to 1 to take effect.
#if N_METABALLS >= 5
#define USE_DYNAMIC_LOOPS         1
#define LIMIT_TO_ACTIVE_METABALLS 1
#else
#define USE_DYNAMIC_LOOPS         0
#define LIMIT_TO_ACTIVE_METABALLS 0
#endif

#define N_FRACTAL_ITERATIONS 4  // = <1,...>

struct ProceduralPrimitiveAttributes
{
    XMFLOAT3 normal;
};

struct RayPayload
{
    XMFLOAT4 color;           // Accumulated color value of the ray.
    XMFLOAT4 worldPosition;   // World position of the hit. w = 1 for hit, 0 for miss.
    XMFLOAT4 throughput;      // Accumulated throughput of the ray. Only used in path tracing.
    XMFLOAT4 absorption;      // Accumulated absorption of the ray. Only used in path tracing.
    UINT     rngState;        // Random number generator state.
    UINT     recursionDepth;  // Current recursion depth of the ray.
    UINT     inside;          // Inside primitive flag.
    float    bsdfPdf;         // Probability density function of the last BSDF sample.
};

struct ShadowRayPayload
{
    bool hit;
};

struct LightBuffer
{
    XMFLOAT3 position;
    float    intensity;
    XMFLOAT3 emission;
    float    size;       // radius for spheres, side length for squares
    XMFLOAT3 direction;  // for directional light
    UINT     type;       // 0: square area light, 1: directional light
};

struct SceneConstantBuffer
{
    XMMATRIX projectionToWorld;
    XMVECTOR cameraPosition;
    XMFLOAT4 backgroundColor;
    UINT     numLights;
    float    elapsedTime;                // Elapsed application time.
    UINT     elapsedTicks;               // Elapsed application time in ticks.
    UINT     raytracingType;             // Raytracing type to use.
    UINT     importanceSamplingType;     // Importance sampling type to use.
    UINT     maxRecursionDepth;          // Max recursion depth for the raytracing.
    UINT     maxShadowRecursionDepth;    // Max recursion depth for casting shadow rays
    UINT     pathSqrtSamplesPerPixel;    // Number of samples per pixel for path tracing.
    UINT     pathFrameCacheIndex;        // Current frame index for temporal path tracing.
    UINT     applyJitter;                // Apply jitter to the ray sampling (useful in path tracing only).
    UINT     onlyOneLightSample;         // Use only one light sample at a time.
    UINT     russianRouletteDepth;       // Max depth for Russian roulette termination.
    UINT     anisotropicBSDF;            // Use anisotropic BSDF.
    UINT     sceneIndex;                 // Scene index to render.
    XMMATRIX worldToProjection;          // World to projection matrix for motion vectors.
    XMMATRIX previousWorldToProjection;  // Previous frame's world to projection matrix for motion vectors.
};

// Attributes per primitive type.
struct PrimitiveConstantBuffer
{
    XMFLOAT4 albedo;
    UINT     materialIndex;
    float    reflectanceCoef;
    float    diffuseCoef;
    float    specularCoef;
    float    specularPower;
    float    stepScale;  // Step scale for ray marching of signed distance primitives.
    // - Some object transformations don't preserve the distances and
    //   thus require shorter steps.
    XMFLOAT2 padding;
};

// Attributes per primitive type, but physically based rendering version.
struct PBRPrimitiveConstantBuffer
{
    XMFLOAT4 albedo;
    XMFLOAT4 emission;       // Emission color
    XMFLOAT3 extinction;     // Extinction color (absorption + scattering)
    UINT     materialIndex;  // 0 = ground
    float    stepScale;      // Step scale for ray marching of signed distance primitives.

    // All the following values are in the range [0, 1].
    float sheen;                 // Standalone specular lobe (used for cloth materials)
    float sheenTint;             // Sheen tint factor (0 = white, 1 = color influence)
    float clearcoat;             // Clearcoat layer intensity
    float clearcoatGloss;        // Clearcoat layer glossiness
    float roughness;             // Microfacet roughness
    float subsurface;            // Subsurface scattering factor
    float anisotropic;           // Anisotropic factor
    float metallic;              // Metallic factor
    float specularTint;          // Specular tint factor
    float specularTransmission;  // Specular transmission factor
    float eta;                   // Fresnel eta factor: internal / external IOR (assumes 1.0 for air)
    float atDistance;            // Distance at which the transmittance is
};

// Attributes per primitive instance.
struct PrimitiveInstanceConstantBuffer
{
    XMMATRIX transform;      // Matrix to transform the primitive from object space to world space.
    UINT     instanceIndex;  // Index of the primitive instance.
    UINT     primitiveType;  // Procedural primitive type, corresponds to SignedDistancePrimitive::Enum or
                             // AnalyticPrimitive::Enum for AABBs
    // or (0=triangle, 1=square light) for triangle primitives.
    XMFLOAT2 padding;
};

// Dynamic attributes per primitive instance.
struct PrimitiveInstancePerFrameBuffer
{
    XMMATRIX localSpaceToBottomLevelAS;  // Matrix from local primitive space to bottom-level object space.
    XMMATRIX bottomLevelASToLocalSpace;  // Matrix from bottom-level object space to local primitive space.
};

struct Vertex
{
    XMFLOAT3 position;
    XMFLOAT3 normal;
};

namespace SceneTypes
{
    enum SceneType
    {
        CornellBox,
        Demo,
        PbrShowcase,
        Count
    };
};

namespace LightType
{
    enum Enum
    {
        Square = 0,
        Directional,
        Count
    };
}

// Ray tracing types.
namespace RaytracingType
{
    enum Enum
    {
        Whitted = 0,
        PathTracing,
        PathTracingTemporal,
        Count
    };
}

// Sampling types.
namespace ImportanceSamplingType
{
    enum Enum
    {
        Uniform = 0,
        Cosine,
        BSDF,
        Count
    };
}

// Ray types traced in this sample.
namespace RayType
{
    enum Enum
    {
        Radiance = 0,  // ~ Primary, reflected camera/view rays calculating color for each hit.
        Shadow,        // ~ Shadow/visibility rays, only testing for occlusion
        Count
    };
}

namespace TraceRayParameters
{
    static const UINT InstanceMask = ~0;  // Everything is visible.
    namespace HitGroup
    {
        static const UINT Offset[RayType::Count] = {
            0,  // Radiance ray
            1   // Shadow ray
        };
        static const UINT GeometryStride = RayType::Count;
    }  // namespace HitGroup
    namespace MissShader
    {
        static const UINT Offset[RayType::Count] = {
            0,  // Radiance ray
            1   // Shadow ray
        };
    }
}  // namespace TraceRayParameters

static const float InShadowRadiance = 0.00f;

namespace AnalyticPrimitive
{
    enum Enum
    {
        AABB = 0,
        Spheres,
        Count
    };
}

namespace SignedDistancePrimitive
{
    enum Enum
    {
        // MiniSpheres = 0,
        IntersectedRoundCube,
        SquareTorus,
        Cog,
        Cylinder,
        SolidAngle,
        Count
    };
}

#endif  // RAYTRACINGHLSLCOMPAT_H