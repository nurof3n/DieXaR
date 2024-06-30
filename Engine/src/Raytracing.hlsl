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

#ifndef RAYTRACING_HLSL
#define RAYTRACING_HLSL

#define HLSL
#include "RaytracingHlslCompat.h"
#include "ProceduralPrimitivesLibrary.hlsli"
#include "RaytracingShaderHelper.hlsli"
#include "DisneyBSDF.hlsli"

//***************************************************************************
//*****------ Shader resources bound via root signatures -------*************
//***************************************************************************

// Scene wide resources.
//  g_* - bound via a global root signature.
//  l_* - bound via a local root signature.
RaytracingAccelerationStructure g_scene : register(t0, space0);
RWTexture2D<float4> g_renderTarget : register(u0);
ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);
StructuredBuffer<LightBuffer> g_lights : register(t4, space0);

// Triangle resources
ByteAddressBuffer g_indices : register(t1, space0);
StructuredBuffer<Vertex> g_vertices : register(t2, space0);

// Procedural geometry resources
StructuredBuffer<PrimitiveInstancePerFrameBuffer> g_AABBPrimitiveAttributes : register(t3, space0);
ConstantBuffer<PrimitiveConstantBuffer> l_materialCB : register(b1); // Sample material constant buffer
ConstantBuffer<PrimitiveInstanceConstantBuffer> l_aabbCB : register(b2);
ConstantBuffer<PBRPrimitiveConstantBuffer> l_pbrCB : register(b3); // PBR material constant buffer

//***************************************************************************
//*****------ TraceRay wrappers for radiance and shadow rays. -------********
//***************************************************************************

// Trace a radiance ray into the scene and returns a shaded color.
float4 TraceRadianceRay(in Ray ray, in float4 throughput, in float4 absorption, in UINT currentRayRecursionDepth, in bool refraction = false)
{
    if (currentRayRecursionDepth >= g_sceneCB.maxRecursionDepth)
    {
        return float4(0, 0, 0, 0);
    }

    // Set the ray's extents.
    RayDesc rayDesc;
    rayDesc.Origin = ray.origin;
    rayDesc.Direction = ray.direction;
    // Set TMin to a zero value to avoid aliasing artifacts along contact areas.
    // Note: make sure to enable face culling so as to avoid surface face fighting.
    rayDesc.TMin = refraction ? 0.01f : 0.0001f;
    rayDesc.TMax = 10000.0f;
    RayPayload rayPayload = {float4(0.0f, 0.0f, 0.0f, 0.0f), throughput, absorption, currentRayRecursionDepth + 1};

    uint flag = refraction ? RAY_FLAG_CULL_FRONT_FACING_TRIANGLES : RAY_FLAG_CULL_BACK_FACING_TRIANGLES;
    TraceRay(g_scene,
             flag,
             TraceRayParameters::InstanceMask,
             TraceRayParameters::HitGroup::Offset[RayType::Radiance],
             TraceRayParameters::HitGroup::GeometryStride,
             TraceRayParameters::MissShader::Offset[RayType::Radiance],
             rayDesc, rayPayload);

    return rayPayload.color;
}

// Trace a shadow ray and return true if it hits any geometry.
bool TraceShadowRayAndReportIfHit(in Ray ray, in float lightDist, in UINT currentRayRecursionDepth)
{
    if (currentRayRecursionDepth >= g_sceneCB.maxRecursionDepth || currentRayRecursionDepth >= g_sceneCB.maxShadowRecursionDepth)
    {
        return false;
    }

    // Set the ray's extents.
    RayDesc rayDesc;
    rayDesc.Origin = ray.origin;
    rayDesc.Direction = ray.direction;
    // Set TMin to a zero value to avoid aliasing artifacts along contact areas.
    // Note: make sure to enable back-face culling so as to avoid surface face fighting.
    rayDesc.TMin = 0.0001f;
    rayDesc.TMax = lightDist * 0.999999f; // Avoid self-intersection.

    // Initialize shadow ray payload.
    // Set the initial value to true since closest and any hit shaders are skipped.
    // Shadow miss shader, if called, will set it to false.
    ShadowRayPayload shadowPayload = {true};
    TraceRay(g_scene,
             RAY_FLAG_CULL_BACK_FACING_TRIANGLES | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_FORCE_OPAQUE // ~skip any hit shaders
                 | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,                                                                // ~skip closest hit shaders,
             TraceRayParameters::InstanceMask,
             TraceRayParameters::HitGroup::Offset[RayType::Shadow],
             TraceRayParameters::HitGroup::GeometryStride,
             TraceRayParameters::MissShader::Offset[RayType::Shadow],
             rayDesc, shadowPayload);

    return shadowPayload.hit;
}

// ##################################################################### //
// ####################### Path Tracing functions ###################### //
// ##################################################################### //

struct LightSample
{
    float3 L;
    float3 emission;
    float dist;
    float pdf;
};

// Used for MIS.
float PowerHeuristic(float nf, float fPdf, float ng, float gPdf)
{
    float f = nf * fPdf;
    float sqf = sq(f);
    float g = ng * gPdf;
    return sqf / (sqf + sq(g));
}

// Samples a light source with a given direction.
//TODO unused
bool NextEventEstimation(inout uint rng_state, in float3 L, in LightBuffer light, in float3 hitPosition,
                         in uint recursionDepth, out LightSample lightSample)
{
    float angle;
    switch (light.type)
    {
    case LightType::Square:
        lightSample.L = L;
        lightSample.dist = length(lightSample.L);
        lightSample.L /= lightSample.dist;
        lightSample.emission = light.intensity * light.emission;
        angle = dot(lightSample.L, float3(0.0f, 1.0f, 0.0f));
        if (angle <= 0.0f)
            return false;
        lightSample.pdf = (sq(lightSample.dist)) / (sq(light.size) * angle);
        break;
    case LightType::Directional:
        return false;
    }

    // Check if the light is occluded.
    Ray shadowRay = {hitPosition, lightSample.L};
    bool shadowRayHit = recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, lightSample.dist, recursionDepth);
    return !shadowRayHit;
}

// Samples a light source.
bool NextEventEstimation(inout uint rng_state, in LightBuffer light, in float3 hitPosition, in float3 normal,
                         in uint recursionDepth, out LightSample lightSample)
{
    float angle;
    float2 eps;
    float3 samplePos;
    switch (light.type)
    {
    case LightType::Square:
        eps = float2(random(rng_state), random(rng_state));
        samplePos = light.position + float3(eps.x - 0.5f, 0.0f, eps.y - 0.5f) * light.size;
        lightSample.L = samplePos - hitPosition;
        lightSample.dist = length(lightSample.L);
        lightSample.L /= lightSample.dist;
        // bail out if the sample is on the wrong side of the light
        angle = dot(lightSample.L, float3(0.0f, 1.0f, 0.0f));
        if (angle <= 0.0f)
            return false;
        lightSample.emission = light.intensity * light.emission;
        lightSample.pdf = sq(lightSample.dist) / sq(light.size);
        break;
    case LightType::Directional:
        lightSample.L = -normalize(light.direction);
        lightSample.dist = 10000.0f;
        lightSample.emission = light.intensity * light.emission;
        lightSample.pdf = 1.0f;
        break;
    }

    // Check correct side of the normal.
    if (dot(normal, lightSample.L) <= 0.0f)
        return false;

    // Check if the light is occluded.
    Ray shadowRay = {hitPosition, lightSample.L};
    bool shadowRayHit = recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, lightSample.dist, recursionDepth);
    return !shadowRayHit;
}

float3 MIS(inout uint rng_state, PBRPrimitiveConstantBuffer material, in float eta,
           in float3 hitPosition, in LightBuffer light, in float3 N, in uint recursionDepth)
{
    float3 reflectance = float3(0.0f, 0.0f, 0.0f);

    // Sample the light source.
    LightSample lightSample;
    if (NextEventEstimation(rng_state, light, hitPosition, N, recursionDepth, lightSample))
    {
        // Evaluate the BSDF.
        float bsdfPdf;
        reflectance = EvaluateDisneyBSDF(material, g_sceneCB.anisotropicBSDF, eta, -WorldRayDirection(), lightSample.L, N, bsdfPdf);

        // Calculate the MIS weight.
        float weight = 1.0f;
        if (light.type != LightType::Directional)
            weight = PowerHeuristic(1.0f, lightSample.pdf, 1.0f, bsdfPdf);

        // Calculate the final color.
        if (bsdfPdf > 0.0f)
            reflectance *= weight * lightSample.emission / lightSample.pdf;
        else
            reflectance = float3(0.0f, 0.0f, 0.0f);
    }

    // Sample the BSDF
    // float3 L;
    // float bsdfPdf;
    // float3 bsdf = SampleDisneyBSDF(rng_state, material, eta, -WorldRayDirection(), N, L, bsdfPdf);
    // if (bsdfPdf > 0.0f)
    // {
    //     // Evaluate the light source.
    //     float3 lightEmission = float3(0.0f, 0.0f, 0.0f);
    //     float lightPdf = 0.0f;
    //     if (NextEventEstimation(rng_state, L, light, hitPosition, recursionDepth, lightSample))
    //     {
    //         lightEmission = lightSample.emission;
    //         lightPdf = lightSample.pdf;

    //         // Calculate the MIS weight.
    //         float weight = 1.0f;
    //         if (light.type != LightType::Directional)
    //             weight = PowerHeuristic(1.0f, bsdfPdf, 1.0f, lightPdf);

    //         // Calculate the final color.
    //         reflectance += weight * bsdf * lightEmission / bsdfPdf;
    //     }
    // }

    return reflectance;
}

float3 DoPathTracing(in RayPayload rayPayload, in PBRPrimitiveConstantBuffer material, in float3 N, in float3 hitPosition, in float hitDistance)
{
    bool inside = dot(WorldRayDirection(), N) > 0.0f;
    float3 normalSide = inside ? -N : N;

    // Checkerboard pattern for the floor.
    if (material.materialIndex == 0)
        material.roughness = fmod(abs(floor(hitPosition.x / 2.0f) + floor(hitPosition.z / 2.0f)), 2.0f) * 0.25f + 0.25f;

    // Set IOR.
    float eta = inside ? material.eta : 1.0f / material.eta;

    // Reset absorption if going outside.
    float3 absorption = inside ? rayPayload.absorption.xyz : float3(0.0f, 0.0f, 0.0f);

    // Add absorption.
    float3 throughput = rayPayload.throughput.xyz * exp(-absorption * hitDistance);

    // Initialize the random number generator.
    uint rng_state = hash(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks + rayPayload.recursionDepth * 1337 + hash(hitPosition));

    //--- Multiple importance sampling.
    float3 color = float3(0.0f, 0.0f, 0.0f); // Accumulated color

    // 1. Sample direct lighting.

    if (g_sceneCB.onlyOneLightSample)
    {
        // If one light at a time, choose randomly and adjust pdf
        uint idx = random(rng_state) * g_sceneCB.numLights;
        color += throughput * MIS(rng_state, material, eta, hitPosition, g_lights[idx], normalSide, rayPayload.recursionDepth) * g_sceneCB.numLights;
    }
    else
    {
        // Sample all lights at once
        for (uint i = 0; i < g_sceneCB.numLights; i++)
            color += throughput * MIS(rng_state, material, eta, hitPosition, g_lights[i], normalSide, rayPayload.recursionDepth);
    }

    // Apply Russian roulette.
    float russianRoulettePdf = 1.0f;
    if (rayPayload.recursionDepth >= g_sceneCB.russianRouletteDepth)
    {
        russianRoulettePdf = max(throughput.x, max(throughput.y, throughput.z));
        if (random(rng_state) > russianRoulettePdf)
            return color;
    }

    // 2. Sample BSDF.

    float3 reflectance;
    float3 L;
    float samplePdf, bsdfPdf;

    // Compute local space.
    float3 T, B;
    ComputeLocalSpace(N, T, B);

    if (g_sceneCB.importanceSamplingType == 0)
    {
        L = UniformSampleSphere(random(rng_state), random(rng_state), samplePdf);
        L = normalize(GetTangentToWorld(N, T, B, L));
        reflectance = EvaluateDisneyBSDF(material, g_sceneCB.anisotropicBSDF, eta, -WorldRayDirection(), L, normalSide, bsdfPdf);
        bsdfPdf = samplePdf;
    }
    else if (g_sceneCB.importanceSamplingType == 1)
    {
        L = CosineSampleHemisphere(random(rng_state), random(rng_state), samplePdf);
        if (random(rng_state) < 0.5f)
            L = -L;
        L = normalize(GetTangentToWorld(N, T, B, L));
        reflectance = EvaluateDisneyBSDF(material, g_sceneCB.anisotropicBSDF, eta, -WorldRayDirection(), L, normalSide, bsdfPdf);
        bsdfPdf = samplePdf / 2.0f;
    }
    else
    {
        reflectance = SampleDisneyBSDF(rng_state, material, eta, g_sceneCB.anisotropicBSDF, -WorldRayDirection(), normalSide, L, bsdfPdf);
    }

    // Update absorption.
    if (dot(normalSide, L) < 0.0f)
        absorption = -log(material.extinction) / material.atDistance;

    // Update throughput and continue only if the pdf is non-zero.
    if (bsdfPdf > 0.0f)
    {
        throughput *= reflectance / bsdfPdf;

        // Apply the Russian roulette pdf.
        throughput /= russianRoulettePdf;

        // Shoot the next ray and accumulate the color.
        Ray newRay = {hitPosition, L};
        color += TraceRadianceRay(newRay, float4(throughput, 1.0f), float4(absorption, 1.0f), rayPayload.recursionDepth, dot(N, L) < 0.0f).xyz;
    }

    return color;
}

//***************************************************************************
//*****************------ Phong functions -------****************************
//***************************************************************************

// Diffuse lighting calculation.
float CalculateDiffuseCoefficient(in float3 hitPosition, in float3 incidentLightRay, in float3 normal)
{
    float fNDotL = saturate(dot(-incidentLightRay, normal));
    return fNDotL;
}

// Phong lighting specular component
float3 CalculateSpecularCoefficient(in float3 hitPosition, in float3 incidentLightRay, in float3 normal, in float specularPower)
{
    float3 reflectedLightRay = normalize(reflect(incidentLightRay, normal));
    return pow(saturate(dot(reflectedLightRay, normalize(-WorldRayDirection()))), specularPower);
}

// Phong lighting model = diffuse + specular components.
float3 CalculatePhongLighting(in LightBuffer light, in float4 albedo, in float3 normal,
                              in float diffuseCoef = 1.0, in float specularCoef = 1.0, in float specularPower = 50)
{
    float3 hitPosition = HitWorldPosition();
    float3 incidentLightRay;
    if (light.type == LightType::Directional)
        incidentLightRay = normalize(light.direction);
    else
        incidentLightRay = normalize(hitPosition - light.position);

    // Diffuse component.
    float Kd = CalculateDiffuseCoefficient(hitPosition, incidentLightRay, normal);
    float3 diffuseColor = diffuseCoef * Kd * light.emission * light.intensity * albedo.xyz;

    // Specular component.
    float3 lightSpecularColor = light.emission * light.intensity;
    float3 Ks = CalculateSpecularCoefficient(hitPosition, incidentLightRay, normal, specularPower);
    float3 specularColor = specularCoef * Ks * lightSpecularColor;

    float3 color = diffuseColor + specularColor;
    if (light.type != LightType::Directional)
        color *= sq(light.size) / length_toPow2(light.position - hitPosition);
    return color;
}

//***************************************************************************
//********************------ Ray gen shader.. -------************************
//***************************************************************************

[shader("raygeneration")] void RaygenShader()
{
    // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
    Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, float2(0.5f, 0.5f));

    // Cast a ray into the scene and retrieve a shaded color.
    UINT currentRecursionDepth = 0;
    float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), currentRecursionDepth);

    // Write the raytraced color to the output texture.
    const float gammaPow = 1.0f / 2.2f;
    color.xyz = pow(ACES(color.xyz), float3(gammaPow, gammaPow, gammaPow));
    g_renderTarget[DispatchRaysIndex().xy] = color;
}

    [shader("raygeneration")] void RaygenShader_PathTracingTemporal()
{
    // Initialize the random number generator.
    uint rng_state = hash(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks);

    uint numSamples = g_sceneCB.pathSqrtSamplesPerPixel * g_sceneCB.pathSqrtSamplesPerPixel;

    // Compute the ray offset inside the pixel (for stratified sampling), between 0 and 1.
    float2 offset = numSamples == 1 ? float2(0.5f, 0.5f) : float2(((g_sceneCB.pathFrameCacheIndex - 1) % g_sceneCB.pathSqrtSamplesPerPixel + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel, (floor((g_sceneCB.pathFrameCacheIndex - 1) / g_sceneCB.pathSqrtSamplesPerPixel) + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel);

    // Apply jittering if enabled.
    float2 jitter = select(g_sceneCB.applyJitter, float2(random(rng_state), random(rng_state)), float2(0.5f, 0.5f)); // in [0, 1)
    offset += (jitter - 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel;

    // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
    Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, offset);

    // Cast a ray into the scene and retrieve a shaded color.
    UINT currentRecursionDepth = 0;
    float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), currentRecursionDepth);

    // Accumulate the color.
    const float lerpFactor = 1.0f * (g_sceneCB.pathFrameCacheIndex - 1) / g_sceneCB.pathFrameCacheIndex;
    float3 finalColor = lerp(color.xyz, g_renderTarget[DispatchRaysIndex().xy].xyz, lerpFactor);
    const float gammaPow = 1.0f / 2.2f;
    // finalColor = pow(ACES(finalColor), float3(gammaPow, gammaPow, gammaPow));
    g_renderTarget[DispatchRaysIndex().xy] = float4(finalColor, 1.0f);
}

[shader("raygeneration")] void RaygenShader_PathTracing()
{
    // Initialize the random number generator.
    uint rng_state = hash(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks);

    float3 finalColor = float3(0.0f, 0.0f, 0.0f);

    uint numSamples = g_sceneCB.pathSqrtSamplesPerPixel * g_sceneCB.pathSqrtSamplesPerPixel;

    for (uint i = 0; i < g_sceneCB.pathSqrtSamplesPerPixel; i++)
        for (uint j = 0; j < g_sceneCB.pathSqrtSamplesPerPixel; j++)
        {
            // Compute the ray offset inside the pixel (for stratified sampling), between 0 and 1.
            float2 offset = numSamples == 1 ? float2(0.5f, 0.5f) : float2((i + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel, (j + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel);

            // Apply jittering if enabled.
            float2 jitter = select(g_sceneCB.applyJitter, float2(random(rng_state), random(rng_state)), float2(0.5f, 0.5f)); // in [0, 1)
            offset += (jitter - 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel;

            // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
            Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, offset);

            // Cast a ray into the scene and retrieve a shaded color.
            UINT currentRecursionDepth = 0;
            float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), currentRecursionDepth);

            // Accumulate the color.
            finalColor += color.xyz;
        }

    // Average the accumulated color.
    const float gammaPow = 1.0f / 2.2f;
    finalColor = pow(ACES(finalColor / numSamples), float3(gammaPow, gammaPow, gammaPow));
    g_renderTarget[DispatchRaysIndex().xy] = float4(finalColor, 1.0f);
}

//***************************************************************************
//******************------ Closest hit shaders -------***********************
//***************************************************************************

void ClosestHitHelper(inout RayPayload rayPayload, in float3 normal, in float3 hitPosition)
{
    float4 color = float4(0, 0, 0, 1);

    // PERFORMANCE TIP: it is recommended to avoid values carry over across TraceRay() calls.
    // Therefore, in cases like retrieving HitWorldPosition(), it is recomputed every time.
    if (g_sceneCB.raytracingType > 0) // path tracing
    {
        color.xyz = DoPathTracing(rayPayload, l_pbrCB, normal, hitPosition, RayTCurrent());
    }
    else // Whitted-style ray tracing
    {
        // Reflected component.
        if (l_materialCB.reflectanceCoef > 0.001)
        {
            // Trace a reflection ray.
            Ray reflectionRay = {hitPosition, reflect(WorldRayDirection(), normal)};
            float4 reflectionColor = TraceRadianceRay(reflectionRay, rayPayload.throughput, rayPayload.absorption, rayPayload.recursionDepth);

            float3 fresnelR = FresnelReflectanceSchlick(WorldRayDirection(), normal, l_materialCB.albedo.xyz);
            color.xyz += l_materialCB.reflectanceCoef * fresnelR * reflectionColor.xyz;
        }

        for (uint i = 0; i < g_sceneCB.numLights; i++)
        {
            // Shadow component.
            // Trace a shadow ray only if recursion depth allows it.
            float3 shadowRayDir;
            float lightDist;
            if (g_lights[i].type == LightType::Directional) {
                lightDist = 10000.0f;
                shadowRayDir = -normalize(g_lights[i].direction);
            }
            else {
                lightDist = length(g_lights[i].position - hitPosition);
                shadowRayDir = (g_lights[i].position - hitPosition) / lightDist;
            }
            Ray shadowRay = {hitPosition, shadowRayDir};
            bool shadowRayHit = rayPayload.recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, lightDist, rayPayload.recursionDepth);

            // Calculate final color.
            float3 phongColor = shadowRayHit ? float3(0, 0, 0) : CalculatePhongLighting(g_lights[i], l_materialCB.albedo, normal, l_materialCB.diffuseCoef, l_materialCB.specularCoef, l_materialCB.specularPower);
            color.xyz += phongColor;
        }
    }

    // Apply visibility falloff.
    float t = RayTCurrent();
    color = lerp(color, g_sceneCB.backgroundColor * rayPayload.throughput, 1.0 - exp(-0.000002 * t * t * t));

    rayPayload.color = color;
}

[shader("closesthit")] void ClosestHitShader_Triangle(inout RayPayload rayPayload, in BuiltInTriangleIntersectionAttributes attr)
{
    // Get the base index of the triangle's first 16 bit index.
    uint indexSizeInBytes = 2;
    uint indicesPerTriangle = 3;
    uint triangleIndexStride = indicesPerTriangle * indexSizeInBytes;
    uint baseIndex = PrimitiveIndex() * triangleIndexStride;

    // Load up three 16 bit indices for the triangle.
    const uint3 indices = Load3x16BitIndices(baseIndex, g_indices);

    // Retrieve corresponding vertex normals for the triangle vertices.
    float3 triangleNormal = g_vertices[indices[0]].normal;

    ClosestHitHelper(rayPayload, triangleNormal, HitWorldPosition());
}

    [shader("closesthit")] void ClosestHitShader_AABB(inout RayPayload rayPayload, in ProceduralPrimitiveAttributes attr)
{
    // PERFORMANCE TIP: it is recommended to minimize values carry over across TraceRay() calls.
    // Therefore, in cases like retrieving HitWorldPosition(), it is recomputed every time.

    ClosestHitHelper(rayPayload, attr.normal, HitWorldPosition());
}

//***************************************************************************
//**********************------ Miss shaders -------**************************
//***************************************************************************

[shader("miss")] void MissShader(inout RayPayload rayPayload)
{
    rayPayload.color = g_sceneCB.backgroundColor * rayPayload.throughput;
}

    [shader("miss")] void MissShader_ShadowRay(inout ShadowRayPayload rayPayload)
{
    rayPayload.hit = false;
}

//***************************************************************************
//*****************------ Intersection shaders-------************************
//***************************************************************************

// Get ray in AABB's local space.
Ray GetRayInAABBPrimitiveLocalSpace()
{
    PrimitiveInstancePerFrameBuffer attr = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];

    // Retrieve a ray origin position and direction in bottom level AS space
    // and transform them into the AABB primitive's local space.
    Ray ray;
    ray.origin = mul(float4(ObjectRayOrigin(), 1), attr.bottomLevelASToLocalSpace).xyz;
    ray.direction = mul(ObjectRayDirection(), (float3x3)attr.bottomLevelASToLocalSpace);
    return ray;
}

[shader("intersection")] void IntersectionShader_AnalyticPrimitive()
{
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
    AnalyticPrimitive::Enum primitiveType = (AnalyticPrimitive::Enum)l_aabbCB.primitiveType;

    float thit;
    ProceduralPrimitiveAttributes attr = (ProceduralPrimitiveAttributes)0;
    if (RayAnalyticGeometryIntersectionTest(localRay, primitiveType, thit, attr))
    {
        PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3)aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3)ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, /*hitKind*/ 0, attr);
    }
}

    [shader("intersection")] void IntersectionShader_SignedDistancePrimitive()
{
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
    SignedDistancePrimitive::Enum primitiveType = (SignedDistancePrimitive::Enum)l_aabbCB.primitiveType;

    float thit;
    ProceduralPrimitiveAttributes attr = (ProceduralPrimitiveAttributes)0;
    float stepScale = l_materialCB.stepScale;
    if (RaySignedDistancePrimitiveTest(localRay, primitiveType, thit, attr, stepScale))
    {
        PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3)aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3)ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, /*hitKind*/ 0, attr);
    }
}

#endif // RAYTRACING_HLSL