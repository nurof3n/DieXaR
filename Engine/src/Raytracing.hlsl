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
ConstantBuffer<PrimitiveInstanceConstantBuffer> l_primitiveCB : register(b2);
ConstantBuffer<PBRPrimitiveConstantBuffer> l_pbrCB : register(b3); // PBR material constant buffer

// ##################################################################### //
// ########################## Special Effects ########################## //
// ##################################################################### //

float Checkerboard(float3 hitPosition)
{
    return fmod(abs(floor((hitPosition.x + 1.0f) / 2.0f) + floor(hitPosition.z / 2.0f)), 2.0f);
}

// https://www.shadertoy.com/view/lt2SR1
float3 SkyColor(in float3 rd, in LightBuffer light)
{
    float3 sundir = -normalize(light.direction);

    float yd = min(rd.y, 0.);
    rd.y = max(rd.y, 0.);

    float3 col = float3(0., 0., 0.);

    col += sqrt(light.intensity) * float3(.4, .4 - exp(-rd.y * 20.) * .15, .0) * exp(-rd.y * 9.); // Red / Green
    col += sqrt(light.intensity) * float3(.3, .5, .6) * (1. - exp(-rd.y * 8.)) * exp(-rd.y * .9); // Blue

    col = sqrt(light.intensity) * lerp(col * 1.2, float3(.34, .44, .4), 1. - exp(yd * 100.)); // Fog
    col += sqrt(light.intensity) * pow(saturate(dot(rd, sundir)), 150.0);

    col += light.intensity * float3(1.0, .8, .55) * pow(max(dot(rd, sundir), 0.), 15.) * .6; // Sun
    col += pow(max(dot(rd, sundir), 0.), 150.0) * .15;

    return pow(col, 2.2f);
}

float4 ComputeBackground()
{
    switch (g_sceneCB.sceneIndex)
    {
    case SceneTypes::CornellBox:
    case SceneTypes::Demo:
        return g_sceneCB.backgroundColor;
    case SceneTypes::PbrShowcase:
        return float4(SkyColor(WorldRayDirection(), g_lights[0]), 1.0f);
    default:
        return float4(0, 0, 0, 1);
    }
}

//***************************************************************************
//*****------ TraceRay wrappers for radiance and shadow rays. -------********
//***************************************************************************

// Trace a radiance ray into the scene and returns a shaded color.
float4 TraceRadianceRay(in Ray ray, in float4 throughput, in float4 absorption, in UINT rngState, in UINT currentRayRecursionDepth,
                        in float bsdfPdf = 1.0f, in bool inside = false, in bool refraction = false)
{
    if (currentRayRecursionDepth >= g_sceneCB.maxRecursionDepth)
        return float4(0, 0, 0, 0);

    // Set the ray's extents.
    RayDesc rayDesc;
    rayDesc.Origin = ray.origin;
    rayDesc.Direction = ray.direction;
    // Set TMin to a zero value to avoid aliasing artifacts along contact areas.
    // Note: make sure to enable face culling so as to avoid surface face fighting.
    rayDesc.TMin = 0.0001f;
    rayDesc.TMax = 10000.0f;
    if (refraction)
        inside = !inside;

    RayPayload rayPayload = {float4(0.0f, 0.0f, 0.0f, 0.0f), throughput, absorption, rngState, currentRayRecursionDepth + 1, inside, bsdfPdf};

    uint flag = RAY_FLAG_NONE; // because the AABB for example does not work culling faces
    TraceRay(g_scene,
             flag,
             TraceRayParameters::InstanceMask,
             TraceRayParameters::HitGroup::Offset[RayType::Radiance],
             TraceRayParameters::HitGroup::GeometryStride,
             TraceRayParameters::MissShader::Offset[RayType::Radiance],
             rayDesc, rayPayload);

    return radianceClamp(rayPayload.color);
}

// Trace a shadow ray and return true if it hits any geometry.
bool TraceShadowRayAndReportIfHit(in Ray ray, in float lightDist, in UINT currentRayRecursionDepth)
{
    if (currentRayRecursionDepth >= g_sceneCB.maxRecursionDepth + 1 || currentRayRecursionDepth >= g_sceneCB.maxShadowRecursionDepth)
        return false;

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

// Samples a square light.
void SampleSquareLight(in LightBuffer light, in float3 hitPosition, in float3 L, in float dist, out LightSample lightSample)
{
    lightSample.L = L;
    lightSample.dist = dist;
    lightSample.emission = light.intensity * light.emission * sq(light.size);
    lightSample.pdf = sq(dist) / dot(L, float3(0.0f, 1.0f, 0.0f));
}

// Samples a light source.
bool NextEventEstimation(inout uint rng_state, in LightBuffer light, in float3 hitPosition, in float3 normal,
                         in uint recursionDepth, out LightSample lightSample)
{
    float angleL, angleN;
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
        angleL = dot(lightSample.L, float3(0.0f, 1.0f, 0.0f));
        if (angleL <= 0.0f)
            return false;
        // check correct side of the normal (but this is factored in the eval bsdf)
        angleN = dot(normal, lightSample.L);
        if (angleN <= 0.0f)
            return false;
        // compute contribution and pdf
        lightSample.emission = light.intensity * light.emission * sq(light.size);
        lightSample.pdf = sq(lightSample.dist) / angleL;
        break;
    case LightType::Directional:
        lightSample.L = -normalize(light.direction);
        // // offset the direction inside a solid angle
        // lightSample.L = OffsetDirectionInSolidAngle(lightSample.L, random(rng_state) * TWO_PI, random(rng_state) * 0.01f);
        lightSample.dist = 10000.0f;
        // check correct side of the normal (but this is factored in the eval bsdf)
        angleN = dot(normal, lightSample.L);
        if (angleN <= 0.0f)
            return false;
        lightSample.emission = light.intensity * light.emission;
        lightSample.pdf = 1.0f;
        break;
    }

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

        // Calculate the MIS weight (but not for directional lights, because they are sampled with a pdf of 1.0f)
        float weight = 1.0f;
        if (light.type == LightType::Directional)
            weight = 1.0f;
        else
            weight = PowerHeuristic(1.0f, lightSample.pdf, 1.0f, bsdfPdf);

        // Calculate the final color.
        if (bsdfPdf > 0.0f)
            reflectance *= weight * lightSample.emission / lightSample.pdf;
        else
            reflectance = float3(0.0f, 0.0f, 0.0f);
    }

    return reflectance;
}

float3 DoPathTracing(inout RayPayload rayPayload, in PBRPrimitiveConstantBuffer material, in float3 N, in float3 hitPosition, in float hitDistance)
{
    // Compute the corrected normal (that retains the opposite side with V when exiting the object).
    float3 normalSide = dot(WorldRayDirection(), N) < 0.0f ? N : -N;

    // Checkerboard pattern for the floor.
    if (material.materialIndex == 0 && g_sceneCB.sceneIndex != SceneTypes::CornellBox)
    {
        float pattern = Checkerboard(hitPosition);
        material.roughness = pattern * 0.25f;
        material.albedo.xyz = pattern * 0.5f * material.albedo.xyz + 0.5f * material.albedo.xyz;
    }

    // Clamp roughness.
    material.roughness = max(0.001f, material.roughness);

    // Set IOR.
    float eta = rayPayload.inside ? material.eta : 1.0f / material.eta;

    // Reset absorption if going outside.
    float3 absorption = rayPayload.inside ? rayPayload.absorption.xyz : float3(0.0f, 0.0f, 0.0f);

    //--- Multiple importance sampling.
    float3 color = float3(0.0f, 0.0f, 0.0f); // Accumulated color

    // Add absorption.
    float3 throughput = rayPayload.throughput.xyz * exp(-absorption * hitDistance);

    // Direct lighting sampling.

    if (g_sceneCB.sceneIndex != SceneTypes::PbrShowcase)
    {
        if (g_sceneCB.onlyOneLightSample)
        {
            // If one light at a time, choose randomly and adjust pdf
            uint idx = random(rayPayload.rngState) * g_sceneCB.numLights;
            color += throughput * MIS(rayPayload.rngState, material, eta, hitPosition, g_lights[idx], normalSide, rayPayload.recursionDepth) * g_sceneCB.numLights;
        }
        else
        {
            // Sample all lights at once
            for (uint i = 0; i < g_sceneCB.numLights; i++)
                color += throughput * MIS(rayPayload.rngState, material, eta, hitPosition, g_lights[i], normalSide, rayPayload.recursionDepth);
        }
    }

    // Apply Russian roulette path termination.
    float russianRoulettePdf = 1.0f;
    if (rayPayload.recursionDepth >= g_sceneCB.russianRouletteDepth)
    {
        russianRoulettePdf = min(max(throughput.x, max(throughput.y, throughput.z)) + 0.001f, 0.95f);
        if (random(rayPayload.rngState) > russianRoulettePdf)
            return color;
    }

    // Indirect lighting sampling.

    float3 reflectance;
    float3 L;
    float samplePdf, bsdfPdf;

    // Compute local space.
    float3 T, B;
    ComputeLocalSpace(N, T, B);

    if (g_sceneCB.importanceSamplingType == 0)
    {
        L = UniformSampleSphere(random(rayPayload.rngState), random(rayPayload.rngState), samplePdf);
        L = normalize(GetTangentToWorld(N, T, B, L));
        reflectance = EvaluateDisneyBSDF(material, g_sceneCB.anisotropicBSDF, eta, -WorldRayDirection(), L, normalSide, bsdfPdf);
        bsdfPdf = samplePdf;
    }
    else if (g_sceneCB.importanceSamplingType == 1)
    {
        L = CosineSampleHemisphere(random(rayPayload.rngState), random(rayPayload.rngState), samplePdf);
        L = normalize(GetTangentToWorld(N, T, B, L));
        reflectance = EvaluateDisneyBSDF(material, g_sceneCB.anisotropicBSDF, eta, -WorldRayDirection(), L, normalSide, bsdfPdf);
        bsdfPdf = samplePdf;
    }
    else
        reflectance = SampleDisneyBSDF(rayPayload.rngState, material, eta, g_sceneCB.anisotropicBSDF, -WorldRayDirection(), normalSide, L, bsdfPdf);

    // Update absorption if we're changing sides.
    bool refraction = dot(normalSide, L) < 0.0f;
    if (refraction)
        absorption = -log(material.extinction) / material.atDistance;

    // Update throughput and continue only if the pdf is non-zero.
    if (bsdfPdf > 0.0f)
    {
        throughput *= reflectance / bsdfPdf;

        // Apply the Russian roulette pdf.
        throughput /= russianRoulettePdf;

        // Shoot the next ray and accumulate the color.
        Ray newRay = {hitPosition, L};
        color += TraceRadianceRay(newRay, float4(throughput, 1.0f), float4(absorption, 1.0f), rayPayload.rngState, rayPayload.recursionDepth, bsdfPdf, rayPayload.inside, refraction).xyz;
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

    // Initialize the random number generator.
    UINT rngState = hash(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks);

    // Cast a ray into the scene and retrieve a shaded color.
    UINT currentRecursionDepth = 0;
    float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), rngState, currentRecursionDepth);

    // Write the raytraced color to the output texture.
    const float gammaPow = 1.0f / 2.2f;
    color.xyz = pow(ACES(color.xyz), float3(gammaPow, gammaPow, gammaPow));
    g_renderTarget[DispatchRaysIndex().xy] = color;
}

    [shader("raygeneration")] void RaygenShader_PathTracingTemporal()
{
    // Initialize the random number generator.
    uint rngState = hash(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks);

    uint numSamples = g_sceneCB.pathSqrtSamplesPerPixel * g_sceneCB.pathSqrtSamplesPerPixel;

    // Compute the ray offset inside the pixel (for stratified sampling), between 0 and 1.
    float2 offset = numSamples == 1 ? float2(0.5f, 0.5f) : float2(((g_sceneCB.pathFrameCacheIndex - 1) % g_sceneCB.pathSqrtSamplesPerPixel + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel, (floor((g_sceneCB.pathFrameCacheIndex - 1) / g_sceneCB.pathSqrtSamplesPerPixel) + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel);

    // Apply jittering if enabled.
    float2 jitter = select(g_sceneCB.applyJitter, float2(random(rngState), random(rngState)), float2(0.5f, 0.5f)); // in [0, 1)
    offset += (jitter - 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel;

    // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
    Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, offset);

    // Cast a ray into the scene and retrieve a shaded color.
    UINT currentRecursionDepth = 0;
    float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), rngState, currentRecursionDepth);

    // Accumulate the color.
    const float lerpFactor = 1.0f * (g_sceneCB.pathFrameCacheIndex - 1) / g_sceneCB.pathFrameCacheIndex;
    float3 finalColor = lerp(color.xyz, g_renderTarget[DispatchRaysIndex().xy].xyz, lerpFactor);
    const float gammaPow = 1.0f / 2.2f;
    // finalColor = pow(ACES(finalColor), float3(gammaPow, gammaPow, gammaPow));
    g_renderTarget[DispatchRaysIndex().xy] = float4(finalColor, 1.0f);
}

[shader("raygeneration")] void RaygenShader_PathTracing()
{
    float3 finalColor = float3(0.0f, 0.0f, 0.0f);

    uint numSamples = g_sceneCB.pathSqrtSamplesPerPixel * g_sceneCB.pathSqrtSamplesPerPixel;

    for (uint i = 0; i < g_sceneCB.pathSqrtSamplesPerPixel; i++)
        for (uint j = 0; j < g_sceneCB.pathSqrtSamplesPerPixel; j++)
        {
            // Initialize the random number generator.
            uint rngState = hash(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks + 67 * i * g_sceneCB.pathSqrtSamplesPerPixel + j);

            // Compute the ray offset inside the pixel (for stratified sampling), between 0 and 1.
            float2 offset = numSamples == 1 ? float2(0.5f, 0.5f) : float2((i + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel, (j + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel);

            // Apply jittering if enabled.
            float2 jitter = select(g_sceneCB.applyJitter, float2(random(rngState), random(rngState)), float2(0.5f, 0.5f)); // in [0, 1)
            offset += (jitter - 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel;

            // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
            Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, offset);

            // Cast a ray into the scene and retrieve a shaded color.
            UINT currentRecursionDepth = 0;
            float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), rngState, currentRecursionDepth);

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

void ClosestHitHelper(inout RayPayload rayPayload, in float3 normal, in float3 hitPosition, in bool triangleGeometry)
{
    float4 color = float4(0, 0, 0, 1);

    // PERFORMANCE TIP: it is recommended to avoid values carry over across TraceRay() calls.
    // Therefore, in cases like retrieving HitWorldPosition(), it is recomputed every time.
    if (g_sceneCB.raytracingType > 0) // path tracing
    {
        // Check if we hit a light source.
        if (triangleGeometry && l_primitiveCB.primitiveType == 1)
        {
            // Check right side of the light.
            if (dot(float3(0.0f, 1.0f, 0.0f), WorldRayDirection()) > 0.0f)
            {
                color.xyz = g_lights[l_primitiveCB.instanceIndex].emission * g_lights[l_primitiveCB.instanceIndex].intensity;
                if (rayPayload.recursionDepth > 1)
                {
                    LightSample lightSample;
                    SampleSquareLight(g_lights[l_primitiveCB.instanceIndex], hitPosition, WorldRayDirection(), RayTCurrent(), lightSample);
                    color.xyz *= rayPayload.throughput.xyz * PowerHeuristic(rayPayload.bsdfPdf, 1.0f, lightSample.pdf, 1.0f);
                }
            }
            else
                color.xyz = float3(0, 0, 0);
        }
        else
            color.xyz = DoPathTracing(rayPayload, l_pbrCB, normal, hitPosition, RayTCurrent());
    }
    else // Whitted-style ray tracing
    {
        // Check if we hit a light source.
        if (triangleGeometry && l_primitiveCB.primitiveType == 1)
            color.xyz = g_lights[l_primitiveCB.instanceIndex].emission * g_lights[l_primitiveCB.instanceIndex].intensity;
        else
        {
            // Checkerboard pattern for the floor.
            float reflectanceCoef = l_materialCB.reflectanceCoef;
            float4 albedo = l_materialCB.albedo;
            if (l_materialCB.materialIndex == 0 && g_sceneCB.sceneIndex != SceneTypes::CornellBox)
            {
                float pattern = Checkerboard(hitPosition);
                reflectanceCoef = pattern * 0.25f + 0.25f;
                albedo.xyz = pattern * 0.5f * albedo.xyz + 0.5f * albedo.xyz;
            }

            // Reflected component.
            if (reflectanceCoef > 0.001)
            {
                // Trace a reflection ray.
                Ray reflectionRay = {hitPosition, reflect(WorldRayDirection(), normal)};
                float4 reflectionColor = TraceRadianceRay(reflectionRay, rayPayload.throughput, rayPayload.absorption, rayPayload.rngState, rayPayload.recursionDepth);

                float fresnelR = FresnelDielectric(max(0.0f, dot(-WorldRayDirection(), normal)), 1.0f / 1.5f);
                color.xyz += reflectanceCoef * fresnelR * reflectionColor.xyz;
            }

            for (uint i = 0; i < g_sceneCB.numLights; i++)
            {
                // Shadow component.
                // Trace a shadow ray only if recursion depth allows it.
                float3 shadowRayDir;
                float lightDist;
                if (g_lights[i].type == LightType::Directional)
                {
                    lightDist = 10000.0f;
                    shadowRayDir = -normalize(g_lights[i].direction);
                }
                else
                {
                    lightDist = length(g_lights[i].position - hitPosition);
                    shadowRayDir = (g_lights[i].position - hitPosition) / lightDist;
                }
                Ray shadowRay = {hitPosition, shadowRayDir};
                bool shadowRayHit = rayPayload.recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, lightDist, rayPayload.recursionDepth);

                // Calculate final color.
                float3 phongColor = shadowRayHit ? float3(0, 0, 0) : CalculatePhongLighting(g_lights[i], albedo, normal, l_materialCB.diffuseCoef, l_materialCB.specularCoef, l_materialCB.specularPower);
                color.xyz += phongColor;
            }
        }
    }

    // Apply visibility falloff.
    float t = RayTCurrent();
    color = lerp(color, ComputeBackground() * rayPayload.throughput, 1.0 - exp(-0.000002 * t * t * t));
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

    // Transform the normal to world space.
    triangleNormal = normalize(mul((float3x3)ObjectToWorld3x4(), triangleNormal));

    ClosestHitHelper(rayPayload, triangleNormal, HitWorldPosition(), true);
}

    [shader("closesthit")] void ClosestHitShader_AABB(inout RayPayload rayPayload, in ProceduralPrimitiveAttributes attr)
{
    // PERFORMANCE TIP: it is recommended to minimize values carry over across TraceRay() calls.
    // Therefore, in cases like retrieving HitWorldPosition(), it is recomputed every time.

    ClosestHitHelper(rayPayload, attr.normal, HitWorldPosition(), false);
}

//***************************************************************************
//**********************------ Miss shaders -------**************************
//***************************************************************************

[shader("miss")] void MissShader(inout RayPayload rayPayload)
{
    rayPayload.color = ComputeBackground() * rayPayload.throughput;
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
    PrimitiveInstancePerFrameBuffer attr = g_AABBPrimitiveAttributes[l_primitiveCB.instanceIndex];

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
    AnalyticPrimitive::Enum primitiveType = (AnalyticPrimitive::Enum)l_primitiveCB.primitiveType;

    float thit;
    ProceduralPrimitiveAttributes attr = (ProceduralPrimitiveAttributes)0;
    if (RayAnalyticGeometryIntersectionTest(g_sceneCB.sceneIndex, localRay, primitiveType, thit, attr))
    {
        PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_primitiveCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3)aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3)ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, /*hitKind*/ 0, attr);
    }
}

    [shader("intersection")] void IntersectionShader_SignedDistancePrimitive()
{
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
    SignedDistancePrimitive::Enum primitiveType = (SignedDistancePrimitive::Enum)l_primitiveCB.primitiveType;

    float thit;
    ProceduralPrimitiveAttributes attr = (ProceduralPrimitiveAttributes)0;
    float stepScale = l_materialCB.stepScale;
    if (RaySignedDistancePrimitiveTest(localRay, primitiveType, thit, attr, stepScale))
    {
        PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_primitiveCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3)aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3)ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, /*hitKind*/ 0, attr);
    }
}

#endif // RAYTRACING_HLSL