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

//***************************************************************************
//*****------ Shader resources bound via root signatures -------*************
//***************************************************************************

// Scene wide resources.
//  g_* - bound via a global root signature.
//  l_* - bound via a local root signature.
RaytracingAccelerationStructure g_scene : register(t0, space0);
RWTexture2D<float4> g_renderTarget : register(u0);
ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);

// Triangle resources
ByteAddressBuffer g_indices : register(t1, space0);
StructuredBuffer<Vertex> g_vertices : register(t2, space0);

// Procedural geometry resources
StructuredBuffer<PrimitiveInstancePerFrameBuffer> g_AABBPrimitiveAttributes : register(t3, space0);
ConstantBuffer<PrimitiveConstantBuffer> l_materialCB : register(b1); // Sample material constant buffer
ConstantBuffer<PrimitiveInstanceConstantBuffer> l_aabbCB : register(b2);
ConstantBuffer<PBRPrimitiveConstantBuffer> l_pbrCB : register(b3); // PBR material constant buffer

//***************************************************************************
//******************------ PBR functions -------*****************************
//***************************************************************************

// Computes the intensity of the sheen lobe.
float3 ComputeSheen(in float dotLH, in float dotNL)
{
    // compute the luminance of albedo
    float luminance = dot(l_pbrCB.albedo.xyz, float3(0.3f, 0.6f, 0.1f));

    // sheen lobe is tinted by the albedo hue and saturation
    float3 tint = lerp(float3(1.0f, 1.0f, 1.0f), l_pbrCB.albedo.xyz / luminance, l_pbrCB.sheenTint);

    // sheen lobe is using the Schlick-Fresnel shape
    return l_pbrCB.sheen * tint * pow(1.0f - dotLH, 5) * dotNL;
}

// Computes the intensity of the clearcoat lobe.
float ComputeClearCoat(in float dotHL, in float dotNH,
                       in float dotLX, in float dotLY, in float dotLN, in float dotVX, in float dotVY, in float dotVN)
{
    // compute anisotropic roughness
    float a = lerp(0.1f, 0.001f, l_pbrCB.clearcoatGloss);

    // compute D term for the clearcoat
    float D = DGTR1(dotNH, a);

    // compute F term with Schlick approximation with IOR = 1.5 (F0 = 0.04)
    float F = FresnelReflectanceSchlick(dotHL, 0.04f);

    // compute geometric shadowing term G as product of two G1 terms
    float G = SmithG1Anisotropic(dotLX, dotLY, dotLN, a, a) * SmithG1Anisotropic(dotVX, dotVY, dotVN, a, a);

    return D * F * G / (4.0f * abs(dotVN));
}

// Disney Diffuse model.
float3 ComputeDiffuse(in float dotHL, in float dotNV, in float dotNL)
{
    float fd90 = FD90(l_pbrCB.roughness, dotHL);
    return l_pbrCB.albedo.xyz * INV_PI * FD(fd90, dotNV) * FD(fd90, dotNL) * dotNL;
}

// Disney Metallic BRDF - standard Cook-Torrance microfacet BRDF.
// This is modified to include a dielectric specular term that is missing in the diffuse model.
float3 ComputeMetallicBRDF(in float dotHX, in float dotHY, in float dotHN, in float dotLH,
                           in float dotLX, in float dotLY, in float dotLN, in float dotVX, in float dotVY, in float dotVN)
{
    // compute anisotropic parameters
    float ax, ay;
    ComputeAnisotropicAlphas(l_pbrCB.roughness, l_pbrCB.anisotropic, ax, ay);

    // compute distribution term D
    float D = DGTR2Anisotropic(dotHX, dotHY, dotHN, ax, ay);

    // compute Fresnel achromatic component
    // C0 can be tinted to the base color
    float luminance = dot(l_pbrCB.albedo.xyz, float3(0.3f, 0.6f, 0.1f));
    float3 Ks = lerp(float3(1.0f, 1.0f, 1.0f), l_pbrCB.albedo.xyz / luminance, l_pbrCB.specularTint); // same as sheen but different tint parameter
    float3 C0 = lerp(Ks * l_pbrCB.specular * R0FromIOR(l_pbrCB.eta), l_pbrCB.albedo.xyz, l_pbrCB.metallic);

    // compute Fresnel term
    float3 F = FresnelReflectanceSchlick(dotLH, C0);

    // compute geometric shadowing term G as product of two G1 terms
    float G = SmithG1Anisotropic(dotLX, dotLY, dotLN, ax, ay) * SmithG1Anisotropic(dotVX, dotVY, dotVN, ax, ay);

    return D * F * G / (4.0f * abs(dotVN));
}

float3 ComputeGlassBSDF(in bool inside, in float dotHX, in float dotHY, in float dotHN, in float dotLH, in float dotHV,
                        in float dotLX, in float dotLY, in float dotLN, in float dotVX, in float dotVY, in float dotVN, in float ax, in float ay)
{
    // compute distribution term D
    float D = DGTR2Anisotropic(dotHX, dotHY, dotHN, ax, ay);

    // compute Fresnel term
    float F = FresnelDielectric(dotLH, dotHV, l_pbrCB.eta);

    // compute geometric shadowing term G as product of two G1 terms
    float G = SmithG1Anisotropic(dotLX, dotLY, dotLN, ax, ay) * SmithG1Anisotropic(dotVX, dotVY, dotVN, ax, ay);

    float common = D * G / abs(dotVN);
    if (!inside)
        return common * F / 4.0f * l_pbrCB.albedo.xyz;
    else // take square root to account for the fact that the light is coming from the other side
        return common * (1.0f - F) * abs(dotLH * dotHV) / (abs(dotVN) * sq(dotHV + l_pbrCB.eta * dotLH)) * sqrt(l_pbrCB.albedo.xyz);
}

// Computes the Disney BSDF as an aggregate of all the defined lobes.
float3 ComputeDisneyBSDF(in float3 L, in float3 V, in float3 N, in float3 X, in float3 Y)
{
    float3 reflectance = float3(0.0f, 0.0f, 0.0f);

    // compute pdfs
    float pDiffuse, pMetal, pClearcoat, pGlass, wDiffuse, wMetal, wClearcoat, wGlass;
    ComputePdfs(l_pbrCB.metallic, l_pbrCB.specularTransmission, l_pbrCB.clearcoat,
                pMetal, pDiffuse, pClearcoat, pGlass, wMetal, wDiffuse, wClearcoat, wGlass);

    // compute dot products
    float3 H = normalize(L + V);
    float dotNL = dot(N, L);
    float dotNV = dot(N, V);
    float dotNH = dot(N, H);
    float dotLH = dot(L, H);
    float dotVH = dot(V, H);
    float dotHX = dot(H, X);
    float dotHY = dot(H, Y);
    float dotLX = dot(L, X);
    float dotLY = dot(L, Y);
    float dotVX = dot(V, X);
    float dotVY = dot(V, Y);

    bool specularVisible = dotNL > 0.0f && dotNV > 0.0f; // true if both L and V are on the outer side of the surface

    // apply diffuse and sheen
    if (wDiffuse > 0.0f)
    {
        float3 diffuse = ComputeDiffuse(dotLH, dotNV, dotNL);
        float3 sheen = float3(0.0f, 0.0f, 0.0f);

        if (l_pbrCB.sheen > 0.0f)
        {
            sheen = ComputeSheen(dotLH, dotNL);
        }

        reflectance += wDiffuse * (diffuse * l_pbrCB.albedo.xyz + sheen);
    }

    // apply metallic
    if (wMetal > 0.0f && specularVisible)
    {
        float3 metallic = ComputeMetallicBRDF(dotHX, dotHY, dotNH, dotLH, dotLX, dotLY, dotNL, dotVX, dotVY, dotNV);
        reflectance += wMetal * metallic;
    }

    // apply clearcoat
    if (wClearcoat > 0.0f && l_pbrCB.clearcoat > 0.0f && specularVisible)
    {
        float clearcoat = ComputeClearCoat(dotLH, dotNH, dotLX, dotLY, dotNL, dotVX, dotVY, dotNV);
        reflectance += wClearcoat * float3(clearcoat, clearcoat, clearcoat);
    }

    // apply transmission
    if (wGlass > 0.0f)
    {
        // TODO: scale the roughness of the glass by the IOR
        float ax, ay;
        ComputeAnisotropicAlphas(l_pbrCB.roughness, l_pbrCB.anisotropic, ax, ay);

        float3 transmission = ComputeGlassBSDF(dotNV * dotNV <= 0.0f, dotHX, dotHY, dotNH, dotLH, dotVH, dotLX, dotLY, dotNL, dotVX, dotVY, dotNV, ax, ay);
        reflectance += wGlass * transmission;
    }

    return reflectance;
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
float4 CalculateSpecularCoefficient(in float3 hitPosition, in float3 incidentLightRay, in float3 normal, in float specularPower)
{
    float3 reflectedLightRay = normalize(reflect(incidentLightRay, normal));
    return pow(saturate(dot(reflectedLightRay, normalize(-WorldRayDirection()))), specularPower);
}

// Phong lighting model = ambient + diffuse + specular components.
float4 CalculatePhongLighting(in float4 albedo, in float3 normal, in bool isInShadow, in float diffuseCoef = 1.0, in float specularCoef = 1.0, in float specularPower = 50)
{
    float3 hitPosition = HitWorldPosition();
    float3 lightPosition = g_sceneCB.lightPosition.xyz;
    float shadowFactor = isInShadow ? InShadowRadiance : 1.0;
    float3 incidentLightRay = normalize(hitPosition - lightPosition);

    // Diffuse component.
    float4 lightDiffuseColor = g_sceneCB.lightDiffuseColor;
    float Kd = CalculateDiffuseCoefficient(hitPosition, incidentLightRay, normal);
    float4 diffuseColor = shadowFactor * diffuseCoef * Kd * lightDiffuseColor * albedo;

    // Specular component.
    float4 specularColor = float4(0, 0, 0, 0);
    if (!isInShadow)
    {
        float4 lightSpecularColor = float4(1, 1, 1, 1);
        float4 Ks = CalculateSpecularCoefficient(hitPosition, incidentLightRay, normal, specularPower);
        specularColor = specularCoef * Ks * lightSpecularColor;
    }

    // Ambient component.
    // Fake AO: Darken faces with normal facing downwards/away from the sky a little bit.
    float4 ambientColor = g_sceneCB.lightAmbientColor;
    float4 ambientColorMin = g_sceneCB.lightAmbientColor - 0.1;
    float4 ambientColorMax = g_sceneCB.lightAmbientColor;
    float a = 1 - saturate(dot(normal, float3(0, -1, 0)));
    ambientColor = albedo * lerp(ambientColorMin, ambientColorMax, a);

    return ambientColor + diffuseColor + specularColor;
}

//***************************************************************************
//*****------ TraceRay wrappers for radiance and shadow rays. -------********
//***************************************************************************

// Trace a radiance ray into the scene and returns a shaded color.
float4 TraceRadianceRay(in Ray ray, in UINT currentRayRecursionDepth)
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
    rayDesc.TMin = 0;
    rayDesc.TMax = 10000;
    RayPayload rayPayload = {float4(0, 0, 0, 0), currentRayRecursionDepth + 1};
    TraceRay(g_scene,
             RAY_FLAG_CULL_BACK_FACING_TRIANGLES,
             TraceRayParameters::InstanceMask,
             TraceRayParameters::HitGroup::Offset[RayType::Radiance],
             TraceRayParameters::HitGroup::GeometryStride,
             TraceRayParameters::MissShader::Offset[RayType::Radiance],
             rayDesc, rayPayload);

    return rayPayload.color;
}

// Trace a shadow ray and return true if it hits any geometry.
bool TraceShadowRayAndReportIfHit(in Ray ray, in UINT currentRayRecursionDepth)
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
    rayDesc.TMin = 0;
    rayDesc.TMax = 10000;

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

//***************************************************************************
//********************------ Ray gen shader.. -------************************
//***************************************************************************

[shader("raygeneration")] void RaygenShader()
{
    // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
    Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, float2(0.5f, 0.5f));

    // Cast a ray into the scene and retrieve a shaded color.
    UINT currentRecursionDepth = 0;
    float4 color = TraceRadianceRay(ray, currentRecursionDepth);

    // Write the raytraced color to the output texture.
    g_renderTarget[DispatchRaysIndex().xy] = color;
}

[shader("raygeneration")] void RaygenShader_PathTracingTemporal()
{
    uint numSamples = g_sceneCB.pathSqrtSamplesPerPixel * g_sceneCB.pathSqrtSamplesPerPixel;

    // Compute the ray offset inside the pixel (for stratified sampling), between 0 and 1.
    float2 offset = numSamples == 1 ? float2(0.5f, 0.5f) : float2(((g_sceneCB.pathFrameCacheIndex - 1) % g_sceneCB.pathSqrtSamplesPerPixel + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel, (floor((g_sceneCB.pathFrameCacheIndex - 1) / g_sceneCB.pathSqrtSamplesPerPixel) + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel);

    // Apply jittering if enabled.
    float2 jitter = select(float2(0.5f, 0.5f), float2(random(DispatchRaysIndex().xy, g_sceneCB.pathFrameCacheIndex), random(DispatchRaysIndex().xy, g_sceneCB.pathFrameCacheIndex + numSamples)), g_sceneCB.applyJitter); // in [0, 1)
    offset += (jitter - 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel;

    // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
    Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, offset);

    // Cast a ray into the scene and retrieve a shaded color.
    UINT currentRecursionDepth = 0;
    float4 color = TraceRadianceRay(ray, currentRecursionDepth);

    // Accumulate the color.
    const float lerpFactor = g_sceneCB.pathFrameCacheIndex / (g_sceneCB.pathFrameCacheIndex + 1.0f);
    g_renderTarget[DispatchRaysIndex().xy] = float4(lerp(color.xyz, g_renderTarget[DispatchRaysIndex().xy].xyz, lerpFactor), 1.0f);
}

[shader("raygeneration")] void RaygenShader_PathTracing()
{
    float3 finalColor = float3(0, 0, 0);

    uint numSamples = g_sceneCB.pathSqrtSamplesPerPixel * g_sceneCB.pathSqrtSamplesPerPixel;

    for (uint i = 0; i < g_sceneCB.pathSqrtSamplesPerPixel; i++)
        for (uint j = 0; j < g_sceneCB.pathSqrtSamplesPerPixel; j++)
        {
            // Compute the ray offset inside the pixel (for stratified sampling), between 0 and 1.
            float2 offset = numSamples == 1 ? float2(0.5f, 0.5f) : float2((i + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel, (j + 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel);

            // Apply jittering if enabled.
            float2 jitter = select(float2(0.5f, 0.5f), float2(random(DispatchRaysIndex().xy, i * g_sceneCB.pathSqrtSamplesPerPixel + j), random(DispatchRaysIndex().xy, i * g_sceneCB.pathSqrtSamplesPerPixel + j + numSamples)), g_sceneCB.applyJitter); // in [0, 1)
            offset += (jitter - 0.5f) / g_sceneCB.pathSqrtSamplesPerPixel;

            // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
            Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld, offset);

            // Cast a ray into the scene and retrieve a shaded color.
            UINT currentRecursionDepth = 0;
            float4 color = TraceRadianceRay(ray, currentRecursionDepth);

            // Accumulate the color.
            finalColor += color.xyz;
        }

    // Average the accumulated color.
        g_renderTarget[DispatchRaysIndex().xy] = float4(finalColor / numSamples, 1.0f);
    }

//***************************************************************************
//******************------ Closest hit shaders -------***********************
//***************************************************************************

// Performs next event estimation importance sampling for a point light source.
// lightDir must be normalized.
bool NextEventEstimation(in uint recursionDepth, in float3 hitPosition, in float lightDist,
                         in float3 lightDir, in float3 lightDiffuse, in float lightIntensity, in float lightSize, out float3 lightContrib)
{
    // The light is pointing downwards on the Y axis.
    float angle = dot(lightDir, float3(0.0f, 1.0f, 0.0f));
    if (angle <= 0.0f)
    {
        lightContrib = float3(0, 0, 0);
        return false;
    }

    // Create a shadow ray towards the light.
    Ray shadowRay = {HitWorldPosition(), lightDir};
    bool shadowRayHit = recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, recursionDepth);
    if (!shadowRayHit)
    {
        // Calculate the light contribution.
        lightContrib = lightIntensity * lightDiffuse * angle / (1.0f + 0.001f * lightDist * lightDist);
        return true;
    }
    lightContrib = float3(0, 0, 0);
    return false;
}

float3 ShadePathTracing(in RayPayload rayPayload, in float3 normal, in float3 hitPosition)
{
    float3 lightPosition = g_sceneCB.lightPosition.xyz;
    float3 lightColor = g_sceneCB.lightDiffuseColor.xyz;
    float lightSize = g_sceneCB.lightSize;
    float lightIntensity = g_sceneCB.lightIntensity;
    float pdf = 1.0f; // Probability density function used to compensate for only sampling one light.

    // Choose light
    if (g_sceneCB.secondaryLight)
    {
        pdf = 0.5f;

        // Sample random number
        float r = random(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks);
        if (r > 0.5f)
        {
            lightPosition = g_sceneCB.light2Position.xyz;
            lightColor = g_sceneCB.light2DiffuseColor.xyz;
            lightSize = g_sceneCB.light2Size;
            lightIntensity = g_sceneCB.light2Intensity;
        }
    }

    // Sample a point on the area light.
    float2 lightSample = float2(random(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks + 1), random(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks + 2))
                            * lightSize - lightSize * 0.5f;
    lightPosition += float3(lightSample.x, 0, lightSample.y);

    float lightDist = distance(hitPosition, lightPosition);
    float3 lightDirection = normalize(lightPosition - hitPosition);

    // Next event estimation.
    float3 lightContrib;
    if (NextEventEstimation(rayPayload.recursionDepth, hitPosition, lightDist, lightDirection, lightColor, lightIntensity, lightSize, lightContrib))
    {
        // Get the local space basis.
        float3 X, Y;
        ComputeLocalSpace(normal, X, Y);

        // Compute the BSDF.
        return lightContrib * ComputeDisneyBSDF(lightDirection, -WorldRayDirection(), normal, X, Y) / pdf;
    }

    return float3(0, 0, 0);
}

void ClosestHitHelper(inout RayPayload rayPayload, in float3 normal, in float3 hitPosition)
{
    float4 color = float4(0, 0, 0, 0);

    // PERFORMANCE TIP: it is recommended to avoid values carry over across TraceRay() calls.
    // Therefore, in cases like retrieving HitWorldPosition(), it is recomputed every time.
    if (g_sceneCB.raytracingType > 0)
    {
        color.xyz = ShadePathTracing(rayPayload, normal, hitPosition);
    }
    else
    {
        // Shadow component.
        // Trace a shadow ray only if recursion depth allows it.
        Ray shadowRay = {hitPosition, normalize(g_sceneCB.lightPosition.xyz - hitPosition)};
        bool shadowRayHit = rayPayload.recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, rayPayload.recursionDepth);

        // Reflected component.
        float4 reflectedColor = float4(0, 0, 0, 0);
        if (l_materialCB.reflectanceCoef > 0.001)
        {
            // Trace a reflection ray.
            Ray reflectionRay = {hitPosition, reflect(WorldRayDirection(), normal)};
            float4 reflectionColor = TraceRadianceRay(reflectionRay, rayPayload.recursionDepth);

            float3 fresnelR = FresnelReflectanceSchlick(WorldRayDirection(), normal, l_materialCB.albedo.xyz);
            reflectedColor = l_materialCB.reflectanceCoef * float4(fresnelR, 1) * reflectionColor;
        }

        // Calculate final color.
        float4 phongColor = CalculatePhongLighting(l_materialCB.albedo, normal, shadowRayHit, l_materialCB.diffuseCoef, l_materialCB.specularCoef, l_materialCB.specularPower);
        color = phongColor + reflectedColor;
    }

    // Apply visibility falloff.
    float t = RayTCurrent();
    color = lerp(color, BackgroundColor, 1.0 - exp(-0.000002 * t * t * t));

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
    float4 backgroundColor = float4(BackgroundColor);
    rayPayload.color = backgroundColor;
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

[shader("intersection")] void IntersectionShader_VolumetricPrimitive()
{
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
    VolumetricPrimitive::Enum primitiveType = (VolumetricPrimitive::Enum)l_aabbCB.primitiveType;

    float thit;
    ProceduralPrimitiveAttributes attr = (ProceduralPrimitiveAttributes)0;
    if (RayVolumetricGeometryIntersectionTest(localRay, primitiveType, thit, attr, g_sceneCB.elapsedTime))
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
    if (RaySignedDistancePrimitiveTest(localRay, primitiveType, thit, attr, l_materialCB.stepScale))
    {
        PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3)aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3)ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, /*hitKind*/ 0, attr);
    }
}

#endif // RAYTRACING_HLSL