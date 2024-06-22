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
//******************------ PBR functions -------*****************************
//***************************************************************************

// Computes the intensity of the sheen lobe.
float3 ComputeSheen(in float dotLH, in float dotNL)
{
    // compute the luminance of albedo
    float luminance = dot(l_pbrCB.albedo.xyz, float3(0.3f, 0.6f, 1.0f));

    // sheen lobe is tinted by the albedo hue and saturation
    float3 tint = luminance > 0.0f ? lerp(float3(1.0f, 1.0f, 1.0f), l_pbrCB.albedo.xyz / luminance, l_pbrCB.sheenTint) : float3(1.0f, 1.0f, 1.0f);

    // sheen lobe is using the Schlick-Fresnel shape
    return l_pbrCB.sheen * tint * pow(saturate(1.0f - dotLH), 5) * dotNL;
}

// Computes the intensity of the clearcoat lobe.
float ComputeClearCoat(in float dotHL, in float dotNH,
                       in float dotLX, in float dotLY, in float dotLN, in float dotVX, in float dotVY, in float dotVN, out float pdf)
{
    // compute anisotropic roughness
    float a = lerp(0.1f, 0.001f, l_pbrCB.clearcoatGloss);

    // compute D term for the clearcoat
    float D = DGTR1(dotNH, a);

    // compute F term with Schlick approximation with IOR = 1.5 (F0 = 0.04)
    float F = FresnelReflectanceSchlick(abs(dotHL), 0.04f);

    // compute geometric shadowing term G as product of two G1 terms
    float G = SmithG1Anisotropic(dotLX, dotLY, dotLN, 0.25f, 0.25f) * SmithG1Anisotropic(dotVX, dotVY, dotVN, 0.25f, 0.25f);

    // compute pdf for sampling
    pdf = 0.25f * D / abs(dotLN);

    return 0.25f * D * F * G / abs(dotVN);
}

// Samples the clear coat lobe.
float3 SampleClearcoat(in float eps0, in float eps1, in float3 V, in float3 N, in float3 X, in float3 Y, out float pdf, out float3 reflectance)
{
    // We use again the roughness as 0.25.
    float a = 0.25f;

    // Sample a micronormal in the local shading frame.
    float cosTheta = sqrt((1.0f - pow(sq(a), 1.0f - eps0)) / (1.0f - sq(a)));
    float phi = 2.0f * PI * eps1;
    float3 H = float3(cos(phi) * cosTheta, cosTheta, sin(phi) * sqrt(1.0f - sq(cosTheta)));

    // Do not go below the surface.
    if (dot(H, N) < 0.0f)
        H = -H;

    // Reflect the view vector.
    float3 direction = reflect(V, H);

    // Compute the reflectance.
    float dotLH = dot(direction, H);
    float dotNH = dot(N, H);
    float dotLX = dot(direction, X);
    float dotLY = dot(direction, Y);
    float dotLN = dot(direction, N);
    float dotVX = dot(V, X);
    float dotVY = dot(V, Y);
    float dotVN = dot(V, N);
    float clearcoat = ComputeClearCoat(dotLH, dotNH, dotLX, dotLY, dotLN, dotVX, dotVY, dotVN, pdf);
    reflectance = float3(clearcoat, clearcoat, clearcoat);

    // Compute the pdf.
    float D = DGTR1(dotNH, a);
    pdf = 0.25f * D / abs(dotLH);

    return direction;
}

// Disney Diffuse model.
float3 ComputeDiffuse(in float dotHL, in float dotNV, in float dotNL)
{
    float fd90 = FD90(l_pbrCB.roughness, dotHL);
    return l_pbrCB.albedo.xyz * INV_PI * FD(fd90, dotNV) * FD(fd90, dotNL) * abs(dotNL);
}

// Samples the diffuse lobe.
float3 SampleDiffuse(in float eps0, in float eps1, in float3 V, in float3 N, in float3 X, in float3 Y, out float pdf, out float3 reflectance)
{
    // Sample the hemisphere with cosine-weighted distribution.
    float3 direction = CosineSampleHemisphere(eps0, eps1, N, X, Y, pdf);

    // Compute the reflectance.
    float3 H = normalize(V + direction);
    reflectance = ComputeDiffuse(dot(direction, H), dot(N, V), dot(N, direction));
    reflectance += ComputeSheen(dot(direction, H), dot(N, direction));

    return direction;
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

    // compute Fresnel achromatic component (to account for dielectric specular reflection)
    float3 F = DisneyFresnel(dotLH, l_pbrCB.albedo.xyz, l_pbrCB.specularTint, l_pbrCB.specular, l_pbrCB.eta, l_pbrCB.metallic);

    // compute geometric shadowing term G as product of two G1 terms
    float G = SmithG1Anisotropic(dotLX, dotLY, dotLN, ax, ay) * SmithG1Anisotropic(dotVX, dotVY, dotVN, ax, ay);

    return 0.25f * D * F * G / abs(dotVN);
}

// Samples the metallic lobe.
float3 SampleMetallic(in float eps0, in float eps1, in float3 V, in float3 N, in float3 X, in float3 Y,
    in float ax, in float ay, out float pdf, out float3 reflectance)
{
    // Sample the visible normals of the Smith GGX model.
    float3 H = VisibleNormalsSampling(eps0, eps1, ax, ay, V);

    // Reflect the view vector.
    float3 direction = reflect(V, H);

    // Since we are sampling the visible normals, we're left with only a few terms.
    float3 F = DisneyFresnel(dot(H, direction), l_pbrCB.albedo.xyz, l_pbrCB.specularTint, l_pbrCB.specular, l_pbrCB.eta, l_pbrCB.metallic);
    float Gv = SmithG1Anisotropic(dot(V, X), dot(V, Y), dot(V, N), ax, ay);
    reflectance = F * Gv;

    // Compute the pdf.
    pdf = VisibleNormalsPdf(ax, ay, dot(direction, H), dot(V, X), dot(V, Y), dot(V, N), dot(H, X), dot(H, Y), dot(H, N)) * 0.25f / abs(dot(H, V));

    return direction;
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
        return 0.25f * common * F * l_pbrCB.albedo.xyz;
    else // take square root to account for the fact that the light is coming from the other side
        return common * (1.0f - F) * abs(dotLH * dotHV) / sq(dotHV + l_pbrCB.eta * dotLH) * sqrt(l_pbrCB.albedo.xyz);
}

// Samples the glass lobe.
float3 SampleGlass(in bool inside, in float eps0, in float eps1, in float3 V, in float dotVX, in float dotVY, in float dotVN, in float dotLH,
                   in float dotHX, in float dotHY, in float dotHN, in float dotHV, in float ax, in float ay, out float pdf, out float3 reflectance)
{
    // Sample the visible normals of the Smith GGX model.
    float3 H = VisibleNormalsSampling(eps0, eps1, ax, ay, V);

    // Compute pdf for sampling.
    pdf = VisibleNormalsPdf(ax, ay, dotLH, dotVX, dotVY, dotVN, dotHX, dotHY, dotHN) * 0.25f / abs(dotHV);

    // Compute IOR fraction.
    float eta = inside ? l_pbrCB.eta : 1.0f / l_pbrCB.eta;

    return float3(0.0f, 0.0f, 0.0f);
}

// Computes the Disney BSDF as an aggregate of all the defined lobes.
float3 ComputeDisneyBSDF(in float3 L, in float3 V, in float3 N, in float3 X, in float3 Y, out float pdf)
{
    float3 reflectance = float3(0.0f, 0.0f, 0.0f);

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

    // check if we are inside the surface
    bool inside = dotNV < 0.0f;

    // compute anisotropic parameters
    float ax, ay;
    ComputeAnisotropicAlphas(l_pbrCB.roughness, l_pbrCB.anisotropic, ax, ay);

    // compute pdfs
    float pDiffuse, pMetal, pClearcoat, pGlass, wDiffuse, wMetal, wClearcoat, wGlass;
    ComputePdfs(inside, l_pbrCB.metallic, l_pbrCB.specularTransmission, l_pbrCB.clearcoat,
                pMetal, pDiffuse, pClearcoat, pGlass, wMetal, wDiffuse, wClearcoat, wGlass);

    bool specularVisible = dotNL > 0.0f && dotNV > 0.0f; // true if both L and V are on the outer side of the surface
    float lobePdf;
    pdf = 0.0f;

    // apply diffuse and sheen
    if (wDiffuse > 0.0f)
    {
        float3 diffuse = ComputeDiffuse(dotLH, dotNV, dotNL);
        pdf += pDiffuse * dotNL * INV_PI;

        float3 sheen = float3(0.0f, 0.0f, 0.0f);
        if (l_pbrCB.sheen > 0.0f)
            sheen = ComputeSheen(dotLH, dotNL);

        reflectance += wDiffuse * (diffuse + (1.0f - l_pbrCB.metallic) * sheen);
    }

    // apply metallic
    if (wMetal > 0.0f && specularVisible)
    {
        float3 metallic = ComputeMetallicBRDF(dotHX, dotHY, dotNH, dotLH, dotLX, dotLY, dotNL, dotVX, dotVY, dotNV);
        // compute pdf for sampling
        pdf += pMetal * VisibleNormalsPdf(ax, ay, dotLH, dotVX, dotVY, dotNV, dotHX, dotHY, dotNH) * 0.25f / abs(dot(H, V));
        reflectance += wMetal * metallic;
    }

    // apply clearcoat
    if (wClearcoat > 0.0f && l_pbrCB.clearcoat > 0.0f && specularVisible)
    {
        float clearcoat = ComputeClearCoat(dotLH, dotNH, dotLX, dotLY, dotNL, dotVX, dotVY, dotNV, lobePdf);
        pdf += pClearcoat * lobePdf;
        reflectance += wClearcoat * float3(clearcoat, clearcoat, clearcoat);
    }

    // apply transmission
    if (wGlass > 0.0f)
    {
        float ax, ay;
        ComputeAnisotropicAlphas(l_pbrCB.roughness, l_pbrCB.anisotropic, ax, ay);

        float3 transmission = ComputeGlassBSDF(dotNV * dotNV <= 0.0f, dotHX, dotHY, dotNH, dotLH, dotVH, dotLX, dotLY, dotNL, dotVX, dotVY, dotNV, ax, ay);
        pdf += pGlass * lobePdf;
        reflectance += wGlass * transmission;
    }

    return reflectance;
}

float3 SampleDisney(inout uint rng_state, in float3 V, in float3 N, in float3 X, in float3 Y, out float pdf, out float3 reflectance)
{
    bool inside = dot(N, V) < 0.0f;

    // Compute the lobe weights.
    float pDiffuse, pMetal, pClearcoat, pGlass, wDiffuse, wMetal, wClearcoat, wGlass;
    ComputePdfs(inside, l_pbrCB.metallic, l_pbrCB.specularTransmission, l_pbrCB.clearcoat,
                pMetal, pDiffuse, pClearcoat, pGlass, wMetal, wDiffuse, wClearcoat, wGlass);

    float pdfLobe;
    float3 direction;

    // Generate two random numbers.
    float eps0 = random(rng_state);
    float eps1 = random(rng_state);

    // Pick one of the lobes to sample.
    float r = random(rng_state);
    if (r < pDiffuse)
    {
        // Sample the diffuse lobe.
        pdfLobe = pDiffuse;
        direction = SampleDiffuse(eps0, eps1, V, N, X, Y, pdf, reflectance);
    }
    else if (r < pDiffuse + pMetal)
    {
        // Sample the metallic lobe.
        float ax, ay;
        ComputeAnisotropicAlphas(l_pbrCB.roughness, l_pbrCB.anisotropic, ax, ay);
        pdfLobe = pMetal;
        direction = SampleMetallic(eps0, eps1, V, N, X, Y, ax, ay, pdf, reflectance);
    }
    else if (r < pDiffuse + pMetal + pClearcoat)
    {
        // Sample the clearcoat lobe.
        pdfLobe = pClearcoat;
        direction = SampleClearcoat(eps0, eps1, V, N, X, Y, pdf, reflectance);
    }
    else
    {
        // Sample the clearcoat lobe.
        pdfLobe = pClearcoat;
        direction = SampleClearcoat(eps0, eps1, V, N, X, Y, pdf, reflectance);
    }

    // Weigh the pdf
    pdf *= pdfLobe;

    return direction;
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

// Phong lighting model = diffuse + specular components.
float4 CalculatePhongLighting(in float3 lightPosition, in float3 lightColor, in float4 albedo, in float3 normal,
    in bool isInShadow, in float diffuseCoef = 1.0, in float specularCoef = 1.0, in float specularPower = 50)
{
    float3 hitPosition = HitWorldPosition();
    float shadowFactor = isInShadow ? InShadowRadiance : 1.0;
    float3 incidentLightRay = normalize(hitPosition - lightPosition);

    // Diffuse component.
    float Kd = CalculateDiffuseCoefficient(hitPosition, incidentLightRay, normal);
    float4 diffuseColor = shadowFactor * diffuseCoef * Kd * float4(lightColor, 1.0f) * albedo;

    // Specular component.
    float4 specularColor = float4(0, 0, 0, 0);
    if (!isInShadow)
    {
        float4 lightSpecularColor = float4(1, 1, 1, 1);
        float4 Ks = CalculateSpecularCoefficient(hitPosition, incidentLightRay, normal, specularPower);
        specularColor = specularCoef * Ks * lightSpecularColor;
    }

    return diffuseColor + specularColor;
}

//***************************************************************************
//*****------ TraceRay wrappers for radiance and shadow rays. -------********
//***************************************************************************

// Trace a radiance ray into the scene and returns a shaded color.
float4 TraceRadianceRay(in Ray ray, in float4 throughput, in UINT currentRayRecursionDepth)
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
    rayDesc.TMin = 0.0f;
    rayDesc.TMax = 10000.0f;
    RayPayload rayPayload = {float4(0.0f, 0.0f, 0.0f, 0.0f), throughput, currentRayRecursionDepth + 1};
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
    float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), currentRecursionDepth);

    // Write the raytraced color to the output texture.
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
    float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), currentRecursionDepth);

    // Accumulate the color.
    const float lerpFactor = 1.0f * (g_sceneCB.pathFrameCacheIndex - 1) / g_sceneCB.pathFrameCacheIndex;
    g_renderTarget[DispatchRaysIndex().xy] = float4(lerp(color.xyz, g_renderTarget[DispatchRaysIndex().xy].xyz, lerpFactor), 1.0f);
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
            float4 color = TraceRadianceRay(ray, float4(1.0f, 1.0f, 1.0f, 1.0f), currentRecursionDepth);

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
bool NextEventEstimation(in uint recursionDepth, in float3 hitPosition, in float3 lightPosition, in float3 lightDiffuse, in float lightIntensity, in float lightSize,
    out float3 lightContrib, out float lightPdf)
{
    lightContrib = float3(0.0f, 0.0f, 0.0f);
    lightPdf = 0.0f;

    // The light is pointing downwards on the Y axis.
    float3 lightDir = normalize(lightPosition - hitPosition);
    float angle = dot(lightDir, float3(0.0f, 1.0f, 0.0f));
    if (angle <= 0.0f)
        return false;

    // Create a shadow ray towards the light.
    Ray shadowRay = {hitPosition, lightDir};
    bool shadowRayHit = recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, recursionDepth);
    if (!shadowRayHit)
    {
        // Calculate the light contribution and the pdf.
        lightPdf = 1.0f / (lightSize * lightSize);
        lightContrib = lightIntensity * lightDiffuse * angle / ((1.0f + 0.001f * length_toPow2(hitPosition - lightPosition)));
        return true;
    }
    return false;
}

float3 MIS(inout uint rng_state, in float3 hitPosition, in float3 lightPosition, in float3 lightColor, in float lightIntensity, in float lightSize,
    in float3 N, in float3 X, in float3 Y, in UINT recursionDepth)
{
    float3 reflectanceLightSampling = float3(0.0f, 0.0f, 0.0f);
    float3 reflectanceBSDFSampling = float3(0.0f, 0.0f, 0.0f);

    float2 lightSample = float2(random(rng_state), random(rng_state)) * lightSize - lightSize * 0.5f;
    lightPosition += float3(lightSample.x, 0, lightSample.y);
    float3 lightDirection = normalize(lightPosition - hitPosition);

    float bsdfPdf;

    // Sample light with MIS.
    float3 lightContrib;
    float lightPdf;
    if (NextEventEstimation(recursionDepth, hitPosition, lightPosition, lightColor, lightIntensity, lightSize, lightContrib, lightPdf))
    {
        reflectanceLightSampling = lightContrib * ComputeDisneyBSDF(lightDirection, -WorldRayDirection(), N, X, Y, bsdfPdf);
        reflectanceLightSampling *= PowerHeuristic(1.0f, lightPdf, 1.0f, bsdfPdf) / lightPdf;
    }

    // Sample the BSDF
    float samplePdf;
    float3 newDirection;
    if (g_sceneCB.importanceSamplingType == 0)
    {
        newDirection = UniformSampleHemisphere(random(rng_state), random(rng_state), N, X, Y, samplePdf);
        reflectanceBSDFSampling = ComputeDisneyBSDF(newDirection, -WorldRayDirection(), N, X, Y, bsdfPdf) / samplePdf;
    }
    else if (g_sceneCB.importanceSamplingType == 1)
    {
        newDirection = CosineSampleHemisphere(random(rng_state), random(rng_state), N, X, Y, samplePdf);
        reflectanceBSDFSampling = ComputeDisneyBSDF(newDirection, -WorldRayDirection(), N, X, Y, bsdfPdf) / samplePdf;
    }
    else
    {
        newDirection = SampleDisney(rng_state, -WorldRayDirection(), N, X, Y, samplePdf, reflectanceBSDFSampling);
        reflectanceBSDFSampling /= samplePdf;
    }

    if (NextEventEstimation(recursionDepth, hitPosition, lightPosition, lightColor, lightIntensity, lightSize, lightContrib, lightPdf))
    {
        reflectanceBSDFSampling = lightContrib * reflectanceBSDFSampling;
        reflectanceBSDFSampling *= PowerHeuristic(1.0f, bsdfPdf, 1.0f, lightPdf) / bsdfPdf;
    }

    return reflectanceLightSampling + reflectanceBSDFSampling;
}

float3 DoPathTracing(in RayPayload rayPayload, in float3 normal, in float3 hitPosition)
{
    // Initialize the random number generator.
    uint rng_state = hash(DispatchRaysIndex().xy, g_sceneCB.elapsedTicks + rayPayload.recursionDepth * 1337 + hash(hitPosition));
    float3 color = float3(0.0f, 0.0f, 0.0f); // Accumulated color

    // Get the local space basis.
    float3 X, Y;
    ComputeLocalSpace(normal, X, Y);

    if (g_sceneCB.onlyOneLightSample)
    {
        // If one light at a time, choose randomly and adjust pdf
        uint lightIndex = random(rng_state) * g_sceneCB.numLights;
        color += rayPayload.throughput.xyz * MIS(rng_state, hitPosition, g_lights[lightIndex].position, g_lights[lightIndex].emission,
            g_lights[lightIndex].intensity, g_lights[lightIndex].size, normal, X, Y, rayPayload.recursionDepth) * g_sceneCB.numLights;
    }
    else
    {
        // Sample all lights
        for (uint i = 0; i < g_sceneCB.numLights; i++)
        {
            color += rayPayload.throughput.xyz * MIS(rng_state, hitPosition, g_lights[i].position, g_lights[i].emission,
                g_lights[i].intensity, g_lights[i].size, normal, X, Y, rayPayload.recursionDepth);
        }
    }

    // Sample new direction
    float3 reflectance;
    float3 newDirection;
    float samplePdf, bsdfPdf;

    if (g_sceneCB.importanceSamplingType == 0)
    {
        newDirection = UniformSampleHemisphere(random(rng_state), random(rng_state), normal, X, Y, samplePdf);
        reflectance = ComputeDisneyBSDF(newDirection, -WorldRayDirection(), normal, X, Y, bsdfPdf) / samplePdf;
    }
    else if (g_sceneCB.importanceSamplingType == 1)
    {
        newDirection = CosineSampleHemisphere(random(rng_state), random(rng_state), normal, X, Y, samplePdf);
        reflectance = ComputeDisneyBSDF(newDirection, -WorldRayDirection(), normal, X, Y, bsdfPdf) / samplePdf;
    }
    else
    {
        newDirection = SampleDisney(rng_state, -WorldRayDirection(), normal, X, Y, samplePdf, reflectance);
        reflectance /= samplePdf;
    }

    // Shoot the next ray and accumulate the color.
    Ray newRay = {hitPosition, newDirection};
    color += TraceRadianceRay(newRay, float4(reflectance, 1.0f) * rayPayload.throughput, rayPayload.recursionDepth).xyz;

    return color * rayPayload.throughput.xyz;
}

void ClosestHitHelper(inout RayPayload rayPayload, in float3 normal, in float3 hitPosition)
{
    float4 color = float4(0, 0, 0, 1);

    // PERFORMANCE TIP: it is recommended to avoid values carry over across TraceRay() calls.
    // Therefore, in cases like retrieving HitWorldPosition(), it is recomputed every time.
    if (g_sceneCB.raytracingType > 0) // path tracing
    {
        color.xyz = DoPathTracing(rayPayload, normal, hitPosition);
    }
    else // Whitted-style ray tracing
    {
        // Reflected component.
        float4 reflectedColor = float4(0, 0, 0, 0);
        if (l_materialCB.reflectanceCoef > 0.001)
        {
            // Trace a reflection ray.
            Ray reflectionRay = {hitPosition, reflect(WorldRayDirection(), normal)};
            float4 reflectionColor = TraceRadianceRay(reflectionRay, rayPayload.throughput, rayPayload.recursionDepth);

            float3 fresnelR = FresnelReflectanceSchlick(WorldRayDirection(), normal, l_materialCB.albedo.xyz);
            reflectedColor = l_materialCB.reflectanceCoef * float4(fresnelR, 1) * reflectionColor;
        }

        color.xyz += reflectedColor.xyz;
        for (uint i = 0; i < g_sceneCB.numLights; i++)
        {
            // Shadow component.
            // Trace a shadow ray only if recursion depth allows it.
            Ray shadowRay = { hitPosition, normalize(g_lights[i].position - hitPosition) };
            bool shadowRayHit = rayPayload.recursionDepth < g_sceneCB.maxShadowRecursionDepth && TraceShadowRayAndReportIfHit(shadowRay, rayPayload.recursionDepth);
                                    
            // Calculate final color.
            float4 phongColor = CalculatePhongLighting(g_lights[i].position, g_lights[i].emission * g_lights[i].intensity, l_materialCB.albedo, normal,
                                                       shadowRayHit, l_materialCB.diffuseCoef, l_materialCB.specularCoef, l_materialCB.specularPower) /
                                (1.0f + 0.001f * length_toPow2(g_lights[i].position - hitPosition));
            color.xyz += phongColor.xyz;
        }
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