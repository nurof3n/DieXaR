#ifndef BSDF_H
#define BSDF_H

#include "RayTracingHlslCompat.h"
#include "RaytracingShaderHelper.hlsli"

// ##################################################################### //
// ############################# SAMPLING ############################## //
// ##################################################################### //

// Uniformly samples a hemisphere.
// eps0 and eps1 are random numbers in [0, 1).
// phi = 2 * PI * eps1
// theta = acos(1 - eps0)
// pdf = 1 / (2 * PI)
// y is the up vector.
float3 UniformSampleHemisphere(in float eps0, in float eps1, out float pdf)
{
    float sinTheta = sqrt(1.0f - eps0 * eps0);
    float phi = TWO_PI * eps1;

    // Compute the pdf.
    pdf = 0.5f * INV_PI;

    return float3(sinTheta * cos(phi), eps0, sinTheta * sin(phi));
}

// Uniformly samples a sphere.
// eps0 and eps1 are random numbers in [0, 1).
// phi = 2 * PI * eps1
// theta = acos(1 - 2 * eps0)
// pdf = 1 / (4 * PI)
// y is the up vector.
float3 UniformSampleSphere(in float eps0, in float eps1, out float pdf)
{
    float cosTheta = 1.0f - 2.0f * eps0;
    float sinTheta = sqrt(1.0f - sq(cosTheta));
    float phi = TWO_PI * eps1;

    // Compute the pdf.
    pdf = 0.25f * INV_PI;

    return float3(sinTheta * cos(phi), cosTheta, sinTheta * sin(phi));
}

// Cosine-weighted hemisphere sampling.
// eps0 and eps1 are random numbers in [0, 1).
// phi = 2 * PI * eps1
// theta = asin(sqrt(eps0))
// pdf = cos(theta) / PI
// y is the up vector.
float3 CosineSampleHemisphere(in float eps0, in float eps1, out float pdf)
{
    float sinTheta = sqrt(eps0);
    float cosTheta = sqrt(1.0f - eps0);
    float phi = TWO_PI * eps1;

    // Compute the pdf.
    pdf = INV_PI * cosTheta;

    return float3(sinTheta * cos(phi), cosTheta, sinTheta * sin(phi));
}

float3 SampleVNDF(in float eps0, in float eps1, in float roughness, in float3 V)
{
    float3 stretchedV = normalize(float3(roughness * V.x, V.y, roughness * V.z));
    
    float3 T, B;
    ComputeLocalSpace(stretchedV, T, B);

    float r = sqrt(eps0);
    float phi = eps1 * TWO_PI;
    float p1 = r * cos(phi);
    float p2 = r * sin(phi);
    float s = 0.5f * (1.0f + stretchedV.y);
    p2 = (1.0f - s) * sqrt(1.0f - sq(p1)) + s * p2;

    float3 normal = p1 * T + p2 * B + sqrt(max(0.0f, 1.0f - sq(p1) - sq(p2))) * stretchedV;
    return normalize(float3(roughness * normal.x, max(0.0f, normal.y), roughness * normal.z));
}

// https://hal.science/hal-01509746/document
float3 SampleVNDFAnisotropic(in float eps0, in float eps1, in float ax, in float ay, in float3 V)
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
    float3 normal = p1 * T + p2 * B + sqrt(max(0.0f, 1.0f - sq(p1) - sq(p2))) * stretchedV;
    return normalize(float3(ax * normal.x, max(0.0f, normal.y), ay * normal.z));
}

// float VisibleNormalsPdf(in float ax, in float ay, in float dotLH, in float dotVX, in float dotVY, in float dotVN,
//                         in float dotHX, in float dotHY, in float dotHN)
// {
//     float G1 = SmithG1Anisotropic(dotVX, dotVY, dotVN, ax, ay);
//     float D = DGTR2Anisotropic(dotHX, dotHY, dotHN, ax, ay);
//     return D * G1 * abs(dotLH);
// }

// ##################################################################### //
// ############################### F term ############################## //
// ##################################################################### //

float R0FromIOR(float ior)
{
    return sq((1.0f - ior) / (1.0f + ior));
}

// The Fresnel reflectance for a dielectric material.
// Eta is the index of refraction of the material.
// We assume outside medium is air (ior = 1).
float FresnelDielectric(in float cosThetaI, float eta)
{
    // Snell's law.
    float sinThetaTSq = sq(eta) * (1.0f - sq(cosThetaI));

    // Check for total internal reflection.
    if (sinThetaTSq > 1.0f)
        return 1.0f;
    
    float cosThetaT = sqrt(max(0.0f, 1.0f - sinThetaTSq));

    float Rs = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
    float Rp = (cosThetaT - eta * cosThetaI) / (cosThetaT + eta * cosThetaI);
    return 0.5f * (sq(Rs) + sq(Rp));
}

float FresnelReflectanceSchlick(in float cosi)
{
    return pow(clamp((1.0f - cosi), 0.0f, 1.0f), 5);
}

// Fresnel reflectance - schlick approximation.
float3 FresnelReflectanceSchlick(in float3 I, in float3 N, in float3 f0)
{
    return lerp(float3(1.0f, 1.0f, 1.0f), FresnelReflectanceSchlick(saturate(dot(-I, N))), f0);
}

// Fresnel reflectance - schlick approximation.
float FresnelReflectanceSchlick(in float3 I, in float3 N, in float f0)
{
    return lerp(1.0f, FresnelReflectanceSchlick(saturate(dot(-I, N))), f0);
}

// Fresnel reflectance - schlick approximation (but with dot product given).
float3 FresnelReflectanceSchlick(in float dotNL, in float3 f0)
{
    return lerp(float3(1.0f, 1.0f, 1.0f), FresnelReflectanceSchlick(dotNL), f0);
}

// Fresnel reflectance - schlick approximation (but with dot product given).
float FresnelReflectanceSchlick(in float dotNL, in float f0)
{
    return lerp(1.0f, FresnelReflectanceSchlick(dotNL), f0);
}

// Used to influence the grazing angle reflectance.
float FD90(in float roughness, in float dotWH)
{
    return 0.5f + 2.0f * roughness * sq(dotWH);
}

// Modified Fresnel equation for the diffuse model.
float FD(in float fd90, in float dotWN)
{
    return lerp(1.0f, fd90, FresnelReflectanceSchlick(dotWN));
}

// Fresnel reflectance for conductors.
float FSS90(in float roughness, in float dotHL)
{
    return roughness * sq(dotHL);
}

// ##################################################################### //
// ############################## D term ############################### //
// ##################################################################### //

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

    float a2 = sq(a);
    return INV_PI * (a2 - 1.0f) / (log(a2) * (1.0f + (a2 - 1.0f) * sq(dotNH)));
}

// Samples the DGTR1 distribution function.
// Result is in the tangent space.
float3 SampleDGTR1(in float eps0, in float eps1, in float roughness)
{
    // clamp the roughness
    float a = max(0.001f, roughness);
    float a2 = sq(a);

    float phi = eps0 * TWO_PI;

    float cosTheta = sqrt((1.0f - pow(a2, 1.0f - eps1)) / (1.0f - a2));
    float sinTheta = clamp(sqrt(1.0f - sq(cosTheta)), 0.0f, 1.0f);
    float sinPhi = sin(phi);
    float cosPhi = cos(phi);

    return float3(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//
// This computes the GTR2 distribution function for isotropic materials.
// The alpha value is used to control the roughness.
// Note: gamma is set to 2.
// Note: this distribution is normalized.
float DGTR2(in float dotNH, in float a)
{
    float a2 = sq(a);
    float t = 1.0f + (a2 - 1.0f) * sq(dotNH);
    return INV_PI * a2 / sq(t);
}

// Samples the DGTR2 distribution function.
// Result is in the tangent space.
float3 SampleDGTR2(in float eps0, in float eps1, in float roughness)
{
    // clamp the roughness
    float a = max(0.001f, roughness);
    float a2 = sq(a);

    float phi = eps0 * TWO_PI;

    float cosTheta = sqrt((1.0f - eps1) / (1.0f + (a2 - 1.0f) * eps1));
    float sinTheta = clamp(sqrt(1.0f - (sq(cosTheta))), 0.0f, 1.0f);
    float sinPhi = sin(phi);
    float cosPhi = cos(phi);

    return float3(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//
// This computes the GTR2 distribution function for anisotropic materials.
// The alpha x and y values are used to control the roughness in the x and y directions (tangent space).
// Note: gamma is set to 2.
// Note: this distribution is normalized.
float DGTR2Anisotropic(in float dotHX, in float dotHY, in float dotHN, in float ax, in float ay)
{
    return INV_PI / (ax * ay * sq(sq(dotHX / ax) + sq(dotHY / ay) + sq(dotHN)));
}

// Samples the DGTR2Anisotropic distribution function for anisotropic materials.
// Result is in the tangent space.
float3 SampleDGTR2Anisotropic(in float eps0, in float eps1, in float ax, in float ay)
{
    float phi = eps0 * TWO_PI;

    float sinPhi = ay * sin(phi);
    float cosPhi = ax * cos(phi);
    float tanTheta = sqrt(eps1 / (1.0f - eps1));

    return normalize(float3(tanTheta * cosPhi, 1.0f, tanTheta * sinPhi));
}

// ##################################################################### //
// ############################### G term ############################## //
// ##################################################################### //

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//      and https://www.jcgt.org/published/0003/02/03/paper.pdf
//      and https://sayan1an.github.io/pdfs/references/disneyBrdf.pdf
//
// This computes the unidirectional (separable) G1 masking function for isotropic materials.
// The alpha is the roughness parameter.
// This function is used to account for shadowing and masking effects in the microfacet model,
// to prevent energy conservation issues.
float SmithG1(in float dotWN, in float a)
{
    if (dotWN == 0.0f)
        return 0.0f;
    float a2 = sq(a);
    float dot2 = sq(dotWN);
    //return 2.0f / (1.0f + sqrt(1.0f + a2 * (1.0f / sq(dotWN) - 1.0f)));
    return (1.0f * dotWN) / (dotWN + sqrt(a2 + dot2 - a2 * dot2));
}

// Credits: https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
//      and https://www.jcgt.org/published/0003/02/03/paper.pdf
//      and https://sayan1an.github.io/pdfs/references/disneyBrdf.pdf
//
// This computes the unidirectional (separable) G1 masking function for anisotropic materials.
// The alpha x and y values are the aniostropic roughness parameters.
// This function is used to account for shadowing and masking effects in the microfacet model,
// to prevent energy conservation issues.
float SmithG1Anisotropic(in float dotWX, in float dotWY, in float dotWN, in float ax, in float ay)
{
    float inv_a2 = (sq(dotWX * ax) + sq(dotWY * ay)) / sq(dotWN);
    float lambda = -0.5f + 0.5f * sqrt(1.0f + inv_a2);
    return 1.0f / (1.0f + lambda);
}

#endif // BSDF_H