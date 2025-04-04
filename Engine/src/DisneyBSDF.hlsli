#ifndef DISNEYBSDF_H
#define DISNEYBSDF_H

#include "BSDF.hlsli"

// ##################################################################### //
// ############################# DISNEY BSDF ########################### //
// ##################################################################### //

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

// Mixes the diffuse and metallic specular components.
float DisneyFresnelMix(in float dotVH, in float eta, in float metallic)
{
    float metallicFresnel = FresnelReflectanceSchlick(dotVH);
    float dielectricFresnel = FresnelDielectric(dotVH, eta);
    return lerp(dielectricFresnel, metallicFresnel, metallic);
}

// Computes both the sheen color and the diffuse specular color.
// It is used in the Fresnel term of the Metal BRDF.
void ComputeSpecularColor(in PBRPrimitiveConstantBuffer material, in float eta, out float3 specularColor, out float3 sheenColor)
{
    float luminance = GetLuminance(material.albedo.xyz);
    float3 tint = luminance > 0.0f ? material.albedo.xyz / luminance : float3(1.0f, 1.0f, 1.0f);
    float3 R0 = R0FromIOR(eta);
    sheenColor = lerp(float3(1.0f, 1.0f, 1.0f), tint, material.sheenTint);
    specularColor = lerp(R0 * sheenColor, material.albedo.xyz, material.metallic);
}

// Computes the intensity of the clearcoat BRDF lobe.
float3 EvaluateClearcoat(in PBRPrimitiveConstantBuffer material, in bool anisotropic, in float3 V, in float3 L, in float3 H, out float pdf)
{
    pdf = 0.0f;

    // Reject if we are below the surface.
    if (L.y <= 0.0f)
        return float3(0.0f, 0.0f, 0.0f);

    // Precompute dot products.
    float dotVH = dot(V, H);

    // Compute anisotropic roughness.
    float a = lerp(0.1f, 0.001f, material.clearcoatGloss);

    // Compute D term for the clearcoat.
    float D = DGTR1(H.y, a);

    // Compute F term with IOR = 1.5 and F0 = 0.04.
    float F = lerp(0.04f, 1.0f, FresnelDielectric(dotVH, 0.6667f));

    // Compute geometric shadowing term G as product of two G1 terms.
    float G = anisotropic ? (SmithG1Anisotropic(L.x, L.z, L.y, 0.25f, 0.25f) * SmithG1Anisotropic(V.x, V.z, V.y, 0.25f, 0.25f)) : (SmithG1(L.y, 0.25f) * SmithG1(V.y, 0.25f));

    // Compute pdf for sampling.
    pdf = 0.25f * H.y * D / dotVH;

    return float3(0.25f, 0.25f, 0.25f) * material.clearcoat * D * F * G / (4.0f * L.y * V.y);
}

// Disney Diffuse BRDF, with sheen and approximated subsurface scattering.
float3 EvaluateDiffuse(in PBRPrimitiveConstantBuffer material, in float3 sheenColor,
                       in float3 V, in float3 L, in float3 H, out float pdf)
{
    pdf = 0.0f;

    // Reject if we are below the surface.
    if (L.y <= 0.0f)
        return float3(0.0f, 0.0f, 0.0f);

    // Precompute dot products.
    float dotHL = dot(H, L);

    // Compute diffuse component.
    float fd90 = FD90(material.roughness, dotHL);
    float d = FD(fd90, V.y) * FD(fd90, L.y);

    // Compute subsurface scattering approximation.
    float fss90 = FSS90(material.roughness, dotHL);
    float Fss = FD(fss90, V.y) * FD(fss90, L.y);
    float ss = 1.25f * (Fss * (1.0f / (L.y + V.y) - 0.5f) + 0.5f);

    // Compute the sheen component.
    float3 sh = material.sheen * FresnelReflectanceSchlick(dotHL) * sheenColor;

    // Compute the pdf.
    pdf = L.y * INV_PI;

    return (1.0f - material.metallic) * (1.0f - material.specularTransmission) * (INV_PI * material.albedo.xyz * lerp(d, ss, material.subsurface) + sh);
}

// Disney Specular BRDF (only reflection) - standard Cook-Torrance microfacet BRDF.
// This is modified to include a dielectric specular term that is missing in the diffuse model.
float3 EvaluateSpecularReflection(in PBRPrimitiveConstantBuffer material, in float eta, in float3 specularColor, in float3 V, in float3 L, in float3 H,
                                  in bool anisotropic, in float ax, in float ay, out float pdf)
{
    pdf = 0.0f;

    // Reject if we are below the surface.
    if (L.y <= 0.0f)
        return float3(0.0f, 0.0f, 0.0f);

    // Precompute dot products.
    float dotLH = dot(L, H);
    float dotVH = dot(V, H);

    // Compute distribution term D.
    float D = anisotropic ? DGTR2Anisotropic(H.x, H.z, H.y, ax, ay) : DGTR2(H.y, material.roughness);

    // compute Fresnel chromatic component (to account for dielectric specular reflection)
    float3 F = lerp(specularColor, float3(1.0f, 1.0f, 1.0f), DisneyFresnelMix(dotVH, eta, material.metallic));

    // compute geometric shadowing term G as product of two G1 terms
    float Gv = anisotropic ? SmithG1Anisotropic(V.x, V.z, V.y, ax, ay) : SmithG1(V.y, material.roughness);
    float G = Gv * (anisotropic ? SmithG1Anisotropic(L.x, L.z, L.y, ax, ay) : SmithG1(L.y, material.roughness));

    // Compute pdf.
    pdf = 0.25f * Gv * max(0.0f, dotVH) * D / (V.y * dotVH);

    return 0.25f * D * F * G / (L.y * V.y);
}

// Disney Specular BSDF (only refraction).
float3 EvaluateSpecularTransmission(in PBRPrimitiveConstantBuffer material, in float eta, in float3 V, in float3 L,
                                    in float3 H, in bool anisotropic, in float ax, in float ay, out float pdf)
{
    pdf = 0.0f;

    // Reject if we are above the surface.
    if (L.y >= 0.0f)
        return float3(0.0f, 0.0f, 0.0f);

    // Precompute dot products.
    float dotLH = dot(L, H);
    float dotVH = dot(V, H);
    float tmp = 1.0f / sq(abs(dotLH) + dotVH * eta);

    // compute distribution term D
    float D = anisotropic ? DGTR2Anisotropic(H.x, H.z, H.y, ax, ay) : DGTR2(H.y, material.roughness);

    // compute Fresnel term
    float F = FresnelDielectric(abs(dotVH), eta);

    // compute geometric shadowing term G as product of two G1 terms
    float Gv = anisotropic ? SmithG1Anisotropic(V.x, V.z, V.y, ax, ay) : SmithG1(V.y, material.roughness);
    float G = Gv * (anisotropic ? SmithG1Anisotropic(L.x, L.z, L.y, ax, ay) : SmithG1(L.y, material.roughness));

    // Compute pdf.
    pdf = Gv * max(0.0f, dotVH) * D * tmp * abs(dotLH) / V.y;

    return (1.0f - material.metallic) * material.specularTransmission * sqrt(material.albedo.xyz) * (1.0f - F) * D * G * abs(dotLH) * abs(dotVH) * sq(eta) * tmp / abs(L.y * V.y);
}

// Computes the lobes' probability distribution functions for the microfacet model.
// Ignores the sheen lobe because its influence is minimal.
void ComputePdfs(in PBRPrimitiveConstantBuffer material, in float3 specularColor, in float fresnelMix,
                 out float pSpecularReflection, out float pDiffuse, out float pClearcoat, out float pSpecularRefraction)
{
    float wDiffuse = GetLuminance(material.albedo.xyz) * (1.0f - material.metallic) * (1.0f - material.specularTransmission);
    float wSpecularReflection = GetLuminance(lerp(specularColor, float3(1, 1, 1), fresnelMix));
    float wSpecularRefraction = GetLuminance(material.albedo.xyz) * (1.0f - fresnelMix) * material.specularTransmission * (1.0f - material.metallic);
    float wClearcoat = material.clearcoat * (1.0f - material.metallic);

    float totalWeight = wDiffuse + wSpecularReflection + wClearcoat + wSpecularRefraction;

    pSpecularReflection = wSpecularReflection / totalWeight;
    pDiffuse = wDiffuse / totalWeight;
    pClearcoat = wClearcoat / totalWeight;
    pSpecularRefraction = wSpecularRefraction / totalWeight;
}

// Computes the Disney BSDF as an aggregate of all the components.
float3 EvaluateDisneyBSDF(in PBRPrimitiveConstantBuffer material, in bool anisotropic,
                          in float eta, in float3 V, in float3 L, in float3 N, out float pdf)
{
    pdf = 0.0f;
    float3 reflectance = float3(0.0f, 0.0f, 0.0f);

    // Transform V and L into the tangent space.
    float3 T, B;
    ComputeLocalSpace(N, T, B);
    V = GetWorldToTangent(N, T, B, V);
    L = GetWorldToTangent(N, T, B, L);

    // Compute the half vector (with correction for transmission).
    float3 H;
    if (L.y >= 0.0f)
        H = normalize(L + V);
    else
        H = normalize(L + V * eta);

    // Correct the half vector if it is below the surface.
    if (H.y < 0.0f)
        H = -H;

    // Compute dot products.
    float dotVH = dot(V, H);

    // Compute anisotropic parameters.
    float ax = 0.0f, ay = 0.0f;
    if (anisotropic)
        ComputeAnisotropicAlphas(material.roughness, material.anisotropic, ax, ay);

    // Compute specular and sheen color.
    float3 specularColor, sheenColor;
    ComputeSpecularColor(material, eta, specularColor, sheenColor);

    // Compute lobe pdfs.
    float pDiffuse, pSpecularReflection, pClearcoat, pSpecularRefraction;
    float fresnelMix = DisneyFresnelMix(dotVH, eta, material.metallic);
    ComputePdfs(material, specularColor, fresnelMix, pSpecularReflection, pDiffuse, pClearcoat, pSpecularRefraction);

    // Evaluate the lobes.
    float lobePdf;

    // apply diffuse, sheen and approximate subsurface scattering
    if (pDiffuse > 0.0f && L.y > 0.0f)
    {
        reflectance += EvaluateDiffuse(material, sheenColor, V, L, H, lobePdf);
        pdf += pDiffuse * lobePdf;
    }

    // apply specular reflection (only if visible)
    if (pSpecularReflection > 0.0f && L.y > 0.0f && V.y > 0.0f)
    {
        reflectance += EvaluateSpecularReflection(material, eta, specularColor, V, L, H, anisotropic, ax, ay, lobePdf);
        pdf += pSpecularReflection * lobePdf;
    }

    // apply clearcoat (only if visible)
    if (pClearcoat > 0.0f && L.y > 0.0f && V.y > 0.0f)
    {
        reflectance += EvaluateClearcoat(material, anisotropic, V, L, H, lobePdf);
        pdf += pClearcoat * lobePdf;
    }

    // apply transmission
    if (pSpecularRefraction > 0.0f && L.y < 0.0f)
    {
        reflectance += EvaluateSpecularTransmission(material, eta, V, L, H, anisotropic, ax, ay, lobePdf);
        pdf += pSpecularRefraction * lobePdf;
    }

    return reflectance * abs(L.y); // factor in the cosine term
}

float3 SampleDisneyBSDF(inout uint rng_state, in PBRPrimitiveConstantBuffer material, in float eta,
                        in bool anisotropic, in float3 V, in float3 N, out float3 L, out float pdf)
{
    pdf = 0.0f;
    float3 reflectance = float3(0.0f, 0.0f, 0.0f);

    // Compute anisotropic parameters.
    float ax = 0.0f, ay = 0.0f;
    ComputeAnisotropicAlphas(material.roughness, material.anisotropic, ax, ay);

    // Transform V into the tangent space.
    float3 T, B;
    ComputeLocalSpace(N, T, B);
    V = GetWorldToTangent(N, T, B, V);

    // Compute the specular and sheen color.
    float3 specularColor, sheenColor;
    ComputeSpecularColor(material, eta, specularColor, sheenColor);

    // Compute the lobe weights.
    float pDiffuse, pSpecularReflection, pClearcoat, pSpecularRefraction;
    float fresnelMix = DisneyFresnelMix(V.y, eta, material.metallic);
    ComputePdfs(material, specularColor, fresnelMix, pSpecularReflection, pDiffuse, pClearcoat, pSpecularRefraction);

    // Generate random numbers.
    float eps0 = random(rng_state);
    float eps1 = random(rng_state);
    float choice = random(rng_state);

    // Pick one of the lobes to sample.
    float3 cdf;
    float3 H;
    cdf.x = pDiffuse;
    cdf.y = cdf.x + pSpecularReflection;
    cdf.z = cdf.y + pClearcoat;
    if (choice < cdf.x)
    {
        // Sample the diffuse lobe with cosine-weighted distribution for outgoing direction.
        L = CosineSampleHemisphere(eps0, eps1, pdf);
        H = normalize(V + L);

        reflectance = EvaluateDiffuse(material, sheenColor, V, L, H, pdf);
        pdf *= pDiffuse;
    }
    else if (choice < cdf.y)
    {
        // Sample the specular reflection lobe with VNDF for the half vector.
        if (anisotropic)
            H = SampleVNDFAnisotropic(eps0, eps1, ax, ay, V);
        else
            H = SampleVNDF(eps0, eps1, material.roughness, V);
        
        // Correct the half vector if it is below the surface.
        if (H.y < 0.0f)
            H = -H;

        // Reflect the view vector.
        L = normalize(reflect(-V, H));

        reflectance = EvaluateSpecularReflection(material, eta, specularColor, V, L, H, anisotropic, ax, ay, pdf);
        pdf *= pSpecularReflection;
    }
    else if (choice < cdf.z)
    {
        // Compute anisotropic roughness.
        float a = lerp(0.1f, 0.001f, material.clearcoatGloss);

        // Sample the clearcoat lobe by sampling GTR1 distribution.
        H = SampleDGTR1(eps0, eps1, a);

        // Correct the half vector if it is below the surface.
        if (H.y < 0.0f)
            H = -H;

        // Reflect the view vector.
        L = normalize(reflect(-V, H));

        reflectance = EvaluateClearcoat(material, anisotropic, V, L, H, pdf);
        pdf *= pClearcoat;
    }
    else
    {
        // Sample the specular refraction lobe with VNDF for the half vector.
        if (anisotropic)
            H = SampleVNDFAnisotropic(eps0, eps1, ax, ay, V);
        else
            H = SampleVNDF(eps0, eps1, material.roughness, V);

        // Correct the half vector if it is below the surface.
        if (H.y < 0.0f)
            H = -H;

        // Compute the refracted direction.
        L = refract(-V, H, eta);
        
        // Check if the refraction is total internal.
        if (dot(L, L) == 0.0f) {
            pdf = 0.0f;
            return float3(0.0f, 0.0f, 0.0f);
        }
        
        L = normalize(L);

        reflectance = EvaluateSpecularTransmission(material, eta, V, L, H, anisotropic, ax, ay, pdf);
        pdf *= pSpecularRefraction;
    }

    // Transform the outgoing direction back to world space.
    L = GetTangentToWorld(N, T, B, L);

    return reflectance * abs(dot(N, L)); // factor in the cosine term
}

#endif // DISNEYBSDF_H