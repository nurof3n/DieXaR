#pragma once

#include "RayTracingHlslCompat.h"

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

// Represents a scene that embodies lights, materials, and geometry.
class Scene
{
public:
	void Load(std::vector<LightBuffer> lights, PrimitiveConstantBuffer planeMaterialCB,
		std::vector<PrimitiveConstantBuffer> aabbMaterialCB, std::vector<PBRPrimitiveConstantBuffer> pbrAabbMaterialCB,
		std::vector<PBRPrimitiveConstantBuffer> pbrMaterialCB);

	UINT GetLightCount() const { return static_cast<UINT>(m_lights.size()); }
	UINT GetAABBCount() const { return static_cast<UINT>(m_aabbMaterialCB.size()); }
	UINT GetMeshCount() const { return static_cast<UINT>(m_materialCB.size()); }

	static void SetLight(
		LightBuffer& light,
		const XMFLOAT3& position,
		const XMFLOAT3& color,
		float intensity = 1.0f,
		UINT type = LightType::Square,
		float size = 1.0f,
		const XMFLOAT3& direction = XMFLOAT3(0.0f, -1.0f, 0.0f))
	{
		light.position = position;
		light.emission = color;
		light.intensity = intensity;
		light.type = type;
		light.size = size;
		light.direction = direction;
	};

	static void SetAttributes(
		PrimitiveConstantBuffer& attributes,
		const XMFLOAT4& albedo,
		float reflectanceCoef = 0.0f,
		float diffuseCoef = 0.9f,
		float specularCoef = 0.7f,
		float specularPower = 20.0f,
		float stepScale = 1.0f)
	{
		attributes.albedo = albedo;
		attributes.reflectanceCoef = reflectanceCoef;
		attributes.diffuseCoef = diffuseCoef;
		attributes.specularCoef = specularCoef;
		attributes.specularPower = specularPower;
		attributes.stepScale = stepScale;
	};

	static void SetPBRAttributes(
		PBRPrimitiveConstantBuffer& attributes,
		const XMFLOAT4& albedo,
		float roughness = 0.0f,
		float metallic = 0.0f,
		float specularTint = 0.0f,
		float sheen = 0.0f,
		float sheenTint = 0.0f,
		float clearcoat = 0.0f,
		float clearcoatGloss = 0.0f,
		float anisotropic = 0.0f,
		float specularTransmission = 0.0f,
		float eta = 1.5f,
		float atDistance = 1.0f,
		const XMFLOAT3& extinction = XMFLOAT3(1.0f, 1.0f, 1.0f))
	{
		attributes.albedo = albedo;
		attributes.roughness = roughness;
		attributes.metallic = metallic;
		attributes.specularTint = specularTint;
		attributes.sheen = sheen;
		attributes.sheenTint = sheenTint;
		attributes.clearcoat = clearcoat;
		attributes.clearcoatGloss = clearcoatGloss;
		attributes.anisotropic = anisotropic;
		attributes.specularTransmission = specularTransmission;
		attributes.eta = eta;
		attributes.atDistance = atDistance;
		attributes.extinction = extinction;
	};

public:
	SceneTypes::SceneType m_sceneType{};

	XMVECTOR m_eye{};
	XMVECTOR m_at{};

	std::vector<LightBuffer> m_lights{};

	// Geometry
	

	// Root constants
	PrimitiveConstantBuffer m_planeMaterialCB{};
	PBRPrimitiveConstantBuffer m_pbrPlaneMaterialCB{};
	std::vector<PrimitiveConstantBuffer> m_aabbMaterialCB{};
	std::vector<PBRPrimitiveConstantBuffer> m_pbrAabbMaterialCB{};
	std::vector<PrimitiveConstantBuffer> m_materialCB{};
	std::vector<PBRPrimitiveConstantBuffer> m_pbrMaterialCB{};
};
