#pragma once

#include "RaytracingSceneDefines.h"

// Represents a scene that embodies lights, materials, and geometry.
class Scene
{
public:
	UINT GetLightCount() const { return static_cast<UINT>(m_lights.size()); }
	int CountGeometryLights() const {
		int count = 0;
		for (int i = 0; i < m_lights.size(); ++i)
			if (m_lights[i].type == LightType::Square)
				count++;
		return count;
	}
	std::vector<int> GetGeometryLightsIndices() const {
		std::vector<int> geometryLights;
		for (int i = 0; i < m_lights.size(); ++i)
			if (m_lights[i].type == LightType::Square)
				geometryLights.push_back(i);
		return geometryLights;
	}
	UINT GetPrimitiveCount() const {
		UINT totalPrimitiveCount = 0;
		for (UINT i = 0; i < m_primitiveCount.size(); ++i)
			totalPrimitiveCount += m_primitiveCount[i];
		return totalPrimitiveCount;
	}
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
		XMVECTOR dir = XMLoadFloat3(&direction);
		XMStoreFloat3(&light.direction, XMVector3Normalize(dir));
	};

	void SetAttributes(
		UINT materialIndex,
		const XMFLOAT4& albedo,
		float reflectanceCoef = 0.0f,
		float diffuseCoef = 0.9f,
		float specularCoef = 0.7f,
		float specularPower = 20.0f,
		float stepScale = 1.0f)
	{
		PrimitiveConstantBuffer attributes;
		attributes.materialIndex = materialIndex;
		attributes.albedo = albedo;
		attributes.reflectanceCoef = reflectanceCoef;
		attributes.diffuseCoef = diffuseCoef;
		attributes.specularCoef = specularCoef;
		attributes.specularPower = specularPower;
		attributes.stepScale = stepScale;

		if (materialIndex == 0)
			m_planeMaterialCB = attributes;
		else
			m_materialCB[materialIndex - 1] = attributes;
	};

	void SetPBRAttributes(
		UINT materialIndex,
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
		const XMFLOAT3& extinction = XMFLOAT3(1.0f, 1.0f, 1.0f),
		float subsurface = 0.0f,
		const XMFLOAT4& emission = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f)
	)
	{
		PBRPrimitiveConstantBuffer attributes;
		attributes.materialIndex = materialIndex;
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
		attributes.subsurface = subsurface;
		attributes.emission = emission;

		if (materialIndex == 0)
			m_pbrPlaneMaterialCB = attributes;
		else
			m_pbrMaterialCB[materialIndex - 1] = attributes;
	};

public:
	SceneTypes::SceneType m_sceneType{};

	XMVECTOR m_eye{};
	XMVECTOR m_at{};

	std::vector<LightBuffer> m_lights{};

	// Geometry
	std::vector<UINT> m_primitiveCount{ IntersectionShaderType::Count, 0 };

	// Root constants
	PrimitiveConstantBuffer m_planeMaterialCB{};
	PBRPrimitiveConstantBuffer m_pbrPlaneMaterialCB{};
	std::vector<PrimitiveConstantBuffer> m_materialCB{};
	std::vector<PBRPrimitiveConstantBuffer> m_pbrMaterialCB{};
};
