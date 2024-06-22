#include "stdafx.h"
#include "Scene.h"

void Scene::Load(std::vector<LightBuffer> lights, PrimitiveConstantBuffer planeMaterialCB, std::vector<PrimitiveConstantBuffer> aabbMaterialCB, std::vector<PBRPrimitiveConstantBuffer> pbrAabbMaterialCB, std::vector<PBRPrimitiveConstantBuffer> pbrMaterialCB)
{
	m_lights = lights;
	m_planeMaterialCB = planeMaterialCB;
	m_aabbMaterialCB = aabbMaterialCB;
	m_pbrAabbMaterialCB = pbrAabbMaterialCB;
	m_pbrMaterialCB = pbrMaterialCB;
}
