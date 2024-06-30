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

#pragma once

#include "DXSample.h"
#include "StepTimer.h"
#include "RaytracingSceneDefines.h"
#include "DirectXRaytracingHelper.h"
#include "PerformanceTimers.h"
#include "Scene.h"

class DieXaR : public DXSample
{
public:
	DieXaR(UINT width, UINT height, std::wstring name);

	// Resets the camera location
	virtual void ResetCamera(XMVECTOR eye, XMVECTOR at);

	// Reloads D3D resources and reinitializes them (also rebuilds the scene)
	// This should be called from outside the application loop
	virtual void Reload() override;

	// Resets the settings stored in the CB to the ones stored in the application settings
	// Already called in Reload() so no need to call it again after Reload()
	virtual void ResetSettingsCB();

	// Advances the frame index for temporal path tracing
	virtual void AdvancePathTracing();

	// Resets the frame cache for temporal path tracing
	virtual void ResetPathTracing();

	// IDeviceNotify
	virtual void OnDeviceLost() override;
	virtual void OnDeviceRestored() override;

	// Messages
	virtual void OnInit();
	virtual void OnKeyDown(UINT8 key);
	virtual void OnKeyUp(UINT8 key);
	virtual void OnMouseMove(UINT x, UINT y);
	virtual void OnUpdate();
	virtual void OnRender();
	virtual void OnSizeChanged(UINT width, UINT height, bool minimized);
	virtual void OnDestroy();
	virtual IDXGISwapChain* GetSwapchain() { return m_deviceResources->GetSwapChain(); }

	bool ShouldReload() const { return m_shouldReload; }
private:
	static const UINT FrameCount = 3;

	// Constants.
	const UINT NUM_BLAS = 2;          // Triangle + AABB bottom-level AS.
	const UINT NUM_SCENES = 3;		  // Number of scenes available.

	// DirectX Raytracing (DXR) attributes
	ComPtr<ID3D12Device5> m_dxrDevice;
	ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList;
	ComPtr<ID3D12StateObject> m_dxrStateObject;

	// Root signatures
	ComPtr<ID3D12RootSignature> m_raytracingGlobalRootSignature;
	ComPtr<ID3D12RootSignature> m_raytracingLocalRootSignature[LocalRootSignature::Type::Count];

	// Descriptors
	ComPtr<ID3D12DescriptorHeap> m_descriptorHeap;
	UINT m_descriptorsAllocated;
	UINT m_descriptorSize;

	// Raytracing scene
	ConstantBuffer<SceneConstantBuffer> m_sceneCB;
	StructuredBuffer<LightBuffer> m_lights;
	StructuredBuffer<PrimitiveInstancePerFrameBuffer> m_aabbPrimitiveAttributeBuffer;
	std::vector<D3D12_RAYTRACING_AABB> m_aabbs;

	// Scene description
	UINT m_crtScene{ SceneTypes::Demo };
	Scene m_scenes[SceneTypes::Count];

	// Geometry
	D3DBuffer m_indexBuffer;
	D3DBuffer m_vertexBuffer;
	D3DBuffer m_aabbBuffer;

	// Acceleration structure
	ComPtr<ID3D12Resource> m_bottomLevelAS[BottomLevelASType::Count];
	ComPtr<ID3D12Resource> m_topLevelAS;

	// Raytracing output
	ComPtr<ID3D12Resource> m_raytracingOutput;
	D3D12_GPU_DESCRIPTOR_HANDLE m_raytracingOutputResourceUAVGpuDescriptor;
	UINT m_raytracingOutputResourceUAVDescriptorHeapIndex;

	// Shader tables
	static const wchar_t* c_hitGroupNames_TriangleGeometry[RayType::Count];
	static const wchar_t* c_hitGroupNames_AABBGeometry[IntersectionShaderType::Count][RayType::Count];
	static const wchar_t* c_raygenShaderNames[RaytracingType::Count];
	static const wchar_t* c_intersectionShaderNames[IntersectionShaderType::Count];
	static const wchar_t* c_closestHitShaderNames[GeometryType::Count];
	static const wchar_t* c_missShaderNames[RayType::Count];

	ComPtr<ID3D12Resource> m_missShaderTable;
	UINT m_missShaderTableStrideInBytes;
	ComPtr<ID3D12Resource> m_hitGroupShaderTable;
	UINT m_hitGroupShaderTableStrideInBytes;
	ComPtr<ID3D12Resource> m_rayGenShaderTable;

	// Application state
	DX::GPUTimer m_gpuTimers[GpuTimers::Count];
	StepTimer m_timer;
	float m_animateGeometryTime;
	bool m_animateGeometry;
	bool m_cameraFly{};	// if false then rotates around the middle of the scene
	bool m_cameraMovingForward{};
	bool m_cameraMovingBackward{};
	bool m_cameraMovingLeft{};
	bool m_cameraMovingRight{};
	bool m_cameraMovingUp{};
	bool m_cameraMovingDown{};
	bool m_cameraMovingSlow{};
	bool m_cameraLocked{};
	float m_cameraSpeed{};
	const float m_cameraBaseRotateSpeed{ 1.0f };
	const float m_cameraBaseMoveSpeed{ 20.0f };
	XMVECTOR m_eye;
	XMVECTOR m_at;
	XMVECTOR m_up;

	// Application settings
	// Most of these have correspondents in the scene constant buffer
	RaytracingType::Enum m_raytracingType{ RaytracingType::PathTracing };
	ImportanceSamplingType::Enum m_importanceSamplingType{ ImportanceSamplingType::BSDF };
	bool m_applyJitter{ true };
	UINT m_maxRecursionDepth{ 3 };
	UINT m_maxShadowRecursionDepth{ 3 };	// one shadow pass in first reflection/refraction
	UINT m_pathSqrtSamplesPerPixel{ 1 };	// CAUTION: increasing this value will increase the number of rays per pixel exponentially
	UINT m_pathFrameCacheIndex{ 1 };		// current frame index for temporal path tracing (ALWAYS >= 1)
	bool m_onlyOneLightSample{ true };		// if true, only one light is sampled at a time
	UINT m_russianRouletteDepth{ 3 };		// the depth at which Russian roulette is applied
	bool m_anisotropicBSDF{ true };			// if true, anisotropic BSDF is used
	XMFLOAT4 m_backgroundColor{ 0.678f, 0.788f, 0.819f, 1.0f };

	void UpdateCameraMatrices();
	void UpdateAABBPrimitiveAttributes(float animationTime);
	void UpdateAABBPrimitiveAttributesDemo(float animationTime);
	void UpdateAABBPrimitiveAttributesCornellBox(float animationTime);
	void UpdateAABBPrimitiveAttributesPbrShowcase(float animationTime);
	void InitializeScene();
	void InitializeCornellBox();
	void InitializeDemo();
	void InitializePbrShowcase();
	void RecreateD3D();
	void DoRaytracing();
	void ShowUI();
	void HelpMarker(const char* desc);
	void CreateConstantBuffers();
	void CreateLightBuffer();
	void CreateAABBPrimitiveAttributesBuffers();
	void CreateDeviceDependentResources();
	void CreateWindowSizeDependentResources();
	void ReleaseDeviceDependentResources();
	void ReleaseWindowSizeDependentResources();
	void CreateRaytracingInterfaces();
	void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig);
	void CreateRootSignatures();
	void CreateDxilLibrarySubobject(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
	void CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
	void CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
	void CreateRaytracingPipelineStateObject();
	void CreateAuxiliaryDeviceResources();
	void CreateDescriptorHeap();
	void CreateRaytracingOutputResource();
	void BuildProceduralGeometryAABBs();
	void BuildProceduralGeometryAABBsDemo();
	void BuildProceduralGeometryAABBsCornellBox();
	void BuildProceduralGeometryAABBsPbrShowcase();
	void BuildGeometry();
	void BuildPlaneGeometry();
	void BuildGeometryDescsForBottomLevelAS(std::array<std::vector<D3D12_RAYTRACING_GEOMETRY_DESC>, BottomLevelASType::Count>& geometryDescs);
	template <class InstanceDescType, class BLASPtrType>
	void BuildBotomLevelASInstanceDescs(BLASPtrType* bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource);
	template <class InstanceDescType, class BLASPtrType>
	void BuildBotomLevelASInstanceDescsDemo(BLASPtrType* bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource);
	template <class InstanceDescType, class BLASPtrType>
	void BuildBotomLevelASInstanceDescsCornellBox(BLASPtrType* bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource);
	template <class InstanceDescType, class BLASPtrType>
	void BuildBotomLevelASInstanceDescsPbrShowcase(BLASPtrType* bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource);
	AccelerationStructureBuffers BuildBottomLevelAS(const std::vector<D3D12_RAYTRACING_GEOMETRY_DESC>& geometryDesc, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);
	AccelerationStructureBuffers BuildTopLevelAS(AccelerationStructureBuffers bottomLevelAS[BottomLevelASType::Count], D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);
	void BuildAccelerationStructures();
	void BuildShaderTables();
	void UpdateForSizeChange(UINT clientWidth, UINT clientHeight);
	void CopyRaytracingOutputToBackbuffer();
	void CalculateFrameStats();
	UINT AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse = UINT_MAX);
	UINT CreateBufferSRV(D3DBuffer* buffer, UINT numElements, UINT elementSize);
};
