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

#include "stdafx.h"
#include "DieXaR.h"
#include "CompiledShaders\Raytracing.hlsl.h"
#include "CompiledShaders\MotionVector.hlsl.h"
#include "CompiledShaders\VisualizeMotionVectors.hlsl.h"

// imgui
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"

#include <random>

using namespace std;
using namespace DX;

// From: http://blog.selfshadow.com/publications/s2015-shading-course/hoffman/s2015_pbs_physics_math_slides.pdf
static const XMFLOAT4 ChromiumReflectance = XMFLOAT4(0.549f, 0.556f, 0.554f, 1.0f);
static const XMFLOAT4 white               = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
static const XMFLOAT4 green               = XMFLOAT4(0.1f, 0.6f, 0.41f, 1.0f);
static const XMFLOAT4 red                 = XMFLOAT4(0.6f, 0.2f, 0.23f, 1.0f);
static const XMFLOAT4 yellow              = XMFLOAT4(0.7f, 0.6f, 0.2f, 1.0f);
static const XMFLOAT4 violet              = XMFLOAT4(0.5f, 0.5f, 1.0f, 1.0f);
static const XMFLOAT4 orange              = XMFLOAT4(0.7f, 0.2f, 0.0f, 1.0f);
static const XMFLOAT4 ruby                = XMFLOAT4(0.6f, 0.0f, 0.0f, 1.0f);
static const XMFLOAT4 silver              = XMFLOAT4(0.972f, 0.960f, 0.915f, 1.0f);
static const XMFLOAT4 gold                = XMFLOAT4(1.022f, 0.782f, 0.344f, 1.0f);
static const XMFLOAT4 copper              = XMFLOAT4(0.955f, 0.638f, 0.538f, 1.0f);

// Shader entry points.
const wchar_t* DieXaR::c_raygenShaderNames[] = {
    L"RaygenShader",
    L"RaygenShader_PathTracing",
    L"RaygenShader_PathTracingTemporal",
};
const wchar_t* DieXaR::c_intersectionShaderNames[] = {
    L"IntersectionShader_AnalyticPrimitive",
    L"IntersectionShader_SignedDistancePrimitive",
};
const wchar_t* DieXaR::c_closestHitShaderNames[] = {
    L"ClosestHitShader_Triangle",
    L"ClosestHitShader_AABB",
};
const wchar_t* DieXaR::c_missShaderNames[] = { L"MissShader", L"MissShader_ShadowRay" };
// Hit groups.
const wchar_t* DieXaR::c_hitGroupNames_TriangleGeometry[] = { L"HitGroup_Triangle", L"HitGroup_Triangle_ShadowRay" };
const wchar_t* DieXaR::c_hitGroupNames_AABBGeometry[][RayType::Count] = {
    { L"HitGroup_AABB_AnalyticPrimitive", L"HitGroup_AABB_AnalyticPrimitive_ShadowRay" },
    { L"HitGroup_AABB_SignedDistancePrimitive", L"HitGroup_AABB_SignedDistancePrimitive_ShadowRay" },
};

DieXaR::DieXaR(UINT width, UINT height, std::wstring name)
    : DXSample(width, height, name),
      m_raytracingOutputResourceUAVDescriptorHeapIndex(UINT_MAX),
      m_animateGeometryTime(0.0f),
      m_animateGeometry(true),
      m_descriptorsAllocated(0),
      m_descriptorSize(0),
      m_missShaderTableStrideInBytes(UINT_MAX),
      m_hitGroupShaderTableStrideInBytes(UINT_MAX)
{}

void DieXaR::ResetCamera(XMVECTOR eye, XMVECTOR at)
{
    // Initialize the view and projection inverse matrices.
    m_eye            = eye;
    m_at             = at;
    XMVECTOR forward = XMVector4Normalize(at - eye);

    // Compute up and right vectors.
    XMVECTOR right = XMVector3Normalize(XMVector3Cross(forward, XMVectorSet(0, 1, 0, 0)));
    m_up           = XMVector3Normalize(XMVector3Cross(right, forward));

    // Reset parameters
    m_cameraMovingLeft = m_cameraMovingRight = m_cameraMovingForward = m_cameraMovingBackward = m_cameraMovingUp
            = m_cameraMovingDown = m_cameraMovingSlow = false;
    m_cameraSpeed                                     = 0.0f;

    UpdateCameraMatrices();
}

void DieXaR::Reload()
{
    m_resetUpscale = true;
    OnDestroy();
    OnInit();
    m_shouldReload = false;
}

void DieXaR::ResetSettingsCB()
{
    m_sceneCB->numLights                 = m_scenes[m_crtScene].GetLightCount();
    m_sceneCB->raytracingType            = m_raytracingType;
    m_sceneCB->importanceSamplingType    = m_importanceSamplingType;
    m_sceneCB->maxRecursionDepth         = m_maxRecursionDepth;
    m_sceneCB->maxShadowRecursionDepth   = m_maxShadowRecursionDepth;
    m_sceneCB->pathSqrtSamplesPerPixel   = m_pathSqrtSamplesPerPixel;
    m_sceneCB->applyJitter               = m_applyJitter;
    m_sceneCB->onlyOneLightSample        = m_onlyOneLightSample;
    m_sceneCB->russianRouletteDepth      = m_russianRouletteDepth;
    m_sceneCB->anisotropicBSDF           = m_anisotropicBSDF;
    m_sceneCB->backgroundColor           = m_backgroundColor;
    m_sceneCB->sceneIndex                = m_crtScene;
    m_sceneCB->previousWorldToProjection = m_sceneCB->worldToProjection;
}

void DieXaR::AdvancePathTracing()
{
    m_sceneCB->pathFrameCacheIndex = ++m_pathFrameCacheIndex;
}

void DieXaR::ResetPathTracing()
{
    // Reset the frame index.
    m_sceneCB->pathFrameCacheIndex = m_pathFrameCacheIndex = 1;
}

void DieXaR::OnInit()
{
    UpdateForSizeChange(m_width, m_height);

    m_deviceResources = std::make_unique<DeviceResources>(DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_UNKNOWN, FrameCount,
            D3D_FEATURE_LEVEL_11_0,
            // Sample shows handling of use cases with tearing support, which is OS dependent and has been
            // supported since TH2. Since the sample requires build 1809 (RS5) or higher, we don't need to
            // handle non-tearing cases.
            DeviceResources::c_RequireTearingSupport, m_adapterIDoverride);
    m_deviceResources->RegisterDeviceNotify(this);
    m_deviceResources->SetWindow(Win32Application::GetHwnd(), m_width, m_height);
    m_deviceResources->InitializeDXGIAdapter();

    ThrowIfFalse(IsDirectXRaytracingSupported(m_deviceResources->GetAdapter()),
            L"ERROR: DirectX Raytracing is not supported by your OS, GPU and/or driver.\n\n");

    m_deviceResources->CreateDeviceResources();
    m_deviceResources->CreateWindowSizeDependentResources();

    // Only initialize the scene if we are changing it.
    if (!m_shouldReload || m_crtScene != m_nextScene) {
        m_crtScene = m_nextScene;
        InitializeScene();
    }

    // Reset Path Tracing.
    ResetPathTracing();

    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();

    // FSR3 Initialization
    InitializeFSR3();

    // Initialize imgui.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui_ImplWin32_Init(Win32Application::GetHwnd());
    D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle;
    UINT                        descriptorHeapIndex = AllocateDescriptor(&cpuHandle);

    ImGui_ImplDX12_Shutdown();
    ImGui_ImplDX12_Init(m_deviceResources->GetD3DDevice(), FrameCount, DXGI_FORMAT_R8G8B8A8_UNORM,
            m_descriptorHeap.Get(), cpuHandle,
            CD3DX12_GPU_DESCRIPTOR_HANDLE(
                    m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), descriptorHeapIndex, m_descriptorSize));

    ImGui::StyleColorsDark();

    // Initialize application settings.
    ResetSettingsCB();

    // Make window not resizable
    HWND     hwnd  = Win32Application::GetHwnd();
    LONG_PTR style = GetWindowLongPtr(hwnd, GWL_STYLE);
    style &= ~WS_THICKFRAME;
    // style &= ~WS_MAXIMIZEBOX;
    SetWindowLongPtr(hwnd, GWL_STYLE, style);
}

// Update camera matrices passed into the shader.
void DieXaR::UpdateCameraMatrices()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    m_sceneCB->cameraPosition = m_eye;
    float    fovAngleY        = 45.0f;
    XMMATRIX view             = XMMatrixLookAtLH(m_eye, m_at, m_up);
    XMMATRIX proj             = XMMatrixPerspectiveFovLH(XMConvertToRadians(fovAngleY), m_aspectRatio, 0.01f, 125.0f);

    // FSR: Apply Jitter
    if (m_upscalingContext) {
        int32_t                                  jitterPhaseCount   = 0;
        ffx::QueryDescUpscaleGetJitterPhaseCount getJitterPhaseDesc = {};
        getJitterPhaseDesc.renderWidth                              = m_renderWidth;
        getJitterPhaseDesc.displayWidth                             = m_width;
        getJitterPhaseDesc.pOutPhaseCount                           = &jitterPhaseCount;
        ffx::Query(m_upscalingContext, getJitterPhaseDesc);

        m_jitterIndex = (m_jitterIndex + 1) % jitterPhaseCount;

        ffx::QueryDescUpscaleGetJitterOffset getJitterOffsetDesc = {};
        getJitterOffsetDesc.index                                = m_jitterIndex;
        getJitterOffsetDesc.phaseCount                           = jitterPhaseCount;
        getJitterOffsetDesc.pOutX                                = &m_jitterX;
        getJitterOffsetDesc.pOutY                                = &m_jitterY;
        ffx::Query(m_upscalingContext, getJitterOffsetDesc);

        // Apply jitter to the projection matrix
        XMMATRIX jitterMatrix
                = XMMatrixTranslation(2.0f * m_jitterX / m_renderWidth, -2.0f * m_jitterY / m_renderHeight, 0.0f);
        proj = proj * jitterMatrix;
    }


    XMMATRIX viewProj            = view * proj;
    m_sceneCB->projectionToWorld = XMMatrixInverse(nullptr, viewProj);
    m_sceneCB->worldToProjection = viewProj;
}

// Update AABB primitive attributes buffers passed into the shader.
void DieXaR::UpdateAABBPrimitiveAttributes(float animationTime)
{
    switch (m_crtScene) {
        case SceneTypes::CornellBox:
            UpdateAABBPrimitiveAttributesCornellBox(animationTime);
            break;
        case SceneTypes::Demo:
            UpdateAABBPrimitiveAttributesDemo(animationTime);
            break;
        case SceneTypes::PbrShowcase:
            UpdateAABBPrimitiveAttributesPbrShowcase(animationTime);
            break;
    }
}

void DieXaR::UpdateAABBPrimitiveAttributesDemo(float animationTime)
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    XMMATRIX mIdentity = XMMatrixIdentity();

    XMMATRIX mScale15y = XMMatrixScaling(1, 1.5, 1);
    XMMATRIX mScale15  = XMMatrixScaling(1.5, 1.5, 1.5);
    XMMATRIX mScale2   = XMMatrixScaling(2, 2, 2);
    XMMATRIX mScale3   = XMMatrixScaling(5, 5, 5);

    XMMATRIX mRotation = XMMatrixRotationY(-2 * animationTime);

    // Apply scale, rotation and translation transforms.
    // The intersection shader tests in this sample work with local space, so here
    // we apply the BLAS object space translation that was passed to geometry descs.
    auto SetTransformForAABB = [&](UINT primitiveIndex, XMMATRIX& mScale, XMMATRIX& mRotation) {
        XMVECTOR vTranslation = 0.5f
                                * (XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MinX))
                                        + XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MaxX)));
        XMMATRIX mTranslation = XMMatrixTranslationFromVector(vTranslation);

        XMMATRIX mTransform                                                      = mScale * mRotation * mTranslation;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].localSpaceToBottomLevelAS = mTransform;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].bottomLevelASToLocalSpace = XMMatrixInverse(nullptr, mTransform);
    };

    UINT offset = 0;
    // Analytic primitives.
    {
        using namespace AnalyticPrimitive;
        SetTransformForAABB(offset + AABB, mScale15y, mIdentity);
        SetTransformForAABB(offset + Spheres, mScale15, mIdentity);
        offset += AnalyticPrimitive::Count;
    }

    // Signed distance primitives.
    {
        using namespace SignedDistancePrimitive;

        // SetTransformForAABB(offset + MiniSpheres, mIdentity, mIdentity);
        SetTransformForAABB(offset + IntersectedRoundCube, mIdentity, mIdentity);
        SetTransformForAABB(offset + SquareTorus, mScale15, mIdentity);
        SetTransformForAABB(offset + Cog, mIdentity, mRotation);
        SetTransformForAABB(offset + Cylinder, mScale15y, mIdentity);
        SetTransformForAABB(offset + SolidAngle, mScale3, mIdentity);
    }
}

void DieXaR::UpdateAABBPrimitiveAttributesCornellBox(float animationTime)
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    XMMATRIX mIdentity = XMMatrixIdentity();
    XMMATRIX mScale15y = XMMatrixScaling(0.9, 1.5, 0.9);
    XMMATRIX mRotation = XMMatrixRotationY(-2.0f);

    // Apply scale, rotation and translation transforms.
    // The intersection shader tests in this sample work with local space, so here
    // we apply the BLAS object space translation that was passed to geometry descs.
    auto SetTransformForAABB = [&](UINT primitiveIndex, XMMATRIX& mScale, XMMATRIX& mRotation) {
        XMVECTOR vTranslation = 0.5f
                                * (XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MinX))
                                        + XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MaxX)));
        XMMATRIX mTranslation = XMMatrixTranslationFromVector(vTranslation);

        XMMATRIX mTransform                                                      = mScale * mRotation * mTranslation;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].localSpaceToBottomLevelAS = mTransform;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].bottomLevelASToLocalSpace = XMMatrixInverse(nullptr, mTransform);
    };

    using namespace AnalyticPrimitive;
    SetTransformForAABB(AABB, mScale15y, mRotation);
    SetTransformForAABB(Spheres, mIdentity, mIdentity);
}

void DieXaR::UpdateAABBPrimitiveAttributesPbrShowcase(float animationTime)
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    XMMATRIX mIdentity = XMMatrixIdentity();

    // Apply scale, rotation and translation transforms.
    // The intersection shader tests in this sample work with local space, so here
    // we apply the BLAS object space translation that was passed to geometry descs.
    auto SetTransformForAABB = [&](UINT primitiveIndex, XMMATRIX& mScale, XMMATRIX& mRotation) {
        XMVECTOR vTranslation = 0.5f
                                * (XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MinX))
                                        + XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MaxX)));
        XMMATRIX mTranslation = XMMatrixTranslationFromVector(vTranslation);

        XMMATRIX mTransform                                                      = mScale * mRotation * mTranslation;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].localSpaceToBottomLevelAS = mTransform;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].bottomLevelASToLocalSpace = XMMatrixInverse(nullptr, mTransform);
    };

    for (UINT i = 0; i < m_scenes[m_crtScene].GetPrimitiveCount(); ++i)
        SetTransformForAABB(i, mIdentity * 0.9f, mIdentity);
}

// Initialize scene rendering parameters.
void DieXaR::InitializeScene()
{
    switch (m_crtScene) {
        case SceneTypes::CornellBox:
            InitializeCornellBox();
            break;
        case SceneTypes::Demo:
            InitializeDemo();
            break;
        case SceneTypes::PbrShowcase:
            InitializePbrShowcase();
            break;
    }
}

void DieXaR::InitializeCornellBox()
{
    m_scenes[SceneTypes::CornellBox].m_sceneType = SceneTypes::CornellBox;

    // Setup Camera
    m_scenes[m_crtScene].m_eye = { 0.0f, 3.5f, 8.0f, 0.0f };
    m_scenes[m_crtScene].m_at  = { 0.0f, 2.5f, 0.0f, 0.0f };
    ResetCamera(m_scenes[m_crtScene].m_eye, m_scenes[m_crtScene].m_at);

    // Setup Materials
    {
        // Setup plane
        {
            m_scenes[m_crtScene].SetAttributes(0, XMFLOAT4(0.9f, 0.9f, 0.9f, 1.0f), 0.25f, 1, 0.7f, 50, 1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    0, XMFLOAT4(0.9f, 0.9f, 0.9f, 1.0f), 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f);
        }

        // Setup walls
        UINT offset                                                                      = 1;
        m_scenes[m_crtScene].m_primitiveCount[IntersectionShaderType::AnalyticPrimitive] = AnalyticPrimitive::Count;
        m_scenes[m_crtScene].m_primitiveCount[IntersectionShaderType::SignedDistancePrimitive] = 0;
        UINT totalPrimitiveCount = m_scenes[m_crtScene].GetPrimitiveCount();
        m_scenes[m_crtScene].m_materialCB.resize(totalPrimitiveCount + 4);
        m_scenes[m_crtScene].m_pbrMaterialCB.resize(totalPrimitiveCount + 4);
        {
            m_scenes[m_crtScene].SetAttributes(offset, white, 0.0f, 1, 0.0f, 50, 1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset++, white, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f);
            m_scenes[m_crtScene].SetAttributes(offset, white, 0.0f, 1, 0.0f, 50, 1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset++, white, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f);
            m_scenes[m_crtScene].SetAttributes(offset, green, 0.0f, 1, 0.0f, 50, 1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset++, green, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f);
            m_scenes[m_crtScene].SetAttributes(offset, red, 0.0f, 1, 0.0f, 50, 1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset++, red, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f);
        }

        // Setup analytic primitives
        {
            using namespace AnalyticPrimitive;
            m_scenes[m_crtScene].SetAttributes(offset + AABB, white, 0.3f, 0.8f, 0.6f);
            m_scenes[m_crtScene].SetPBRAttributes(offset + AABB, white, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.7f, 1.0f, XMFLOAT3(1.0f, 0.9f, 1.0f), 1.0f);
            m_scenes[m_crtScene].SetAttributes(offset + Spheres, white, 1.0f);
            m_scenes[m_crtScene].SetPBRAttributes(offset + Spheres, ChromiumReflectance, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 1.5f, 1.0f, XMFLOAT3(0.7f, 1.0f, 1.0f), 0.0f);
        }
    }

    // Setup Lights
    m_backgroundColor = XMFLOAT4(0.00f, 0.00f, 0.00f, 1.0f);
    m_scenes[m_crtScene].m_lights.resize(1);
    Scene::SetLight(m_scenes[m_crtScene].m_lights[0], XMFLOAT3(0.0f, 4.799f, -1.5f), XMFLOAT3(1.0f, 1.0f, 0.7f), 5.0f,
            LightType::Square, 1.825f);
}

void DieXaR::InitializeDemo()
{
    m_scenes[SceneTypes::Demo].m_sceneType = SceneTypes::Demo;

    // Setup Camera
    m_scenes[m_crtScene].m_eye = { 10.63f, 5.21f, 4.13f, 0.0f };
    m_scenes[m_crtScene].m_at  = { -0.89f, -0.24f, -0.38f, 0.0f };
    m_scenes[m_crtScene].m_at += m_scenes[m_crtScene].m_eye;
    ResetCamera(m_scenes[m_crtScene].m_eye, m_scenes[m_crtScene].m_at);

    // Setup Materials
    {
        // Setup plane
        {
            m_scenes[m_crtScene].SetAttributes(0, XMFLOAT4(0.9f, 0.9f, 0.9f, 1.0f), 0.25f, 1, 0.7f, 50, 1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    0, XMFLOAT4(0.9f, 0.9f, 0.9f, 1.0f), 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f);
        }

        UINT offset                                                                      = 0;
        m_scenes[m_crtScene].m_primitiveCount[IntersectionShaderType::AnalyticPrimitive] = AnalyticPrimitive::Count;
        m_scenes[m_crtScene].m_primitiveCount[IntersectionShaderType::SignedDistancePrimitive]
                = SignedDistancePrimitive::Count;
        UINT totalPrimitiveCount = m_scenes[m_crtScene].GetPrimitiveCount();
        m_scenes[m_crtScene].m_materialCB.resize(totalPrimitiveCount);
        m_scenes[m_crtScene].m_pbrMaterialCB.resize(totalPrimitiveCount);

        // Setup analytic primitives
        {
            using namespace AnalyticPrimitive;
            m_scenes[m_crtScene].SetAttributes(offset + AABB + 1, orange, 0.3f, 0.8f, 0.6f);
            m_scenes[m_crtScene].SetPBRAttributes(offset + AABB + 1, orange, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.7f, 1.7f, 1.0f, XMFLOAT3(1.0f, 0.9f, 1.0f), 1.0f);
            m_scenes[m_crtScene].SetAttributes(offset + Spheres + 1, red, 1.0f);
            m_scenes[m_crtScene].SetPBRAttributes(offset + Spheres + 1, red, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 1.5f, 1.0f, XMFLOAT3(0.7f, 1.0f, 1.0f), 0.0f);
            offset += AnalyticPrimitive::Count;
        }

        // Setup signed distance primitives
        {
            using namespace SignedDistancePrimitive;
            m_scenes[m_crtScene].SetAttributes(offset + IntersectedRoundCube + 1, green);
            m_scenes[m_crtScene].SetPBRAttributes(offset + IntersectedRoundCube + 1, green, 0.8f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f);
            m_scenes[m_crtScene].SetAttributes(offset + SquareTorus + 1, violet, 1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset + SquareTorus + 1, violet, 0.1f, 0.0f, 0.2f, 0.0f, 0.0f, 0.7f, 0.0f, 0.0f, 0.0f, 1.5f);
            m_scenes[m_crtScene].SetAttributes(offset + Cog + 1, yellow, 0.1f);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset + Cog + 1, yellow, 0.9f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.51f);
            m_scenes[m_crtScene].SetAttributes(offset + Cylinder + 1, silver, 1.0f);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset + Cylinder + 1, silver, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.15f);
            m_scenes[m_crtScene].SetAttributes(offset + SolidAngle + 1, copper, 0.1);
            m_scenes[m_crtScene].SetPBRAttributes(
                    offset + SolidAngle + 1, copper, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.7f, 1.0f, 0.0f, 0.0f, 1.1f);
        }
    }

    // Setup Lights
    m_backgroundColor = XMFLOAT4(0.00f, 0.00f, 0.00f, 1.0f);
    m_scenes[m_crtScene].m_lights.resize(2);
    Scene::SetLight(m_scenes[m_crtScene].m_lights[0], XMFLOAT3(-0.8f, 3.882f, 0.393f), XMFLOAT3(0.474f, 0.376f, 0.75f),
            5.885f, LightType::Square, 1.825f, XMFLOAT3(0.76f, -0.196f, 0.596f));
    Scene::SetLight(m_scenes[m_crtScene].m_lights[1], XMFLOAT3(2.4f, 11.368f, -1.275f), XMFLOAT3(0.78f, 0.815f, 0.65f),
            4.792f, LightType::Square, 5.468f, XMFLOAT3(0.76f, -0.196f, 0.596f));
}

void DieXaR::InitializePbrShowcase()
{
    m_scenes[SceneTypes::PbrShowcase].m_sceneType = SceneTypes::PbrShowcase;

    // Setup Camera
    m_scenes[m_crtScene].m_eye = { -15.63f, 5.21f, 0.0f, 0.0f };
    m_scenes[m_crtScene].m_at  = { 0.0f, 0.0f, 0.0f, 0.0f };
    ResetCamera(m_scenes[m_crtScene].m_eye, m_scenes[m_crtScene].m_at);

    // Setup materials
    {
        std::mt19937                          rng(67);
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);

        UINT numSpheres = 40;

        // Set base material
        PrimitiveConstantBuffer baseMaterial;
        baseMaterial.materialIndex   = 0;
        baseMaterial.stepScale       = 1.0f;
        baseMaterial.albedo          = silver;
        baseMaterial.diffuseCoef     = 0.5f;
        baseMaterial.reflectanceCoef = 0.5f;
        baseMaterial.specularCoef    = 0.0f;
        baseMaterial.specularPower   = 50.0f;

        PBRPrimitiveConstantBuffer pbrBaseMaterial;
        pbrBaseMaterial.materialIndex        = 0;
        pbrBaseMaterial.stepScale            = 1.0f;
        pbrBaseMaterial.materialIndex        = 0;
        pbrBaseMaterial.albedo               = silver;
        pbrBaseMaterial.anisotropic          = 0.0f;
        pbrBaseMaterial.atDistance           = 1.0f;
        pbrBaseMaterial.extinction           = XMFLOAT3(1.0f, 1.0f, 1.0f);
        pbrBaseMaterial.eta                  = 1.5f;
        pbrBaseMaterial.metallic             = 0.0f;
        pbrBaseMaterial.roughness            = 0.0f;
        pbrBaseMaterial.sheen                = 0.0f;
        pbrBaseMaterial.sheenTint            = 0.0f;
        pbrBaseMaterial.specularTint         = 0.0f;
        pbrBaseMaterial.specularTransmission = 0.0f;
        pbrBaseMaterial.subsurface           = 0.0f;
        pbrBaseMaterial.clearcoat            = 0.0f;
        pbrBaseMaterial.clearcoatGloss       = 0.0f;
        pbrBaseMaterial.emission             = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);

        m_scenes[m_crtScene].m_primitiveCount[IntersectionShaderType::AnalyticPrimitive]       = numSpheres;
        m_scenes[m_crtScene].m_primitiveCount[IntersectionShaderType::SignedDistancePrimitive] = 0;
        UINT totalPrimitiveCount = m_scenes[m_crtScene].GetPrimitiveCount();
        m_scenes[m_crtScene].m_materialCB.resize(totalPrimitiveCount);
        m_scenes[m_crtScene].m_pbrMaterialCB.resize(totalPrimitiveCount);

        // Set material for plane
        m_scenes[m_crtScene].m_planeMaterialCB    = baseMaterial;
        m_scenes[m_crtScene].m_pbrPlaneMaterialCB = pbrBaseMaterial;

        // Set random material variations for spheres
        for (UINT i = 0; i < numSpheres; ++i) {
            XMFLOAT4 albedo = XMFLOAT4(dist(rng), dist(rng), dist(rng), 1.0f);

            PrimitiveConstantBuffer attributes   = baseMaterial;
            attributes.materialIndex             = i + 1;
            attributes.albedo                    = albedo;
            attributes.diffuseCoef               = dist(rng);
            attributes.reflectanceCoef           = dist(rng);
            attributes.specularCoef              = dist(rng);
            attributes.specularPower             = 10.0f + 100.0f * dist(rng);
            m_scenes[m_crtScene].m_materialCB[i] = attributes;

            PBRPrimitiveConstantBuffer pbrAttributes = pbrBaseMaterial;
            pbrAttributes.materialIndex              = i + 1;
            pbrAttributes.albedo                     = albedo;
            pbrAttributes.anisotropic                = dist(rng) > 0.5f ? 1.0f : 0.0f;
            pbrAttributes.metallic                   = dist(rng);
            pbrAttributes.roughness                  = dist(rng);
            pbrAttributes.sheen                      = dist(rng);
            pbrAttributes.sheenTint                  = dist(rng);
            pbrAttributes.specularTint               = dist(rng);
            pbrAttributes.specularTransmission       = dist(rng) > 0.5f ? 1.0f : 0.0f;
            pbrAttributes.subsurface                 = dist(rng);
            pbrAttributes.clearcoat                  = dist(rng);
            pbrAttributes.clearcoatGloss             = dist(rng);
            pbrAttributes.emission
                    = dist(rng) > 0.9f ? XMFLOAT4(dist(rng), dist(rng), dist(rng), 1.0f) : pbrBaseMaterial.emission;
            m_scenes[m_crtScene].m_pbrMaterialCB[i] = pbrAttributes;
        }
    }

    // Setup Lights (color will not be used in the shader)
    m_scenes[m_crtScene].m_lights.resize(1);
    Scene::SetLight(m_scenes[m_crtScene].m_lights[0], XMFLOAT3(0.0f, 18.0f, -20.0f), XMFLOAT3(0.8f, 0.8f, 0.65f), 1.0f,
            LightType::Directional, 1.0f, XMFLOAT3(0.76f, -0.196f, 0.596f));
}

// Create constant buffers.
void DieXaR::CreateConstantBuffers()
{
    auto device     = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();

    m_sceneCB.Create(device, frameCount, L"Scene Constant Buffer");
}

void DieXaR::CreateLightBuffer()
{
    auto device     = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();

    m_lights.Create(device, m_scenes[m_crtScene].GetLightCount(), frameCount, L"Light Buffer");
}

// Create AABB primitive attributes buffers.
void DieXaR::CreateAABBPrimitiveAttributesBuffers()
{
    auto device     = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();
    m_aabbPrimitiveAttributeBuffer.Create(
            device, m_scenes[m_crtScene].GetPrimitiveCount(), frameCount, L"AABB primitive attributes");
}

// Create resources that depend on the device.
void DieXaR::CreateDeviceDependentResources()
{
    CreateAuxiliaryDeviceResources();

    // Initialize raytracing pipeline.

    // Create raytracing interfaces: raytracing device and commandlist.
    CreateRaytracingInterfaces();

    // Create root signatures for the shaders.
    CreateRootSignatures();

    // Create Motion Vector PSO.
    CreateMotionVectorPSO();
    CreateVisualizeMV_PSO();

    // Create a raytracing pipeline state object which defines the binding of shaders, state and resources to be used
    // during raytracing.
    CreateRaytracingPipelineStateObject();

    // Create a heap for descriptors.
    CreateDescriptorHeap();

    // Build geometry to be used in the sample.
    BuildGeometry();

    // Build raytracing acceleration structures from the generated geometry.
    BuildAccelerationStructures();

    // Create constant buffers for the geometry and the scene.
    CreateConstantBuffers();

    // Create light buffer.
    CreateLightBuffer();

    // Create AABB primitive attribute buffers.
    CreateAABBPrimitiveAttributesBuffers();

    // Build shader tables, which define shaders and their local root arguments.
    BuildShaderTables();
}

void DieXaR::SerializeAndCreateRaytracingRootSignature(
        D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig)
{
    auto             device = m_deviceResources->GetD3DDevice();
    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> error;

    ThrowIfFailed(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &error),
            error ? static_cast<wchar_t*>(error->GetBufferPointer()) : nullptr);
    ThrowIfFailed(
            device->CreateRootSignature(1, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&(*rootSig))));
}

void DieXaR::CreateRootSignatures()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Global Root Signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[3];  // Perfomance TIP: Order from most frequent to least frequent.
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output texture
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 1);  // 2 static index and vertex buffers.
        ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 2);  // World Position UAV at register u2

        CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignature::Slot::Count];
        rootParameters[GlobalRootSignature::Slot::OutputView].InitAsDescriptorTable(1, &ranges[0]);
        rootParameters[GlobalRootSignature::Slot::AccelerationStructure].InitAsShaderResourceView(0);
        rootParameters[GlobalRootSignature::Slot::SceneConstant].InitAsConstantBufferView(0);
        rootParameters[GlobalRootSignature::Slot::WorldPositionOutput].InitAsDescriptorTable(1, &ranges[2]);
        rootParameters[GlobalRootSignature::Slot::LightBuffer].InitAsShaderResourceView(4);
        rootParameters[GlobalRootSignature::Slot::AABBattributeBuffer].InitAsShaderResourceView(3);
        rootParameters[GlobalRootSignature::Slot::VertexBuffers].InitAsDescriptorTable(1, &ranges[1]);
        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        SerializeAndCreateRaytracingRootSignature(globalRootSignatureDesc, &m_raytracingGlobalRootSignature);
    }

    // Local Root Signature
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    {
        // Triangle geometry
        {
            namespace RootSignatureSlots = LocalRootSignature::Triangle::Slot;
            CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            rootParameters[RootSignatureSlots::MaterialConstant].InitAsConstants(
                    SizeOfInUint32(PrimitiveConstantBuffer), 1);
            rootParameters[RootSignatureSlots::GeometryIndex].InitAsConstants(
                    SizeOfInUint32(PrimitiveInstanceConstantBuffer), 2);
            rootParameters[RootSignatureSlots::PbrConstant].InitAsConstants(
                    SizeOfInUint32(PBRPrimitiveConstantBuffer), 3);

            CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            SerializeAndCreateRaytracingRootSignature(
                    localRootSignatureDesc, &m_raytracingLocalRootSignature[LocalRootSignature::Type::Triangle]);
        }

        // AABB geometry
        {
            namespace RootSignatureSlots = LocalRootSignature::AABB::Slot;
            CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            rootParameters[RootSignatureSlots::MaterialConstant].InitAsConstants(
                    SizeOfInUint32(PrimitiveConstantBuffer), 1);
            rootParameters[RootSignatureSlots::GeometryIndex].InitAsConstants(
                    SizeOfInUint32(PrimitiveInstanceConstantBuffer), 2);
            rootParameters[RootSignatureSlots::PbrConstant].InitAsConstants(
                    SizeOfInUint32(PBRPrimitiveConstantBuffer), 3);

            CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            SerializeAndCreateRaytracingRootSignature(
                    localRootSignatureDesc, &m_raytracingLocalRootSignature[LocalRootSignature::Type::AABB]);
        }
    }
}

// Create raytracing device and command list.
void DieXaR::CreateRaytracingInterfaces()
{
    auto device      = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    ThrowIfFailed(device->QueryInterface(IID_PPV_ARGS(&m_dxrDevice)),
            L"Couldn't get DirectX Raytracing interface for the device.\n");
    ThrowIfFailed(commandList->QueryInterface(IID_PPV_ARGS(&m_dxrCommandList)),
            L"Couldn't get DirectX Raytracing interface for the command list.\n");
}

// DXIL library
// This contains the shaders and their entrypoints for the state object.
// Since shaders are not considered a subobject, they need to be passed in via DXIL library subobjects.
void DieXaR::CreateDxilLibrarySubobject(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    auto                  lib     = raytracingPipeline->CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
    D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE((void*) g_pRaytracing, ARRAYSIZE(g_pRaytracing));
    lib->SetDXILLibrary(&libdxil);
    // Use default shader exports for a DXIL library/collection subobject ~ surface all shaders.
}

// Hit groups
// A hit group specifies closest hit, any hit and intersection shaders
// to be executed when a ray intersects the geometry.
void DieXaR::CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    // Triangle geometry hit groups
    {
        for (UINT rayType = 0; rayType < RayType::Count; rayType++) {
            auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
            if (rayType == RayType::Radiance) {
                hitGroup->SetClosestHitShaderImport(c_closestHitShaderNames[GeometryType::Triangle]);
            }
            hitGroup->SetHitGroupExport(c_hitGroupNames_TriangleGeometry[rayType]);
            hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_TRIANGLES);
        }
    }

    // AABB geometry hit groups
    {
        // Create hit groups for each intersection shader.
        for (UINT t = 0; t < IntersectionShaderType::Count; t++)
            for (UINT rayType = 0; rayType < RayType::Count; rayType++) {
                auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
                hitGroup->SetIntersectionShaderImport(c_intersectionShaderNames[t]);
                if (rayType == RayType::Radiance) {
                    hitGroup->SetClosestHitShaderImport(c_closestHitShaderNames[GeometryType::AABB]);
                }
                hitGroup->SetHitGroupExport(c_hitGroupNames_AABBGeometry[t][rayType]);
                hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
            }
    }
}

// Local root signature and shader association
// This is a root signature that enables a shader to have unique arguments that come from shader tables.
void DieXaR::CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    // Ray gen and miss shaders in this sample are not using a local root signature and thus one is not associated with
    // them.

    // Hit groups
    // Triangle geometry
    {
        auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
        localRootSignature->SetRootSignature(m_raytracingLocalRootSignature[LocalRootSignature::Type::Triangle].Get());
        // Shader association
        auto rootSignatureAssociation
                = raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
        rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
        rootSignatureAssociation->AddExports(c_hitGroupNames_TriangleGeometry);
    }

    // AABB geometry
    {
        auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
        localRootSignature->SetRootSignature(m_raytracingLocalRootSignature[LocalRootSignature::Type::AABB].Get());
        // Shader association
        auto rootSignatureAssociation
                = raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
        rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
        for (auto& hitGroupsForIntersectionShaderType : c_hitGroupNames_AABBGeometry) {
            rootSignatureAssociation->AddExports(hitGroupsForIntersectionShaderType);
        }
    }
}

// Create a raytracing pipeline state object (RTPSO).
// An RTPSO represents a full set of shaders reachable by a DispatchRays() call,
// with all configuration options resolved, such as local signatures and other state.
void DieXaR::CreateRaytracingPipelineStateObject()
{
    // Create 18 subobjects that combine into a RTPSO:
    // Subobjects need to be associated with DXIL exports (i.e. shaders) either by way of default or explicit
    // associations. Default association applies to every exported shader entrypoint that doesn't have any of the same
    // type of subobject associated with it. This simple sample utilizes default shader association except for local
    // root signature subobject which has an explicit association specified purely for demonstration purposes. 1 - DXIL
    // library 8 - Hit group types - 4 geometries (1 triangle, 3 aabb) x 2 ray types (ray, shadowRay) 1 - Shader config
    // 6 - 3 x Local root signature and association
    // 1 - Global root signature
    // 1 - Pipeline config
    CD3DX12_STATE_OBJECT_DESC raytracingPipeline{ D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE };

    // DXIL library
    CreateDxilLibrarySubobject(&raytracingPipeline);

    // Hit groups
    CreateHitGroupSubobjects(&raytracingPipeline);

    // Shader config
    // Defines the maximum sizes in bytes for the ray rayPayload and attribute structure.
    auto shaderConfig  = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
    UINT payloadSize   = max(sizeof(RayPayload), sizeof(ShadowRayPayload));
    UINT attributeSize = sizeof(struct ProceduralPrimitiveAttributes);
    shaderConfig->Config(payloadSize, attributeSize);

    // Local root signature and shader association
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    CreateLocalRootSignatureSubobjects(&raytracingPipeline);

    // Global root signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    auto globalRootSignature = raytracingPipeline.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    globalRootSignature->SetRootSignature(m_raytracingGlobalRootSignature.Get());

    // Pipeline config
    // Defines the maximum TraceRay() recursion depth.
    auto pipelineConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
    // PERFOMANCE TIP: Set max recursion depth as low as needed
    // as drivers may apply optimization strategies for low recursion depths.
    // IMPORTANT: we set this to +1 because we want to allow for a shadow ray to be spawned always.
    pipelineConfig->Config(m_maxRecursionDepth + 1);

    PrintStateObjectDesc(raytracingPipeline);

    // Create the state object.
    ThrowIfFailed(m_dxrDevice->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&m_dxrStateObject)),
            L"Couldn't create DirectX Raytracing state object.\n");
}

void DieXaR::CreateAuxiliaryDeviceResources()
{
    auto device       = m_deviceResources->GetD3DDevice();
    auto commandQueue = m_deviceResources->GetCommandQueue();

    for (auto& gpuTimer : m_gpuTimers) {
        gpuTimer.RestoreDevice(device, commandQueue, FrameCount);
    }
}

void DieXaR::CreateDescriptorHeap()
{
    auto device = m_deviceResources->GetD3DDevice();

    D3D12_DESCRIPTOR_HEAP_DESC descriptorHeapDesc = {};
    // Allocate a heap for 4 descriptors:
    // 2 - vertex and index  buffer SRVs
    // 1 - raytracing output texture SRV
    // 1 - ImGui font texture SRV
    // 3 - worldpos, motionvec, upscaled
    descriptorHeapDesc.NumDescriptors = 7;
    descriptorHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    descriptorHeapDesc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    descriptorHeapDesc.NodeMask       = 0;
    device->CreateDescriptorHeap(&descriptorHeapDesc, IID_PPV_ARGS(&m_descriptorHeap));
    NAME_D3D12_OBJECT(m_descriptorHeap);

    m_descriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
}

// Build AABBs for procedural geometry within a bottom-level acceleration structure.
void DieXaR::BuildProceduralGeometryAABBs()
{
    switch (m_crtScene) {
        case SceneTypes::CornellBox:
            BuildProceduralGeometryAABBsCornellBox();
            break;
        case SceneTypes::Demo:
            BuildProceduralGeometryAABBsDemo();
            break;
        case SceneTypes::PbrShowcase:
            BuildProceduralGeometryAABBsPbrShowcase();
            break;
    }
}

void DieXaR::BuildProceduralGeometryAABBsDemo()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Set up AABBs on a grid.
    {
        XMINT3         aabbGrid     = XMINT3(4, 1, 4);
        const XMFLOAT3 basePosition = {
            -(aabbGrid.x * 2 + (aabbGrid.x - 1) * 2) / 2.0f,
            -(aabbGrid.y * 2 + (aabbGrid.y - 1) * 2) / 2.0f,
            -(aabbGrid.z * 2 + (aabbGrid.z - 1) * 2) / 2.0f,
        };

        XMFLOAT3 stride         = XMFLOAT3(2 + 2, 2 + 2, 2 + 2);
        auto     InitializeAABB = [&](auto& offsetIndex, auto& size) {
            return D3D12_RAYTRACING_AABB{
                basePosition.x + offsetIndex.x * stride.x,
                basePosition.y + offsetIndex.y * stride.y,
                basePosition.z + offsetIndex.z * stride.z,
                basePosition.x + offsetIndex.x * stride.x + size.x,
                basePosition.y + offsetIndex.y * stride.y + size.y,
                basePosition.z + offsetIndex.z * stride.z + size.z,
            };
        };

        m_aabbs.resize(m_scenes[m_crtScene].GetPrimitiveCount());
        UINT offset = 0;

        // Analytic primitives.
        {
            using namespace AnalyticPrimitive;
            m_aabbs[offset + AABB]    = InitializeAABB(XMFLOAT3(1.7f, 0, 0), XMFLOAT3(2, 3, 2));
            m_aabbs[offset + Spheres] = InitializeAABB(XMFLOAT3(1.0f, 0, 0.9f), XMFLOAT3(3, 3, 3));
            offset += AnalyticPrimitive::Count;
        }

        // Signed distance primitives.
        {
            using namespace SignedDistancePrimitive;
            // m_aabbs[offset + MiniSpheres] = InitializeAABB(XMINT3(2, 0, 0), XMFLOAT3(2, 2, 2));
            m_aabbs[offset + IntersectedRoundCube] = InitializeAABB(XMFLOAT3(0, 0, 1), XMFLOAT3(2, 2, 2));
            m_aabbs[offset + SquareTorus]          = InitializeAABB(XMFLOAT3(0.75f, -0.1f, 2.0f), XMFLOAT3(3, 3, 3));
            m_aabbs[offset + Cog]                  = InitializeAABB(XMFLOAT3(0.5f, 0, 0), XMFLOAT3(2, 2, 2));
            m_aabbs[offset + Cylinder]             = InitializeAABB(XMINT3(0, 0, 2), XMFLOAT3(2, 3, 2));
            m_aabbs[offset + SolidAngle]           = InitializeAABB(XMFLOAT3(1.0, 0, 1.0), XMFLOAT3(10, 10, 10));
        }
        AllocateUploadBuffer(device, m_aabbs.data(), m_aabbs.size() * sizeof(m_aabbs[0]), &m_aabbBuffer.resource);
    }
}

void DieXaR::BuildProceduralGeometryAABBsCornellBox()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Set up AABBs on a grid.
    {
        XMINT3         aabbGrid     = XMINT3(3, 1, 3);
        const XMFLOAT3 basePosition = {
            -(aabbGrid.x * 2 + (aabbGrid.x - 1) * 2) / 2.0f,
            -(aabbGrid.y * 2 + (aabbGrid.y - 1) * 2) / 2.0f,
            -(aabbGrid.z * 2 + (aabbGrid.z - 1) * 2) / 2.0f,
        };

        XMFLOAT3 stride         = XMFLOAT3(2, 2, 2);
        auto     InitializeAABB = [&](auto& offsetIndex, auto& size) {
            return D3D12_RAYTRACING_AABB{
                basePosition.x + offsetIndex.x * stride.x,
                basePosition.y + offsetIndex.y * stride.y,
                basePosition.z + offsetIndex.z * stride.z,
                basePosition.x + offsetIndex.x * stride.x + size.x,
                basePosition.y + offsetIndex.y * stride.y + size.y,
                basePosition.z + offsetIndex.z * stride.z + size.z,
            };
        };

        m_aabbs.resize(m_scenes[m_crtScene].GetPrimitiveCount());

        // Analytic primitives.
        {
            using namespace AnalyticPrimitive;
            m_aabbs[AABB]    = InitializeAABB(XMFLOAT3(0.2f, 0, -0.2f), XMFLOAT3(3, 3, 3));
            m_aabbs[Spheres] = InitializeAABB(XMFLOAT3(-0.5f, 0.0f, 1.0f), XMFLOAT3(2, 2, 2));
        }

        AllocateUploadBuffer(device, m_aabbs.data(), m_aabbs.size() * sizeof(m_aabbs[0]), &m_aabbBuffer.resource);
    }
}

void DieXaR::BuildProceduralGeometryAABBsPbrShowcase()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Set up AABBs on a grid.
    {
        XMINT3         aabbGrid     = XMINT3(5, 1, 5);
        const XMFLOAT3 basePosition = {
            -(aabbGrid.x * 2 + (aabbGrid.x - 1) * 2) / 2.0f,
            -(aabbGrid.y * 2 + (aabbGrid.y - 1) * 2) / 2.0f,
            -(aabbGrid.z * 2 + (aabbGrid.z - 1) * 2) / 2.0f,
        };

        XMFLOAT3 stride         = XMFLOAT3(2, 2, 2);
        auto     InitializeAABB = [&](auto& offsetIndex, auto& size) {
            return D3D12_RAYTRACING_AABB{
                basePosition.x + offsetIndex.x * stride.x,
                basePosition.y + offsetIndex.y * stride.y,
                basePosition.z + offsetIndex.z * stride.z,
                basePosition.x + offsetIndex.x * stride.x + size.x,
                basePosition.y + offsetIndex.y * stride.y + size.y,
                basePosition.z + offsetIndex.z * stride.z + size.z,
            };
        };

        m_aabbs.resize(m_scenes[m_crtScene].GetPrimitiveCount());

        // 5x8 grid
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 8; ++j) {
                int index      = i * 8 + j;
                m_aabbs[index] = InitializeAABB(XMFLOAT3(i, 0, j), XMFLOAT3(2, 2, 2));
            }

        AllocateUploadBuffer(device, m_aabbs.data(), m_aabbs.size() * sizeof(m_aabbs[0]), &m_aabbBuffer.resource);
    }
}

void DieXaR::BuildPlaneGeometry()
{
    auto device = m_deviceResources->GetD3DDevice();

    Index indices[] = {
        3,
        1,
        0,
        2,
        1,
        3,
    };
    Vertex vertices[] = {
        { XMFLOAT3(0.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(1.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(1.0f, 0.0f, 1.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(0.0f, 0.0f, 1.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
    };

    AllocateUploadBuffer(device, indices, sizeof(indices), &m_indexBuffer.resource);
    AllocateUploadBuffer(device, vertices, sizeof(vertices), &m_vertexBuffer.resource);

    // Vertex buffer is passed to the shader along with index buffer as a descriptor range.
    UINT descriptorIndexIB = CreateBufferSRV(&m_indexBuffer, sizeof(indices) / 4, 0);
    UINT descriptorIndexVB = CreateBufferSRV(&m_vertexBuffer, ARRAYSIZE(vertices), sizeof(vertices[0]));
    ThrowIfFalse(descriptorIndexVB == descriptorIndexIB + 1,
            L"Vertex Buffer descriptor index must follow that of Index Buffer descriptor index");
}

// Build geometry used in the sample.
void DieXaR::BuildGeometry()
{
    // Build procedural geometry
    BuildProceduralGeometryAABBs();

    // Build one plane geometry that will be used by both ground and square lights.
    BuildPlaneGeometry();
}

// Build geometry descs for bottom-level AS.
void DieXaR::BuildGeometryDescsForBottomLevelAS(
        array<vector<D3D12_RAYTRACING_GEOMETRY_DESC>, BottomLevelASType::Count>& geometryDescs)
{
    // Mark the geometry as opaque.
    // PERFORMANCE TIP: mark geometry as opaque whenever applicable as it can enable important ray processing
    // optimizations. Note: When rays encounter opaque geometry an any hit shader will not be executed whether it is
    // present or not.
    D3D12_RAYTRACING_GEOMETRY_FLAGS geometryFlags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;

    // Triangle geometry desc
    {
        // Triangle bottom-level AS contains a plane geometry and square lights.
        int numSquareLights = m_scenes[m_crtScene].CountGeometryLights();
        geometryDescs[BottomLevelASType::Triangle].resize(
                1 + numSquareLights + (m_crtScene == SceneTypes::CornellBox ? 4 : 0));

        // Plane geometry
        auto& geometryDesc                 = geometryDescs[BottomLevelASType::Triangle][0];
        geometryDesc                       = {};
        geometryDesc.Type                  = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
        geometryDesc.Triangles.IndexBuffer = m_indexBuffer.resource->GetGPUVirtualAddress();
        geometryDesc.Triangles.IndexCount  = static_cast<UINT>(m_indexBuffer.resource->GetDesc().Width) / sizeof(Index);
        geometryDesc.Triangles.IndexFormat = DXGI_FORMAT_R16_UINT;
        geometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
        geometryDesc.Triangles.VertexCount
                = static_cast<UINT>(m_vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
        geometryDesc.Triangles.VertexBuffer.StartAddress  = m_vertexBuffer.resource->GetGPUVirtualAddress();
        geometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
        geometryDesc.Flags                                = geometryFlags;

        // Square lights geometry
        for (int i = 1; i <= numSquareLights; ++i) {
            auto& geometryDesc                 = geometryDescs[BottomLevelASType::Triangle][i];
            geometryDesc                       = {};
            geometryDesc.Type                  = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
            geometryDesc.Triangles.IndexBuffer = m_indexBuffer.resource->GetGPUVirtualAddress();
            geometryDesc.Triangles.IndexCount
                    = static_cast<UINT>(m_indexBuffer.resource->GetDesc().Width) / sizeof(Index);
            geometryDesc.Triangles.IndexFormat  = DXGI_FORMAT_R16_UINT;
            geometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
            geometryDesc.Triangles.VertexCount
                    = static_cast<UINT>(m_vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
            geometryDesc.Triangles.VertexBuffer.StartAddress  = m_vertexBuffer.resource->GetGPUVirtualAddress();
            geometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
            geometryDesc.Flags                                = geometryFlags;
        }

        // Walls geometry
        if (m_crtScene == SceneTypes::CornellBox)
            for (int i = 1; i <= 4; ++i) {
                auto& geometryDesc                 = geometryDescs[BottomLevelASType::Triangle][i + numSquareLights];
                geometryDesc                       = {};
                geometryDesc.Type                  = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
                geometryDesc.Triangles.IndexBuffer = m_indexBuffer.resource->GetGPUVirtualAddress();
                geometryDesc.Triangles.IndexCount
                        = static_cast<UINT>(m_indexBuffer.resource->GetDesc().Width) / sizeof(Index);
                geometryDesc.Triangles.IndexFormat  = DXGI_FORMAT_R16_UINT;
                geometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
                geometryDesc.Triangles.VertexCount
                        = static_cast<UINT>(m_vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
                geometryDesc.Triangles.VertexBuffer.StartAddress  = m_vertexBuffer.resource->GetGPUVirtualAddress();
                geometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
                geometryDesc.Flags                                = geometryFlags;
            }
    }

    // AABB geometry desc
    {
        D3D12_RAYTRACING_GEOMETRY_DESC aabbDescTemplate = {};
        aabbDescTemplate.Type                           = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
        aabbDescTemplate.AABBs.AABBCount                = 1;
        aabbDescTemplate.AABBs.AABBs.StrideInBytes      = sizeof(D3D12_RAYTRACING_AABB);
        aabbDescTemplate.Flags                          = geometryFlags;

        // One AABB primitive per geometry.
        geometryDescs[BottomLevelASType::AABB].resize(m_scenes[m_crtScene].GetPrimitiveCount(), aabbDescTemplate);

        // Create AABB geometries.
        // Having separate geometries allows of separate shader record binding per geometry.
        // In this sample, this lets us specify custom hit groups per AABB geometry.
        for (UINT i = 0; i < m_scenes[m_crtScene].GetPrimitiveCount(); ++i) {
            auto& geometryDesc = geometryDescs[BottomLevelASType::AABB][i];
            geometryDesc.AABBs.AABBs.StartAddress
                    = m_aabbBuffer.resource->GetGPUVirtualAddress() + i * sizeof(D3D12_RAYTRACING_AABB);
        }
    }
}

AccelerationStructureBuffers DieXaR::BuildBottomLevelAS(const vector<D3D12_RAYTRACING_GEOMETRY_DESC>& geometryDescs,
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS                                           buildFlags)
{
    auto                   device      = m_deviceResources->GetD3DDevice();
    auto                   commandList = m_deviceResources->GetCommandList();
    ComPtr<ID3D12Resource> scratch;
    ComPtr<ID3D12Resource> bottomLevelAS;

    // Get the size requirements for the scratch and AS buffers.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC    bottomLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& bottomLevelInputs    = bottomLevelBuildDesc.Inputs;
    bottomLevelInputs.Type           = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    bottomLevelInputs.DescsLayout    = D3D12_ELEMENTS_LAYOUT_ARRAY;
    bottomLevelInputs.Flags          = buildFlags;
    bottomLevelInputs.NumDescs       = static_cast<UINT>(geometryDescs.size());
    bottomLevelInputs.pGeometryDescs = geometryDescs.data();

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO bottomLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&bottomLevelInputs, &bottomLevelPrebuildInfo);
    ThrowIfFalse(bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    // Create a scratch buffer.
    AllocateUAVBuffer(device, bottomLevelPrebuildInfo.ScratchDataSizeInBytes, &scratch,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"ScratchResource");

    // Allocate resources for acceleration structures.
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap
    // equivalent). Default heap is OK since the application doesn�t need CPU read/write access to them. The resources
    // that will contain acceleration structures must be created in the state
    // D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, and must have resource flag
    // D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both:
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the
    //  scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using
    //  UAV barriers.
    {
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
        AllocateUAVBuffer(device, bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes, &bottomLevelAS,
                initialResourceState, L"BottomLevelAccelerationStructure");
    }

    // bottom-level AS desc.
    {
        bottomLevelBuildDesc.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
        bottomLevelBuildDesc.DestAccelerationStructureData    = bottomLevelAS->GetGPUVirtualAddress();
    }

    // Build the acceleration structure.
    m_dxrCommandList->BuildRaytracingAccelerationStructure(&bottomLevelBuildDesc, 0, nullptr);

    AccelerationStructureBuffers bottomLevelASBuffers;
    bottomLevelASBuffers.accelerationStructure    = bottomLevelAS;
    bottomLevelASBuffers.scratch                  = scratch;
    bottomLevelASBuffers.ResultDataMaxSizeInBytes = bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes;
    return bottomLevelASBuffers;
}

template <class BLASPtrType>
void DieXaR::BuildBotomLevelASInstanceDescs(
        BLASPtrType bottomLevelASaddresses[NUM_BLAS], ComPtr<ID3D12Resource>* instanceDescsResource)
{
    auto device = m_deviceResources->GetD3DDevice();

    vector<D3D12_RAYTRACING_INSTANCE_DESC> instanceDescs;
    UINT                                   numWalls = (m_crtScene == SceneTypes::CornellBox ? 4 : 0);
    instanceDescs.resize(NUM_BLAS + m_scenes[m_crtScene].CountGeometryLights() + numWalls);

    UINT c_aabbWidth    = 2;
    UINT c_aabbDistance = 2;

    // Width of a bottom-level AS geometry.
    // Make the plane a little larger than the actual number of primitives in each dimension.
    const XMUINT3  NUM_AABB = XMUINT3(700, 1, 700);
    XMFLOAT3       fWidth   = XMFLOAT3(NUM_AABB.x * c_aabbWidth + (NUM_AABB.x - 1) * c_aabbDistance,
                    NUM_AABB.y * c_aabbWidth + (NUM_AABB.y - 1) * c_aabbDistance,
                    NUM_AABB.z * c_aabbWidth + (NUM_AABB.z - 1) * c_aabbDistance);
    const XMVECTOR vWidth   = XMLoadFloat3(&fWidth);

    // Bottom-level AS with a single plane.
    {
        auto& instanceDesc                               = instanceDescs[BottomLevelASType::Triangle];
        instanceDesc                                     = {};
        instanceDesc.InstanceMask                        = 1;
        instanceDesc.InstanceContributionToHitGroupIndex = 0;
        instanceDesc.AccelerationStructure               = bottomLevelASaddresses[BottomLevelASType::Triangle];

        // Calculate transformation matrix.
        const XMVECTOR vBasePosition = m_crtScene == SceneTypes::CornellBox
                                               ? XMLoadFloat3(&XMFLOAT3(-2.75f, -0.2f, -5.0f))
                                               : vWidth * XMLoadFloat3(&XMFLOAT3(-0.35f, 0.0f, -0.35f));

        // Scale in XZ dimensions.
        if (m_crtScene == SceneTypes::CornellBox)
            fWidth = XMFLOAT3(5.5f, 1.0f, 7.0f);
        XMMATRIX mScale       = XMMatrixScaling(fWidth.x, fWidth.y, fWidth.z);
        XMMATRIX mTranslation = XMMatrixTranslationFromVector(vBasePosition);
        XMMATRIX mTransform   = mScale * mTranslation;
        XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), mTransform);
    }

    // Instance descriptor for each square light.
    auto squareLightIndices = m_scenes[m_crtScene].GetGeometryLightsIndices();
    {
        for (int i = 0; i < squareLightIndices.size(); ++i) {
            auto& instanceDesc                               = instanceDescs[BottomLevelASType::Count + i];
            instanceDesc                                     = {};
            instanceDesc.InstanceMask                        = 1;
            instanceDesc.InstanceContributionToHitGroupIndex = RayType::Count * (i + 1);
            instanceDesc.AccelerationStructure               = bottomLevelASaddresses[BottomLevelASType::Triangle];

            // Calculate transformation matrix.
            const auto& light    = m_scenes[m_crtScene].m_lights[squareLightIndices[i]];
            XMFLOAT3    position = light.position;
            position.x -= 0.5f * light.size;
            position.z -= 0.5f * light.size;
            const XMVECTOR vBasePosition = XMLoadFloat3(&position);

            // Scale in XZ dimensions.
            XMMATRIX mScale       = XMMatrixScaling(light.size, 1.0f, light.size);
            XMMATRIX mTranslation = XMMatrixTranslationFromVector(vBasePosition);
            XMMATRIX mTransform   = mScale * mTranslation;
            XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), mTransform);
        }
    }

    // Instance descriptor for each wall.
    if (numWalls > 0) {
        for (int i = 0; i < numWalls; ++i) {
            auto& instanceDesc        = instanceDescs[BottomLevelASType::Count + squareLightIndices.size() + i];
            instanceDesc              = {};
            instanceDesc.InstanceMask = 1;
            instanceDesc.InstanceContributionToHitGroupIndex = RayType::Count * (squareLightIndices.size() + i + 1);
            instanceDesc.AccelerationStructure               = bottomLevelASaddresses[BottomLevelASType::Triangle];

            // Calculate transformation matrix.
            const XMVECTOR vBasePosition = XMLoadFloat3(&XMFLOAT3(-2.75f, -0.2f, -5.0f));

            XMMATRIX mScaleOffset = XMMatrixIdentity();
            XMMATRIX mRotation    = XMMatrixIdentity();
            XMVECTOR mOffset      = XMVectorSet(0.0f, 0.0f, 0.0f, 0.0f);
            if (i == 0) {
                mRotation    = XMMatrixRotationX(XM_PI);
                mOffset      = XMLoadFloat3(&XMFLOAT3(0.0f, 5.0f, 7.0f));
                mScaleOffset = XMMatrixScaling(1.0f, 1.0f, 7.0f / 5.0f);
            }
            else if (i == 1) {
                mRotation = XMMatrixRotationX(XM_PIDIV2);
                mOffset   = XMLoadFloat3(&XMFLOAT3(0.0f, 5.0f, 0.0f));
            }
            else if (i == 2) {
                mRotation    = XMMatrixRotationZ(-XM_PIDIV2);
                mOffset      = XMLoadFloat3(&XMFLOAT3(0.0f, 5.0f, 0.0f));
                mScaleOffset = XMMatrixScaling(5.0f / 5.5f, 1.0f, 7.0f / 5.0f);
            }
            else if (i == 3) {
                mRotation    = XMMatrixRotationZ(XM_PIDIV2) * XMMatrixRotationX(XM_PI);
                mOffset      = XMLoadFloat3(&XMFLOAT3(5.5f, 5.0f, 7.0f));
                mScaleOffset = XMMatrixScaling(5.0f / 5.5f, 1.0f, 7.0f / 5.0f);
            }

            XMMATRIX mScale       = XMMatrixScaling(5.5f, 1.0f, 5.0f);
            XMMATRIX mTranslation = XMMatrixTranslationFromVector(vBasePosition + mOffset);
            XMMATRIX mTransform   = mScaleOffset * mScale * mRotation * mTranslation;
            XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), mTransform);
        }
    }

    // Create instanced bottom-level AS with procedural geometry AABBs.
    // Instances share all the data, except for a transform.
    {
        auto& instanceDesc        = instanceDescs[BottomLevelASType::AABB];
        instanceDesc              = {};
        instanceDesc.InstanceMask = 1;

        // Set hit group offset to beyond the shader records for the triangle AABB.
        instanceDesc.InstanceContributionToHitGroupIndex = (1 + squareLightIndices.size() + numWalls) * RayType::Count;
        instanceDesc.AccelerationStructure               = bottomLevelASaddresses[BottomLevelASType::AABB];

        // Move all AABBS above the ground plane and center them inside the checkerboard.
        float    yOffset      = m_crtScene == SceneTypes::Demo ? 0.0f : 0.2f;
        XMMATRIX mTranslation = XMMatrixTranslationFromVector(
                XMLoadFloat3(&XMFLOAT3(c_aabbWidth * 2.0f, c_aabbWidth / 2 - yOffset, c_aabbWidth * 0.5f)));
        XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), mTranslation);
    }
    UINT64 bufferSize = static_cast<UINT64>(instanceDescs.size() * sizeof(instanceDescs[0]));
    AllocateUploadBuffer(device, instanceDescs.data(), bufferSize, &(*instanceDescsResource), L"InstanceDescs");
};

AccelerationStructureBuffers DieXaR::BuildTopLevelAS(
        AccelerationStructureBuffers                        bottomLevelAS[BottomLevelASType::Count],
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags)
{
    auto                   device      = m_deviceResources->GetD3DDevice();
    auto                   commandList = m_deviceResources->GetCommandList();
    ComPtr<ID3D12Resource> scratch;
    ComPtr<ID3D12Resource> topLevelAS;

    // Get required sizes for an acceleration structure.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC    topLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& topLevelInputs    = topLevelBuildDesc.Inputs;
    topLevelInputs.Type        = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    topLevelInputs.Flags       = buildFlags;
    topLevelInputs.NumDescs
            = NUM_BLAS + m_scenes[m_crtScene].CountGeometryLights() + (m_crtScene == SceneTypes::CornellBox ? 4 : 0);

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO topLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&topLevelInputs, &topLevelPrebuildInfo);
    ThrowIfFalse(topLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    AllocateUAVBuffer(device, topLevelPrebuildInfo.ScratchDataSizeInBytes, &scratch,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"ScratchResource");

    // Allocate resources for acceleration structures.
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap
    // equivalent). Default heap is OK since the application doesn�t need CPU read/write access to them. The resources
    // that will contain acceleration structures must be created in the state
    // D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, and must have resource flag
    // D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both:
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the
    //  scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using
    //  UAV barriers.
    {
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
        AllocateUAVBuffer(device, topLevelPrebuildInfo.ResultDataMaxSizeInBytes, &topLevelAS, initialResourceState,
                L"TopLevelAccelerationStructure");
    }

    // Create instance descs for the bottom-level acceleration structures.
    ComPtr<ID3D12Resource> instanceDescsResource;
    {
        D3D12_GPU_VIRTUAL_ADDRESS bottomLevelASaddresses[BottomLevelASType::Count]
                = { bottomLevelAS[0].accelerationStructure->GetGPUVirtualAddress(),
                      bottomLevelAS[1].accelerationStructure->GetGPUVirtualAddress() };
        BuildBotomLevelASInstanceDescs(bottomLevelASaddresses, &instanceDescsResource);
    }

    // Top-level AS desc
    {
        topLevelBuildDesc.DestAccelerationStructureData    = topLevelAS->GetGPUVirtualAddress();
        topLevelInputs.InstanceDescs                       = instanceDescsResource->GetGPUVirtualAddress();
        topLevelBuildDesc.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
    }

    // Build acceleration structure.
    m_dxrCommandList->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);

    AccelerationStructureBuffers topLevelASBuffers;
    topLevelASBuffers.accelerationStructure    = topLevelAS;
    topLevelASBuffers.instanceDesc             = instanceDescsResource;
    topLevelASBuffers.scratch                  = scratch;
    topLevelASBuffers.ResultDataMaxSizeInBytes = topLevelPrebuildInfo.ResultDataMaxSizeInBytes;
    return topLevelASBuffers;
}

// Build acceleration structure needed for raytracing.
void DieXaR::BuildAccelerationStructures()
{
    auto device           = m_deviceResources->GetD3DDevice();
    auto commandList      = m_deviceResources->GetCommandList();
    auto commandQueue     = m_deviceResources->GetCommandQueue();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();

    // Reset the command list for the acceleration structure construction.
    commandList->Reset(commandAllocator, nullptr);

    // Build bottom-level AS.
    AccelerationStructureBuffers                                            bottomLevelAS[BottomLevelASType::Count];
    array<vector<D3D12_RAYTRACING_GEOMETRY_DESC>, BottomLevelASType::Count> geometryDescs;
    {
        BuildGeometryDescsForBottomLevelAS(geometryDescs);

        // Build all bottom-level AS.
        for (UINT i = 0; i < BottomLevelASType::Count; i++)
            bottomLevelAS[i] = BuildBottomLevelAS(geometryDescs[i]);
    }

    // Batch all resource barriers for bottom-level AS builds.
    D3D12_RESOURCE_BARRIER resourceBarriers[BottomLevelASType::Count];
    for (UINT i = 0; i < BottomLevelASType::Count; i++) {
        resourceBarriers[i] = CD3DX12_RESOURCE_BARRIER::UAV(bottomLevelAS[i].accelerationStructure.Get());
    }
    commandList->ResourceBarrier(BottomLevelASType::Count, resourceBarriers);

    // Build top-level AS.
    AccelerationStructureBuffers topLevelAS = BuildTopLevelAS(bottomLevelAS);

    // Kick off acceleration structure construction.
    m_deviceResources->ExecuteCommandList();

    // Wait for GPU to finish as the locally created temporary GPU resources will get released once we go out of scope.
    m_deviceResources->WaitForGpu();

    // Store the AS buffers. The rest of the buffers will be released once we exit the function.
    for (UINT i = 0; i < BottomLevelASType::Count; i++) {
        m_bottomLevelAS[i] = bottomLevelAS[i].accelerationStructure;
    }
    m_topLevelAS = topLevelAS.accelerationStructure;
}

// Build shader tables.
// This encapsulates all shader records - shaders and the arguments for their local root signatures.
void DieXaR::BuildShaderTables()
{
    auto device = m_deviceResources->GetD3DDevice();

    void* rayGenShaderID;
    void* missShaderIDs[RayType::Count];
    void* hitGroupShaderIDs_TriangleGeometry[RayType::Count];
    void* hitGroupShaderIDs_AABBGeometry[IntersectionShaderType::Count][RayType::Count];

    // A shader name look-up table for shader table debug print out.
    unordered_map<void*, wstring> shaderIdToStringMap;

    auto GetShaderIDs = [&](auto* stateObjectProperties) {
        rayGenShaderID = stateObjectProperties->GetShaderIdentifier(c_raygenShaderNames[m_raytracingType]);
        shaderIdToStringMap[rayGenShaderID] = c_raygenShaderNames[m_raytracingType];

        for (UINT i = 0; i < RayType::Count; i++) {
            missShaderIDs[i]                      = stateObjectProperties->GetShaderIdentifier(c_missShaderNames[i]);
            shaderIdToStringMap[missShaderIDs[i]] = c_missShaderNames[i];
        }
        for (UINT i = 0; i < RayType::Count; i++) {
            hitGroupShaderIDs_TriangleGeometry[i]
                    = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_TriangleGeometry[i]);
            shaderIdToStringMap[hitGroupShaderIDs_TriangleGeometry[i]] = c_hitGroupNames_TriangleGeometry[i];
        }
        for (UINT r = 0; r < IntersectionShaderType::Count; r++)
            for (UINT c = 0; c < RayType::Count; c++) {
                hitGroupShaderIDs_AABBGeometry[r][c]
                        = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_AABBGeometry[r][c]);
                shaderIdToStringMap[hitGroupShaderIDs_AABBGeometry[r][c]] = c_hitGroupNames_AABBGeometry[r][c];
            }
    };

    // Get shader identifiers.
    UINT shaderIDSize;
    {
        ComPtr<ID3D12StateObjectProperties> stateObjectProperties;
        ThrowIfFailed(m_dxrStateObject.As(&stateObjectProperties));
        GetShaderIDs(stateObjectProperties.Get());
        shaderIDSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
    }

    /*************--------- Shader table layout -------*******************
    | --------------------------------------------------------------------
    | Shader table - HitGroupShaderTable:
    | [0] : MyHitGroup_Triangle
    | [1] : MyHitGroup_Triangle_ShadowRay
    | [2] : MyHitGroup_AABB_AnalyticPrimitive
    | [3] : MyHitGroup_AABB_AnalyticPrimitive_ShadowRay
    | ...
    | [6] : MyHitGroup_AABB_VolumetricPrimitive
    | [7] : MyHitGroup_AABB_VolumetricPrimitive_ShadowRay
    | [8] : MyHitGroup_AABB_SignedDistancePrimitive
    | [9] : MyHitGroup_AABB_SignedDistancePrimitive_ShadowRay,
    | ...
    | [20] : MyHitGroup_AABB_SignedDistancePrimitive
    | [21] : MyHitGroup_AABB_SignedDistancePrimitive_ShadowRay
    | --------------------------------------------------------------------
    **********************************************************************/

    // RayGen shader table.
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIDSize;  // No root arguments

        ShaderTable rayGenShaderTable(device, numShaderRecords, shaderRecordSize, L"RayGenShaderTable");
        rayGenShaderTable.push_back(ShaderRecord(rayGenShaderID, shaderRecordSize, nullptr, 0));
        rayGenShaderTable.DebugPrint(shaderIdToStringMap);
        m_rayGenShaderTable = rayGenShaderTable.GetResource();
    }

    // Miss shader table.
    {
        UINT numShaderRecords = RayType::Count;
        UINT shaderRecordSize = shaderIDSize;  // No root arguments

        ShaderTable missShaderTable(device, numShaderRecords, shaderRecordSize, L"MissShaderTable");
        for (UINT i = 0; i < RayType::Count; i++) {
            missShaderTable.push_back(ShaderRecord(missShaderIDs[i], shaderIDSize, nullptr, 0));
        }
        missShaderTable.DebugPrint(shaderIdToStringMap);
        m_missShaderTableStrideInBytes = missShaderTable.GetShaderRecordSize();
        m_missShaderTable              = missShaderTable.GetResource();
    }

    // Hit group shader table.
    {
        UINT numSquareLights  = m_scenes[m_crtScene].CountGeometryLights();
        UINT numWalls         = (m_crtScene == SceneTypes::CornellBox ? 4 : 0);
        UINT numShaderRecords = RayType::Count * (1 + numSquareLights + numWalls)
                                + m_scenes[m_crtScene].GetPrimitiveCount() * RayType::Count;
        UINT        shaderRecordSize = shaderIDSize + LocalRootSignature::MaxRootArgumentsSize();
        ShaderTable hitGroupShaderTable(device, numShaderRecords, shaderRecordSize, L"HitGroupShaderTable");

        // Triangle geometry hit groups.
        {
            LocalRootSignature::Triangle::RootArguments rootArgs;

            // Plane
            rootArgs.materialCb                = m_scenes[m_crtScene].m_planeMaterialCB;
            rootArgs.pbrCb                     = m_scenes[m_crtScene].m_pbrPlaneMaterialCB;
            rootArgs.primitiveCB.instanceIndex = 0;
            rootArgs.primitiveCB.primitiveType = 0;

            for (auto& hitGroupShaderID : hitGroupShaderIDs_TriangleGeometry) {
                hitGroupShaderTable.push_back(
                        ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
            }

            // Square lights
            rootArgs.primitiveCB.primitiveType = 1;

            auto squareLightIndices = m_scenes[m_crtScene].GetGeometryLightsIndices();
            for (int i = 0; i < numSquareLights; ++i) {
                rootArgs.primitiveCB.instanceIndex = squareLightIndices[i];
                for (auto& hitGroupShaderID : hitGroupShaderIDs_TriangleGeometry) {
                    hitGroupShaderTable.push_back(
                            ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
                }
            }

            // Walls
            if (m_crtScene == SceneTypes::CornellBox) {
                rootArgs.primitiveCB.primitiveType = 0;
                for (int i = 0; i < 4; ++i) {
                    rootArgs.materialCb                = m_scenes[m_crtScene].m_materialCB[i];
                    rootArgs.pbrCb                     = m_scenes[m_crtScene].m_pbrMaterialCB[i];
                    rootArgs.primitiveCB.instanceIndex = i + 1;
                    for (auto& hitGroupShaderID : hitGroupShaderIDs_TriangleGeometry) {
                        hitGroupShaderTable.push_back(
                                ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
                    }
                }
            }
        }

        // AABB geometry hit groups.
        {
            LocalRootSignature::AABB::RootArguments rootArgs;
            UINT                                    instanceIndex = 0;

            // Create a shader record for each primitive.
            for (UINT iShader = 0, instanceIndex = 0; iShader < IntersectionShaderType::Count; iShader++) {
                UINT numPrimitiveTypes = m_scenes[m_crtScene].m_primitiveCount[iShader];

                // Primitives for each intersection shader.
                for (UINT primitiveIndex = 0; primitiveIndex < numPrimitiveTypes; primitiveIndex++, instanceIndex++) {
                    rootArgs.materialCb                = m_scenes[m_crtScene].m_materialCB[numWalls + instanceIndex];
                    rootArgs.pbrCb                     = m_scenes[m_crtScene].m_pbrMaterialCB[numWalls + instanceIndex];
                    rootArgs.primitiveCB.instanceIndex = instanceIndex;
                    switch (m_crtScene) {
                        case SceneTypes::Demo:
                            rootArgs.primitiveCB.primitiveType = primitiveIndex;
                            break;
                        case SceneTypes::PbrShowcase:
                            rootArgs.primitiveCB.primitiveType = 1;
                            break;
                        case SceneTypes::CornellBox:
                            rootArgs.primitiveCB.primitiveType = primitiveIndex;
                            break;
                    }

                    // Ray types.
                    for (UINT r = 0; r < RayType::Count; r++) {
                        auto& hitGroupShaderID = hitGroupShaderIDs_AABBGeometry[iShader][r];
                        hitGroupShaderTable.push_back(
                                ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
                    }
                }
            }
        }

        hitGroupShaderTable.DebugPrint(shaderIdToStringMap);
        m_hitGroupShaderTableStrideInBytes = hitGroupShaderTable.GetShaderRecordSize();
        m_hitGroupShaderTable              = hitGroupShaderTable.GetResource();
    }
}

void DieXaR::OnKeyDown(UINT8 key)
{
    switch (key) {
        case VK_ESCAPE:
            // Close the application gracefully.
            OnDestroy();
            PostQuitMessage(0);
            break;
        case VK_F1:
            m_shouldReload = true;
            break;
        case 'C':
            if (m_cameraLocked)
                break;
            m_cameraFly = !m_cameraFly;
            if (!m_cameraFly)
                ResetCamera(m_scenes[m_crtScene].m_eye, m_scenes[m_crtScene].m_at);
            else if (!m_cameraLocked) {
                // Move the focus point very close to the camera to make camera rotation more intuitive.
                m_at          = m_eye + XMVector3Normalize(m_at - m_eye) * 0.01f;
                m_cameraSpeed = m_cameraBaseMoveSpeed;
            }
            break;
        case 'G':
            m_animateGeometry = !m_animateGeometry;
            break;
        case 'R':
            m_cameraLocked     = !m_cameraLocked;
            m_cameraMovingLeft = m_cameraMovingRight = m_cameraMovingForward = m_cameraMovingBackward = m_cameraMovingUp
                    = m_cameraMovingDown                                                              = false;
            break;
        case 'A':
            if (m_cameraLocked)
                break;
            if (!m_cameraFly)
                m_cameraSpeed -= m_cameraBaseRotateSpeed;
            else
                m_cameraMovingLeft = true;
            break;
        case 'D':
            if (m_cameraLocked)
                break;
            if (!m_cameraFly)
                m_cameraSpeed += m_cameraBaseRotateSpeed;
            else
                m_cameraMovingRight = true;
            break;
        case 'W':
            if (m_cameraLocked)
                break;
            if (m_cameraFly)
                m_cameraMovingForward = true;
            break;
        case 'S':
            if (m_cameraLocked)
                break;
            if (m_cameraFly)
                m_cameraMovingBackward = true;
            break;
        case 'E':
            if (m_cameraLocked)
                break;
            if (m_cameraFly)
                m_cameraMovingUp = true;
            break;
        case 'Q':
            if (m_cameraLocked)
                break;
            if (m_cameraFly)
                m_cameraMovingDown = true;
            break;
        case VK_SHIFT:
            if (m_cameraFly)
                m_cameraSpeed = m_cameraBaseMoveSpeed / 4.0f;
            break;
    }
}

void DieXaR::OnKeyUp(UINT8 key)
{
    switch (key) {
        case 'A':
            m_cameraMovingLeft = false;
            break;
        case 'D':
            m_cameraMovingRight = false;
            break;
        case 'W':
            m_cameraMovingForward = false;
            break;
        case 'S':
            m_cameraMovingBackward = false;
            break;
        case 'E':
            m_cameraMovingUp = false;
            break;
        case 'Q':
            m_cameraMovingDown = false;
            break;
        case VK_SHIFT:
            if (m_cameraLocked)
                break;
            if (m_cameraFly)
                m_cameraSpeed = m_cameraBaseMoveSpeed;
            break;
    }
}

void DieXaR::OnMouseMove(UINT x, UINT y)
{
    static POINT lastMousePos;
    float        dx = XMConvertToRadians(0.25f * static_cast<float>((LONG) x - lastMousePos.x));
    float        dy = -XMConvertToRadians(0.25f * static_cast<float>((LONG) y - lastMousePos.y));

    if (!m_cameraLocked && m_cameraFly) {
        // Reset path tracing if we're moving
        if (dx != 0.0f || dy != 0.0f)
            ResetPathTracing();

        // rotate camera
        XMMATRIX R   = XMMatrixRotationY(dx);
        XMVECTOR eye = m_eye - m_at;
        eye          = XMVector3Transform(eye, R);
        m_eye        = m_at + eye;

        XMVECTOR up = XMVector3Transform(m_up, R);
        m_up        = up;

        XMMATRIX pitch = XMMatrixRotationAxis(XMVector3Normalize(XMVector3Cross(m_at - m_eye, m_up)), dy);
        eye            = XMVector3Transform(eye, pitch);
        up             = XMVector3Transform(up, pitch);
        m_eye          = m_at + eye;
        m_up           = up;

        UpdateCameraMatrices();
    }

    lastMousePos.x = x;
    lastMousePos.y = y;
}

// Update frame-based values.
void DieXaR::OnUpdate()
{
    if (m_shouldReload) {
        Reload();
    }

    m_timer.Tick();
    CalculateFrameStats();
    float        elapsedTime    = static_cast<float>(m_timer.GetElapsedSeconds());
    unsigned int ticks          = static_cast<unsigned int>(m_timer.GetTotalTicks());
    auto         frameIndex     = m_deviceResources->GetCurrentFrameIndex();
    auto         prevFrameIndex = m_deviceResources->GetPreviousFrameIndex();

    // Write settings to CB
    ResetSettingsCB();

    // Show ImGui.
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
    ShowUI();

    // Rotate the camera around Y axis.
    if (!m_cameraFly) {
        // Reset path tracing if we're moving
        if (m_cameraSpeed != 0.0f)
            ResetPathTracing();
        float    secondsToRotateAround = -48.0f / m_cameraSpeed;
        float    angleToRotateBy       = 360.0f * (elapsedTime / secondsToRotateAround);
        XMMATRIX rotate                = XMMatrixRotationY(XMConvertToRadians(angleToRotateBy));
        m_eye                          = XMVector3Transform(m_eye, rotate);
        m_up                           = XMVector3Transform(m_up, rotate);
        m_at                           = XMVector3Transform(m_at, rotate);
        UpdateCameraMatrices();
    }
    // Camera fly mode.
    else {
        // Reset Path Tracing if camera is moving.
        if (m_cameraMovingUp || m_cameraMovingDown || m_cameraMovingLeft || m_cameraMovingRight || m_cameraMovingForward
                || m_cameraMovingBackward)
            ResetPathTracing();

        XMVECTOR forward = m_at - m_eye;
        forward          = XMVector3Normalize(forward);
        XMVECTOR right   = -XMVector3Cross(forward, m_up);
        if (m_cameraMovingUp) {
            m_eye += m_up * m_cameraSpeed * elapsedTime;
            m_at += m_up * m_cameraSpeed * elapsedTime;
        }
        if (m_cameraMovingDown) {
            m_eye -= m_up * m_cameraSpeed * elapsedTime;
            m_at -= m_up * m_cameraSpeed * elapsedTime;
        }
        if (m_cameraMovingLeft) {
            m_eye -= right * m_cameraSpeed * elapsedTime;
            m_at -= right * m_cameraSpeed * elapsedTime;
        }
        if (m_cameraMovingRight) {
            m_eye += right * m_cameraSpeed * elapsedTime;
            m_at += right * m_cameraSpeed * elapsedTime;
        }
        if (m_cameraMovingForward) {
            m_eye += forward * m_cameraSpeed * elapsedTime;
            m_at += forward * m_cameraSpeed * elapsedTime;
        }
        if (m_cameraMovingBackward) {
            m_eye -= forward * m_cameraSpeed * elapsedTime;
            m_at -= forward * m_cameraSpeed * elapsedTime;
        }
        UpdateCameraMatrices();
    }

    // Transform the procedural geometry.
    if (m_animateGeometry)
        m_animateGeometryTime += elapsedTime;
    UpdateAABBPrimitiveAttributes(m_animateGeometryTime);
    m_sceneCB->elapsedTime += elapsedTime;
    m_sceneCB->elapsedTicks = ticks;

    // Update lights buffer
    for (UINT i = 0; i < m_lights.NumInstances(); ++i)
        m_lights[i] = m_scenes[m_crtScene].m_lights[i];

    // Rebuild acceleration structures if needed.
    if (m_shouldRebuildAccelerationStructures) {
        BuildAccelerationStructures();
        m_shouldRebuildAccelerationStructures = false;
    }
}

void DieXaR::DoRaytracing()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex  = m_deviceResources->GetCurrentFrameIndex();

    auto DispatchRays = [&](auto* raytracingCommandList, auto* stateObject, auto* dispatchDesc) {
        dispatchDesc->HitGroupTable.StartAddress             = m_hitGroupShaderTable->GetGPUVirtualAddress();
        dispatchDesc->HitGroupTable.SizeInBytes              = m_hitGroupShaderTable->GetDesc().Width;
        dispatchDesc->HitGroupTable.StrideInBytes            = m_hitGroupShaderTableStrideInBytes;
        dispatchDesc->MissShaderTable.StartAddress           = m_missShaderTable->GetGPUVirtualAddress();
        dispatchDesc->MissShaderTable.SizeInBytes            = m_missShaderTable->GetDesc().Width;
        dispatchDesc->MissShaderTable.StrideInBytes          = m_missShaderTableStrideInBytes;
        dispatchDesc->RayGenerationShaderRecord.StartAddress = m_rayGenShaderTable->GetGPUVirtualAddress();
        dispatchDesc->RayGenerationShaderRecord.SizeInBytes  = m_rayGenShaderTable->GetDesc().Width;
        dispatchDesc->Width                                  = m_renderWidth;
        dispatchDesc->Height                                 = m_renderHeight;
        dispatchDesc->Depth                                  = 1;
        raytracingCommandList->SetPipelineState1(stateObject);

        m_gpuTimers[GpuTimers::Raytracing].Start(commandList);
        raytracingCommandList->DispatchRays(dispatchDesc);
        m_gpuTimers[GpuTimers::Raytracing].Stop(commandList);
    };

    auto SetCommonPipelineState = [&](auto* descriptorSetCommandList) {
        descriptorSetCommandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
        // Set index and successive vertex buffer decriptor tables.
        commandList->SetComputeRootDescriptorTable(
                GlobalRootSignature::Slot::VertexBuffers, m_indexBuffer.gpuDescriptorHandle);
        commandList->SetComputeRootDescriptorTable(
                GlobalRootSignature::Slot::OutputView, m_raytracingOutputResourceUAVGpuDescriptor);
        commandList->SetComputeRootDescriptorTable(
                GlobalRootSignature::Slot::WorldPositionOutput, m_worldPositionOutputResourceUAVGpuDescriptor);
    };

    commandList->SetComputeRootSignature(m_raytracingGlobalRootSignature.Get());

    // Copy dynamic buffers to GPU.
    {
        m_sceneCB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootConstantBufferView(
                GlobalRootSignature::Slot::SceneConstant, m_sceneCB.GpuVirtualAddress(frameIndex));

        m_lights.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(
                GlobalRootSignature::Slot::LightBuffer, m_lights.GpuVirtualAddress(frameIndex));

        m_aabbPrimitiveAttributeBuffer.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::AABBattributeBuffer,
                m_aabbPrimitiveAttributeBuffer.GpuVirtualAddress(frameIndex));
    }

    // Bind the heaps, acceleration structure and dispatch rays.
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    SetCommonPipelineState(commandList);
    commandList->SetComputeRootShaderResourceView(
            GlobalRootSignature::Slot::AccelerationStructure, m_topLevelAS->GetGPUVirtualAddress());
    DispatchRays(m_dxrCommandList.Get(), m_dxrStateObject.Get(), &dispatchDesc);

    if (m_raytracingType == RaytracingType::PathTracingTemporal) {
        AdvancePathTracing();
    }
}

void DieXaR::ShowUI()
{
    bool m_shouldResetPathTracing = false;

    IM_ASSERT(ImGui::GetCurrentContext() != NULL && "No ImGui context.");

    if (!ImGui::Begin("Settings")) {
        // Don't draw if the window is collapsed.
        ImGui::End();
        return;
    }

    ImGuiIO& io = ImGui::GetIO();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::Text("Elapsed time: %.2f (s)", m_sceneCB->elapsedTime);
    ImGui::Text("Elapsed ticks: %u", m_sceneCB->elapsedTicks);
    ImGui::Text("Camera position: (%.2f, %.2f, %.2f)", XMVectorGetX(m_eye), XMVectorGetY(m_eye), XMVectorGetZ(m_eye));
    XMVECTOR forward = m_at - m_eye;
    forward          = XMVector3Normalize(forward);
    ImGui::Text("Camera direction: (%.2f, %.2f, %.2f)", XMVectorGetX(forward), XMVectorGetY(forward),
            XMVectorGetZ(forward));
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("FidelityFX FSR")) {
        ImGui::Spacing();
        ImGui::Text("Render resolution: %ux%u", m_renderWidth, m_renderHeight);
        ImGui::Text("Window resolution: %u%u", m_width, m_height);

        const char* presetOptions[]
                = { "Quality (1.5x)", "Balanced (1.7x)", "Performance (2.0x)", "Ultra Performance (3.0x)" };
        int currentPreset = 0;
        if (m_upscaleRatio == 1.5f)
            currentPreset = 0;
        else if (m_upscaleRatio == 1.7f)
            currentPreset = 1;
        else if (m_upscaleRatio == 2.0f)
            currentPreset = 2;
        else if (m_upscaleRatio == 3.0f)
            currentPreset = 3;

        if (ImGui::Combo("Quality Preset", &currentPreset, presetOptions, IM_ARRAYSIZE(presetOptions))) {
            switch (currentPreset) {
                case 0:
                    m_upscaleRatio = 1.5f;
                    break;
                case 1:
                    m_upscaleRatio = 1.7f;
                    break;
                case 2:
                    m_upscaleRatio = 2.0f;
                    break;
                case 3:
                    m_upscaleRatio = 3.0f;
                    break;
            }

            // When changing ratio, trigger a resize and reset FSR history
            m_shouldReload = true;
            m_resetUpscale = true;
        }

        ImGui::Checkbox("Enable RCAS Sharpening", &m_enableSharpening);
        if (m_enableSharpening) {
            ImGui::SliderFloat("Sharpness", &m_sharpness, 0.f, 1.f);
        }

        if (ImGui::Button("Reset History")) {
            m_resetUpscale = true;
        }

        ImGui::Spacing();
        ImGui::Checkbox("Visualize motion vectors", &m_visualizeMotionVectors);
    }

    if (ImGui::CollapsingHeader("Controls")) {
        ImGui::Spacing();

        ImGui::SeparatorText("Keyboard");
        ImGui::BulletText("F1 - Reload graphics");
        ImGui::BulletText("ESC - Exit application");
        ImGui::BulletText("C - Toggle camera fly/revolution mode");
        ImGui::BulletText("WASD - Move camera (fly mode)");
        ImGui::BulletText("EQ - Move camera up/down (fly mode)");
        ImGui::BulletText("Shift - Move camera slower (fly mode)");
        ImGui::BulletText("AD - Modify camera speed (revolution mode)");
        ImGui::BulletText("R - Stop camera (revolution mode)");

        ImGui::SeparatorText("Mouse");
        ImGui::BulletText("Camera movement available only in fly mode");

        ImGui::Spacing();
    }

    if (ImGui::CollapsingHeader("Renderer")) {
        ImGui::Spacing();

        // Jitter
        if (m_raytracingType != RaytracingType::Whitted) {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
            bool oldApplyJitter = m_applyJitter;
            ImGui::Checkbox("Pixel Jitter", &m_applyJitter);
            ImGui::SameLine();
            HelpMarker("Enable pixel jittering for better sampling of the scene");
            if (oldApplyJitter != m_applyJitter)
                m_shouldResetPathTracing = true;
        }

        // Only one light sample
        if (m_raytracingType != RaytracingType::Whitted) {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
            bool oldOnlyOneLightSample = m_onlyOneLightSample;
            ImGui::Checkbox("Single LightBuffer Sample", &m_onlyOneLightSample);
            ImGui::SameLine();
            HelpMarker("Whether light sampling should be done one at a time or all at once");
            if (oldOnlyOneLightSample != m_onlyOneLightSample)
                m_shouldResetPathTracing = true;
        }

        // Anisotropic BSDF
        // ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
        // ImGui::Checkbox("Anisotropic BSDF", &m_anisotropicBSDF);
        // ImGui::SameLine(); HelpMarker("Enable anisotropy model");

        // Ray Tracing Type
        const char*          options[]          = { "Whitted", "Path Tracing", "Progressive Path Tracing" };
        RaytracingType::Enum prevRaytracingType = m_raytracingType;
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
        if (ImGui::BeginCombo("Raytracing Type", options[m_raytracingType])) {
            bool selected = m_raytracingType == RaytracingType::Whitted;
            if (ImGui::Selectable("Whitted", selected))
                m_raytracingType = RaytracingType::Whitted;
            if (selected)
                ImGui::SetItemDefaultFocus();

            selected = m_raytracingType == RaytracingType::PathTracing;
            if (ImGui::Selectable("Path Tracing", selected))
                m_raytracingType = RaytracingType::PathTracing;
            if (selected)
                ImGui::SetItemDefaultFocus();

            selected = m_raytracingType == RaytracingType::PathTracingTemporal;
            if (ImGui::Selectable("Progressive Path Tracing", selected))
                m_raytracingType = RaytracingType::PathTracingTemporal;
            if (selected)
                ImGui::SetItemDefaultFocus();

            ImGui::EndCombo();
        }
        ImGui::SameLine();
        HelpMarker("Select the raytracing type to use");

        // Trigger a graphics reload if the raytracing type has changed.
        if (prevRaytracingType != m_raytracingType)
            m_shouldReload = true;

        // Importance Sampling type
        if (m_raytracingType != RaytracingType::Whitted) {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
            UINT        oldImportanceSamplingType   = m_importanceSamplingType;
            const char* importanceSamplingOptions[] = { "Uniform", "Cosine", "BSDF" };
            if (ImGui::BeginCombo("Importance Sampling", importanceSamplingOptions[m_importanceSamplingType])) {
                bool selected = m_importanceSamplingType == ImportanceSamplingType::Uniform;
                if (ImGui::Selectable("Uniform Sphere", selected))
                    m_importanceSamplingType = ImportanceSamplingType::Uniform;
                if (selected)
                    ImGui::SetItemDefaultFocus();

                selected = m_importanceSamplingType == ImportanceSamplingType::Cosine;
                if (ImGui::Selectable("Cosine Hemisphere", selected))
                    m_importanceSamplingType = ImportanceSamplingType::Cosine;
                if (selected)
                    ImGui::SetItemDefaultFocus();

                selected = m_importanceSamplingType == ImportanceSamplingType::BSDF;
                if (ImGui::Selectable("BSDF", selected))
                    m_importanceSamplingType = ImportanceSamplingType::BSDF;
                if (selected)
                    ImGui::SetItemDefaultFocus();

                ImGui::EndCombo();
            }
            ImGui::SameLine();
            HelpMarker("Select the importance sampling type to use");
            if (oldImportanceSamplingType != m_importanceSamplingType)
                m_shouldResetPathTracing = true;
        }

        // Sqrt Samples per pixel
        if (m_raytracingType == RaytracingType::PathTracing) {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
            ImGui::SliderInt("Sqrt Samples per pixel", reinterpret_cast<int*>(&m_pathSqrtSamplesPerPixel), 1, 4);
            ImGui::SameLine();
            HelpMarker("Sqrt of number of samples per pixel");
        }

        // Max recursion
        UINT oldMaxRecursionDepth = m_maxRecursionDepth;
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
        ImGui::SliderInt("Max Recursion Depth", reinterpret_cast<int*>(&m_maxRecursionDepth), 1, 10);
        ImGui::SameLine();
        HelpMarker("Maximum recursion depth for path tracing");
        // we need to reload graphics if this changed (because it was set in stone for optimization purposes)
        if (oldMaxRecursionDepth != m_maxRecursionDepth)
            m_shouldReload = true;

        // Max shadow recursion depth
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
        UINT oldMaxShadowRecursionDepth = m_maxShadowRecursionDepth;
        ImGui::SliderInt("Max Shadow Recursion Depth", reinterpret_cast<int*>(&m_maxShadowRecursionDepth), 1, 10);
        ImGui::SameLine();
        HelpMarker("Maximum recursion depth for shooting shadow rays");
        if (oldMaxShadowRecursionDepth != m_maxShadowRecursionDepth)
            m_shouldResetPathTracing = true;

        // Russian Roulette
        if (m_raytracingType != RaytracingType::Whitted) {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
            UINT oldRussianRouletteDepth = m_russianRouletteDepth;
            ImGui::SliderInt("Russian Roulette Depth", reinterpret_cast<int*>(&m_russianRouletteDepth), 1, 10);
            ImGui::SameLine();
            HelpMarker("Depth at which to start Russian Roulette");
            if (oldRussianRouletteDepth != m_russianRouletteDepth)
                m_shouldResetPathTracing = true;
        }

        ImGui::Spacing();
    }

    if (ImGui::CollapsingHeader("Scene")) {
        ImGui::Spacing();

        // Change scene
        const char* sceneOptions[] = { "Cornell Box", "Demo", "PBR Showcase" };
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
        if (ImGui::BeginCombo("Select Scene", sceneOptions[m_nextScene])) {
            bool selected = m_nextScene == SceneTypes::CornellBox;
            if (ImGui::Selectable("Cornell Box", selected))
                m_nextScene = SceneTypes::CornellBox;
            if (selected)
                ImGui::SetItemDefaultFocus();

            selected = m_nextScene == SceneTypes::Demo;
            if (ImGui::Selectable("Demo", selected))
                m_nextScene = SceneTypes::Demo;
            if (selected)
                ImGui::SetItemDefaultFocus();

            selected = m_nextScene == SceneTypes::PbrShowcase;
            if (ImGui::Selectable("PBR Showcase", selected))
                m_nextScene = SceneTypes::PbrShowcase;
            if (selected)
                ImGui::SetItemDefaultFocus();

            ImGui::EndCombo();
        }
        ImGui::SameLine();
        HelpMarker("Select the scene to render");

        // Trigger a graphics reload if the scene has changed.
        if (m_nextScene != m_crtScene)
            m_shouldReload = true;

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Background color
        if (m_crtScene != SceneTypes::PbrShowcase) {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
            XMFLOAT4 oldBackgroundColor = m_backgroundColor;
            ImGui::ColorEdit3("Background Color", &m_backgroundColor.x);
            if (oldBackgroundColor.x != m_backgroundColor.x || oldBackgroundColor.y != m_backgroundColor.y
                    || oldBackgroundColor.z != m_backgroundColor.z)
                m_shouldResetPathTracing = true;
        }

        // Lights
        for (UINT i = 0; i < m_scenes[m_crtScene].GetLightCount(); ++i) {
            ImGui::PushID(i);
            ImGui::Text("LIGHT %u", i);
            ImGui::Spacing();

            // Light type
            if (m_crtScene == SceneTypes::Demo) {
                const char* lightTypeOptions[] = { "Area", "Directional" };
                UINT        prevLightType      = m_scenes[m_crtScene].m_lights[i].type;
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
                if (ImGui::BeginCombo("Type", lightTypeOptions[m_scenes[m_crtScene].m_lights[i].type])) {
                    for (UINT j = 0; j < LightType::Count; ++j) {
                        bool selected = m_scenes[m_crtScene].m_lights[i].type == j;
                        if (ImGui::Selectable(lightTypeOptions[j], selected))
                            m_scenes[m_crtScene].m_lights[i].type = j;
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }

                // Trigger a graphics reload if the light type has changed.
                if (prevLightType != m_scenes[m_crtScene].m_lights[i].type)
                    m_shouldReload = true;
            }

            if (m_crtScene != SceneTypes::PbrShowcase && m_scenes[m_crtScene].m_lights[i].type == LightType::Square) {
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
                float       lastSize = m_scenes[m_crtScene].m_lights[i].size;
                const float maxSize  = m_crtScene == SceneTypes::Demo ? 10.0f : 5.0f;
                ImGui::SliderFloat("Size", &m_scenes[m_crtScene].m_lights[i].size, 0.1f, maxSize);
                if (lastSize != m_scenes[m_crtScene].m_lights[i].size) {
                    m_shouldRebuildAccelerationStructures = true;
                    m_shouldResetPathTracing              = true;
                }

                if (m_crtScene == SceneTypes::Demo) {
                    XMFLOAT3 lastPosition = m_scenes[m_crtScene].m_lights[i].position;
                    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
                    ImGui::SliderFloat3("Position", &m_scenes[m_crtScene].m_lights[i].position.x, -20.0f, 20.0f);
                    if (lastPosition.x != m_scenes[m_crtScene].m_lights[i].position.x
                            || lastPosition.y != m_scenes[m_crtScene].m_lights[i].position.y
                            || lastPosition.z != m_scenes[m_crtScene].m_lights[i].position.z) {
                        m_shouldRebuildAccelerationStructures = true;
                        m_shouldResetPathTracing              = true;
                    }
                }
            }

            if (m_scenes[m_crtScene].m_lights[i].type == LightType::Directional) {
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
                XMFLOAT3 lastDirection = m_scenes[m_crtScene].m_lights[i].direction;
                ImGui::SliderFloat3("Direction", &m_scenes[m_crtScene].m_lights[i].direction.x, -1.0f, 1.0f);
                if (lastDirection.x != m_scenes[m_crtScene].m_lights[i].direction.x
                        || lastDirection.y != m_scenes[m_crtScene].m_lights[i].direction.y
                        || lastDirection.z != m_scenes[m_crtScene].m_lights[i].direction.z)
                    m_shouldResetPathTracing = true;
            }

            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
            float lastIntensity = m_scenes[m_crtScene].m_lights[i].intensity;
            ImGui::SliderFloat("Intensity", &m_scenes[m_crtScene].m_lights[i].intensity, 0.0f, 10.0f);
            if (lastIntensity != m_scenes[m_crtScene].m_lights[i].intensity)
                m_shouldResetPathTracing = true;

            if (m_crtScene != SceneTypes::PbrShowcase) {
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 16);
                XMFLOAT3 lastEmission = m_scenes[m_crtScene].m_lights[i].emission;
                ImGui::ColorEdit3("Emission", &m_scenes[m_crtScene].m_lights[i].emission.x);
                if (lastEmission.x != m_scenes[m_crtScene].m_lights[i].emission.x
                        || lastEmission.y != m_scenes[m_crtScene].m_lights[i].emission.y
                        || lastEmission.z != m_scenes[m_crtScene].m_lights[i].emission.z)
                    m_shouldResetPathTracing = true;
            }

            if (i < m_scenes[m_crtScene].GetLightCount() - 1)
                ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::PopID();
        }

        ImGui::Spacing();
    }

    ImGui::End();

    // Reset path tracing if needed.
    if (m_shouldResetPathTracing)
        ResetPathTracing();
}

void DieXaR::HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

// Update the application state with the new resolution.
void DieXaR::UpdateForSizeChange(UINT width, UINT height)
{
    DXSample::UpdateForSizeChange(width, height);
    m_renderWidth  = static_cast<UINT>(width / m_upscaleRatio);
    m_renderHeight = static_cast<UINT>(height / m_upscaleRatio);
}

// Copy the raytracing output to the backbuffer.
void DieXaR::CopyFinalOutputToBackbuffer()
{
    auto commandList  = m_deviceResources->GetCommandList();
    auto renderTarget = m_deviceResources->GetRenderTarget();

    D3D12_RESOURCE_BARRIER preCopyBarriers[2];
    preCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(
            renderTarget, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
    // Copy from the new upscaled resource
    preCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(
            m_upscaledOutput.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    commandList->ResourceBarrier(ARRAYSIZE(preCopyBarriers), preCopyBarriers);

    commandList->CopyResource(renderTarget, m_upscaledOutput.Get());

    D3D12_RESOURCE_BARRIER postCopyBarriers[2];
    postCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(
            renderTarget, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT);
    // Transition the upscaled resource back to UAV state
    postCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(
            m_upscaledOutput.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    commandList->ResourceBarrier(ARRAYSIZE(postCopyBarriers), postCopyBarriers);
}

// Create resources that are dependent on the size of the main window.
void DieXaR::CreateWindowSizeDependentResources()
{
    // The main raytracing output is now at render resolution
    auto device           = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();

    auto uavDesc = CD3DX12_RESOURCE_DESC::Tex2D(
            backbufferFormat, m_renderWidth, m_renderHeight, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_raytracingOutput)));
    NAME_D3D12_OBJECT(m_raytracingOutput);

    // Create a descriptor for the raytracing output
    D3D12_CPU_DESCRIPTOR_HANDLE rtUavDescriptorHandle;
    m_raytracingOutputResourceUAVDescriptorHeapIndex
            = AllocateDescriptor(&rtUavDescriptorHandle, m_raytracingOutputResourceUAVDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC rtUAVDesc = {};
    rtUAVDesc.ViewDimension                    = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_raytracingOutput.Get(), nullptr, &rtUAVDesc, rtUavDescriptorHandle);
    m_raytracingOutputResourceUAVGpuDescriptor
            = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(),
                    m_raytracingOutputResourceUAVDescriptorHeapIndex, m_descriptorSize);

    // Create the resource for FSR's upscaled output at display resolution
    auto upscaleUavDesc = CD3DX12_RESOURCE_DESC::Tex2D(
            backbufferFormat, m_width, m_height, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &upscaleUavDesc,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_upscaledOutput)));
    NAME_D3D12_OBJECT(m_upscaledOutput);

    // Create a descriptor for the upscaled output
    D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
    m_upscaledOutputDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, m_upscaledOutputDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC UAVDesc = {};
    UAVDesc.ViewDimension                    = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_upscaledOutput.Get(), nullptr, &UAVDesc, uavDescriptorHandle);
    m_upscaledOutputGpuDescriptor
            = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(),
                    m_upscaledOutputDescriptorHeapIndex, m_descriptorSize);

    // Create the World Position buffer
    auto worldPosUavDesc
            = CD3DX12_RESOURCE_DESC::Tex2D(DXGI_FORMAT_R32G32B32A32_FLOAT,  // High precision for world coordinates
                    m_renderWidth, m_renderHeight, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &worldPosUavDesc,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_worldPositionOutput)));
    NAME_D3D12_OBJECT(m_worldPositionOutput);

    D3D12_CPU_DESCRIPTOR_HANDLE worldPosUavHandle;
    m_worldPositionOutputUAVDescriptorHeapIndex
            = AllocateDescriptor(&worldPosUavHandle, m_worldPositionOutputUAVDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC worldPosUAVDesc = {};
    worldPosUAVDesc.ViewDimension                    = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_worldPositionOutput.Get(), nullptr, &worldPosUAVDesc, worldPosUavHandle);
    m_worldPositionOutputResourceUAVGpuDescriptor
            = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(),
                    m_worldPositionOutputUAVDescriptorHeapIndex, m_descriptorSize);


    // Create the Motion Vector buffer
    auto motionVecUavDesc
            = CD3DX12_RESOURCE_DESC::Tex2D(DXGI_FORMAT_R16G16_FLOAT,  // R16G16 is usually sufficient for motion vectors
                    m_renderWidth, m_renderHeight, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &motionVecUavDesc,
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, nullptr, IID_PPV_ARGS(&m_motionVectorOutput)));
    NAME_D3D12_OBJECT(m_motionVectorOutput);

    D3D12_CPU_DESCRIPTOR_HANDLE motionVecUavHandle;
    m_motionVectorOutputUAVDescriptorHeapIndex
            = AllocateDescriptor(&motionVecUavHandle, m_motionVectorOutputUAVDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC motionVecUAVDesc = {};
    motionVecUAVDesc.ViewDimension                    = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_motionVectorOutput.Get(), nullptr, &motionVecUAVDesc, motionVecUavHandle);
    m_motionVectorOutputResourceUAVGpuDescriptor
            = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(),
                    m_motionVectorOutputUAVDescriptorHeapIndex, m_descriptorSize);

    UpdateCameraMatrices();
}

// Release resources that are dependent on the size of the main window.
void DieXaR::ReleaseWindowSizeDependentResources()
{
    m_raytracingOutput.Reset();
}

// Release all resources that depend on the device.
void DieXaR::ReleaseDeviceDependentResources()
{
    for (auto& gpuTimer : m_gpuTimers) {
        gpuTimer.ReleaseDevice();
    }

    m_raytracingGlobalRootSignature.Reset();
    m_visualizeMV_RootSignature.Reset();
    m_visualizeMV_PSO.Reset();
    m_motionVectorRootSignature.Reset();
    m_motionVectorPSO.Reset();
    ResetComPtrArray(&m_raytracingLocalRootSignature);

    m_dxrDevice.Reset();
    m_dxrCommandList.Reset();
    m_dxrStateObject.Reset();

    m_raytracingGlobalRootSignature.Reset();
    ResetComPtrArray(&m_raytracingLocalRootSignature);

    m_descriptorHeap.Reset();
    m_descriptorsAllocated = 0;
    m_sceneCB.Release();
    m_lights.Release();
    m_aabbPrimitiveAttributeBuffer.Release();
    m_indexBuffer.resource.Reset();
    m_vertexBuffer.resource.Reset();
    m_aabbBuffer.resource.Reset();

    ResetComPtrArray(&m_bottomLevelAS);
    m_topLevelAS.Reset();

    m_raytracingOutput.Reset();
    m_raytracingOutputResourceUAVDescriptorHeapIndex = UINT_MAX;
    m_rayGenShaderTable.Reset();
    m_missShaderTable.Reset();
    m_hitGroupShaderTable.Reset();
}

void DieXaR::RecreateD3D()
{
    // Give GPU a chance to finish its execution in progress.
    try {
        m_deviceResources->WaitForGpu();
    }
    catch (HrException&) {
        // Do nothing, currently attached adapter is unresponsive.
    }
    m_deviceResources->HandleDeviceLost();
}

// Render the scene.
void DieXaR::OnRender()
{
    if (!m_deviceResources->IsWindowVisible()) {
        return;
    }

    auto device      = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    // Begin frame.
    m_deviceResources->Prepare();
    for (auto& gpuTimer : m_gpuTimers) {
        gpuTimer.BeginFrame(commandList);
    }

    DoRaytracing();
    DispatchMotionVectorPass(commandList);  // Must be called after DoRaytracing and before FSR/Visualization
    if (m_visualizeMotionVectors) {
        DispatchVisualizeMVPass(commandList);
        commandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
    }
    else {
        DispatchFsrUpscale(commandList);
        commandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
    }

    // Copy the final result (either the visualization or the upscaled image) to the backbuffer
    CopyFinalOutputToBackbuffer();

    // Render ImGui.
    ImGui::Render();

    // Transition the render target to the render target state.
    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Flags                  = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    barrier.Transition.pResource   = m_deviceResources->GetRenderTarget();
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
    barrier.Transition.StateAfter  = D3D12_RESOURCE_STATE_RENDER_TARGET;
    commandList->ResourceBarrier(1, &barrier);

    // Render the ImGui draw data.
    commandList->OMSetRenderTargets(1, &m_deviceResources->GetRenderTargetView(), FALSE, nullptr);
    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), m_deviceResources->GetCommandList());

    // Transition the render target to the present state.
    barrier = CD3DX12_RESOURCE_BARRIER::Transition(
            m_deviceResources->GetRenderTarget(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
    commandList->ResourceBarrier(1, &barrier);

    // End frame.
    for (auto& gpuTimer : m_gpuTimers) {
        gpuTimer.EndFrame(commandList);
    }

    m_deviceResources->Present(D3D12_RESOURCE_STATE_PRESENT);
}

void DieXaR::OnDestroy()
{
    // Let GPU finish before releasing D3D resources.
    m_deviceResources->WaitForGpu();

    if (m_shouldReload) {
        // Restart ImGui.
        ImGui_ImplDX12_Shutdown();
        ImGui_ImplWin32_Shutdown();
        ImGui::DestroyContext();
    }

    DestroyFSR3();

    OnDeviceLost();
}


void DieXaR::CreateVisualizeMV_PSO()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Create a root signature for the visualization shader.
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[2];
        // range[0] is for the input motion vector SRV (t0)
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);
        // range[1] is for the output texture UAV (u0)
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);

        CD3DX12_ROOT_PARAMETER rootParameters[2];
        // Slot 0: SRV for motion vector texture
        rootParameters[0].InitAsDescriptorTable(1, &ranges[0]);
        // Slot 1: UAV for output texture
        rootParameters[1].InitAsDescriptorTable(1, &ranges[1]);

        CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        SerializeAndCreateRaytracingRootSignature(rootSignatureDesc, &m_visualizeMV_RootSignature);
        NAME_D3D12_OBJECT(m_visualizeMV_RootSignature);
    }

    // Create the compute PSO.
    {
        D3D12_COMPUTE_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.pRootSignature                    = m_visualizeMV_RootSignature.Get();
        psoDesc.CS = CD3DX12_SHADER_BYTECODE(g_pVisualizeMotionVectors, ARRAYSIZE(g_pVisualizeMotionVectors));

        ThrowIfFailed(device->CreateComputePipelineState(&psoDesc, IID_PPV_ARGS(&m_visualizeMV_PSO)));
        NAME_D3D12_OBJECT(m_visualizeMV_PSO);
    }
}

void DieXaR::CreateMotionVectorPSO()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Create the root signature for the motion vector compute shader.
    // This defines how resources are bound to the shader.
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[2];
        // range[0] is for the input world position SRV (t0)
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);
        // range[1] is for the output motion vector UAV (u0)
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);

        CD3DX12_ROOT_PARAMETER rootParameters[3];
        // Slot 0: Scene constant buffer (b0)
        rootParameters[0].InitAsConstantBufferView(0);
        // Slot 1: SRV for world position texture (t0)
        rootParameters[1].InitAsDescriptorTable(1, &ranges[0]);
        // Slot 2: UAV for motion vector texture (u0)
        rootParameters[2].InitAsDescriptorTable(1, &ranges[1]);

        CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        SerializeAndCreateRaytracingRootSignature(rootSignatureDesc, &m_motionVectorRootSignature);
        NAME_D3D12_OBJECT(m_motionVectorRootSignature);
    }

    // Create the compute pipeline state object (PSO).
    {
        D3D12_COMPUTE_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.pRootSignature                    = m_motionVectorRootSignature.Get();
        psoDesc.CS = CD3DX12_SHADER_BYTECODE(g_pMotionVector, ARRAYSIZE(g_pMotionVector));

        ThrowIfFailed(device->CreateComputePipelineState(&psoDesc, IID_PPV_ARGS(&m_motionVectorPSO)));
        NAME_D3D12_OBJECT(m_motionVectorPSO);
    }
}

void DieXaR::DispatchMotionVectorPass(ID3D12GraphicsCommandList* commandList)
{
    // Transition resources
    D3D12_RESOURCE_BARRIER barriers[2];
    barriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(m_worldPositionOutput.Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    barriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_motionVectorOutput.Get(),
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    commandList->ResourceBarrier(2, barriers);

    // Set PSO and Root Signature for the motion vector compute shader
    commandList->SetPipelineState(m_motionVectorPSO.Get());
    commandList->SetComputeRootSignature(m_motionVectorRootSignature.Get());

    // Bind resources (you will define these slots in your new root signature)
    commandList->SetComputeRootConstantBufferView(
            0, m_sceneCB.GpuVirtualAddress(m_deviceResources->GetCurrentFrameIndex()));
    commandList->SetComputeRootDescriptorTable(
            1, m_worldPositionOutputResourceUAVGpuDescriptor);  // Or an SRV descriptor
    commandList->SetComputeRootDescriptorTable(2, m_motionVectorOutputResourceUAVGpuDescriptor);

    // Dispatch the compute shader
    UINT numGroupsX = (m_renderWidth + 7) / 8;
    UINT numGroupsY = (m_renderHeight + 7) / 8;
    commandList->Dispatch(numGroupsX, numGroupsY, 1);

    // Transition motion vector output to be read by FSR
    D3D12_RESOURCE_BARRIER finalBarriers[2];
    finalBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(m_motionVectorOutput.Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    finalBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_worldPositionOutput.Get(),
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    commandList->ResourceBarrier(2, finalBarriers);
}


void DieXaR::DispatchVisualizeMVPass(ID3D12GraphicsCommandList* commandList)
{
    // The motion vector buffer is already in a read state from the previous pass.
    // We just need to ensure our output buffer is in a write state.
    auto barrier = CD3DX12_RESOURCE_BARRIER::UAV(m_upscaledOutput.Get());
    commandList->ResourceBarrier(1, &barrier);

    commandList->SetPipelineState(m_visualizeMV_PSO.Get());
    commandList->SetComputeRootSignature(m_visualizeMV_RootSignature.Get());

    // Bind resources
    commandList->SetComputeRootDescriptorTable(0, m_motionVectorOutputGpuDescriptor);  // input
    commandList->SetComputeRootDescriptorTable(1, m_upscaledOutputGpuDescriptor);      // output

    // Dispatch to cover the full display resolution
    UINT numGroupsX = (m_width + 7) / 8;
    UINT numGroupsY = (m_height + 7) / 8;
    commandList->Dispatch(numGroupsX, numGroupsY, 1);
}


void DieXaR::InitializeFSR3()
{
    // 1. Create FFX Backend
    ffx::CreateBackendDX12Desc backendDesc{};
    backendDesc.device = m_deviceResources->GetD3DDevice();

    // 2. Create Upscaling Context
    ffx::CreateContextDescUpscale createUpscaling{};
    createUpscaling.maxUpscaleSize = { m_width, m_height };
    createUpscaling.maxRenderSize  = { m_renderWidth, m_renderHeight };

    // Configure FSR flags
    createUpscaling.flags = FFX_UPSCALE_ENABLE_AUTO_EXPOSURE;
    createUpscaling.flags |= FFX_UPSCALE_ENABLE_HIGH_DYNAMIC_RANGE;

    // Assuming you are using an inverted infinite depth projection like most modern engines
    createUpscaling.flags |= FFX_UPSCALE_ENABLE_DEPTH_INVERTED | FFX_UPSCALE_ENABLE_DEPTH_INFINITE;

    ffx::ReturnCode retCode = ffx::CreateContext(m_upscalingContext, nullptr, createUpscaling, backendDesc);
    if (retCode != ffx::ReturnCode::Ok) {
        ThrowIfFalse(false, L"FFX Upscaling context creation failed.");
    }

    m_resetUpscale = true;  // Ensure history is reset on first frame
}


void DieXaR::DispatchFsrUpscale(ID3D12GraphicsCommandList* commandList)
{
    if (!m_upscalingContext)
        return;

    // This barrier ensures the output resource is in the correct state for writing.
    // Note: We don't need to transition the inputs because the FFX API handles that internally.
    auto barrier = CD3DX12_RESOURCE_BARRIER::UAV(m_upscaledOutput.Get());
    commandList->ResourceBarrier(1, &barrier);

    // Fill the dispatch description
    ffx::DispatchDescUpscale dispatchUpscale{};
    dispatchUpscale.commandList = commandList;
    dispatchUpscale.color       = ffxApiGetResourceDX12(
            m_raytracingOutput.Get(), FFX_API_RESOURCE_STATE_UNORDERED_ACCESS, FFX_API_RESOURCE_USAGE_UAV);
    dispatchUpscale.depth
            = ffxApiGetResourceDX12(m_deviceResources->GetDepthStencil(), FFX_API_RESOURCE_STATE_PIXEL_COMPUTE_READ);
    dispatchUpscale.motionVectors
            = ffxApiGetResourceDX12(m_motionVectorOutput.Get(), FFX_API_RESOURCE_STATE_PIXEL_COMPUTE_READ);
    dispatchUpscale.output = ffxApiGetResourceDX12(
            m_upscaledOutput.Get(), FFX_API_RESOURCE_STATE_UNORDERED_ACCESS, FFX_API_RESOURCE_USAGE_UAV);

    dispatchUpscale.renderSize  = { m_renderWidth, m_renderHeight };
    dispatchUpscale.upscaleSize = { m_width, m_height };

    dispatchUpscale.jitterOffset      = { -m_jitterX, -m_jitterY };
    dispatchUpscale.motionVectorScale = { (float) m_renderWidth, (float) m_renderHeight };

    dispatchUpscale.reset          = m_resetUpscale;
    dispatchUpscale.frameTimeDelta = static_cast<float>(m_timer.GetElapsedSeconds() * 1000.0);
    dispatchUpscale.preExposure    = 1.0f;  // Set your scene's exposure value here

    dispatchUpscale.cameraFovAngleVertical = XMConvertToRadians(45.0f);
    dispatchUpscale.cameraNear             = 0.01f;
    dispatchUpscale.cameraFar              = 125.0f;

    dispatchUpscale.enableSharpening = m_enableSharpening;
    dispatchUpscale.sharpness        = m_sharpness;

    ffx::ReturnCode retCode = ffx::Dispatch(m_upscalingContext, dispatchUpscale);
    if (retCode != ffx::ReturnCode::Ok) {
        OutputDebugString(L"FSR Dispatch Failed\n");
    }

    // After dispatch, reset the flag
    m_resetUpscale = false;
}

void DieXaR::DestroyFSR3()
{
    if (m_upscalingContext) {
        ffx::DestroyContext(m_upscalingContext);
        m_upscalingContext = nullptr;
    }
}

// Release all device dependent resouces when a device is lost.
void DieXaR::OnDeviceLost()
{
    ReleaseWindowSizeDependentResources();
    ReleaseDeviceDependentResources();
}

// Create all device dependent resources when a device is restored.
void DieXaR::OnDeviceRestored()
{
    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();
}

// Compute the average frames per second and million rays per second.
void DieXaR::CalculateFrameStats()
{
    static int    frameCnt  = 0;
    static double prevTime  = 0.0f;
    double        totalTime = m_timer.GetTotalSeconds();

    frameCnt++;

    // Compute averages over one second period.
    if ((totalTime - prevTime) >= 1.0f) {
        float diff = static_cast<float>(totalTime - prevTime);
        float fps  = static_cast<float>(frameCnt) / diff;  // Normalize to an exact second.

        frameCnt             = 0;
        prevTime             = totalTime;
        float raytracingTime = static_cast<float>(m_gpuTimers[GpuTimers::Raytracing].GetAverageMS());
        float MRaysPerSecond = NumMRaysPerSecond(m_width, m_height, raytracingTime);

        wstringstream windowText;
        windowText << setprecision(2) << fixed << L"    fps: " << fps << L"    DispatchRays(): " << raytracingTime
                   << "ms" << L"     ~Million Primary Rays/s: " << MRaysPerSecond << L"    GPU["
                   << m_deviceResources->GetAdapterID() << L"]: " << m_deviceResources->GetAdapterDescription();
        SetCustomWindowText(windowText.str().c_str());
    }
}

// Handle OnSizeChanged message event.
void DieXaR::OnSizeChanged(UINT width, UINT height, bool minimized)
{
    if (!m_deviceResources->WindowSizeChanged(width, height, minimized)) {
        return;
    }

    UpdateForSizeChange(width, height);
    m_shouldReload = true;
}

// Allocate a descriptor and return its index.
// If the passed descriptorIndexToUse is valid, it will be used instead of allocating a new one.
UINT DieXaR::AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse)
{
    auto descriptorHeapCpuBase = m_descriptorHeap->GetCPUDescriptorHandleForHeapStart();
    if (descriptorIndexToUse >= m_descriptorHeap->GetDesc().NumDescriptors) {
        ThrowIfFalse(m_descriptorsAllocated < m_descriptorHeap->GetDesc().NumDescriptors,
                L"Ran out of descriptors on the heap!");
        descriptorIndexToUse = m_descriptorsAllocated++;
    }
    *cpuDescriptor = CD3DX12_CPU_DESCRIPTOR_HANDLE(descriptorHeapCpuBase, descriptorIndexToUse, m_descriptorSize);
    return descriptorIndexToUse;
}

// Create a SRV for a buffer.
UINT DieXaR::CreateBufferSRV(D3DBuffer* buffer, UINT numElements, UINT elementSize)
{
    auto device = m_deviceResources->GetD3DDevice();

    // SRV
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension                   = D3D12_SRV_DIMENSION_BUFFER;
    srvDesc.Shader4ComponentMapping         = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Buffer.NumElements              = numElements;
    if (elementSize == 0) {
        srvDesc.Format                     = DXGI_FORMAT_R32_TYPELESS;
        srvDesc.Buffer.Flags               = D3D12_BUFFER_SRV_FLAG_RAW;
        srvDesc.Buffer.StructureByteStride = 0;
    }
    else {
        srvDesc.Format                     = DXGI_FORMAT_UNKNOWN;
        srvDesc.Buffer.Flags               = D3D12_BUFFER_SRV_FLAG_NONE;
        srvDesc.Buffer.StructureByteStride = elementSize;
    }
    UINT descriptorIndex = AllocateDescriptor(&buffer->cpuDescriptorHandle);
    device->CreateShaderResourceView(buffer->resource.Get(), &srvDesc, buffer->cpuDescriptorHandle);
    buffer->gpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(
            m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), descriptorIndex, m_descriptorSize);
    return descriptorIndex;
}
