//*********************************************************
//
// MotionVector.hlsl
//
//
//*********************************************************

#define HLSL
#include "RaytracingHlslCompat.h"

// The SceneConstantBuffer contains the current and previous camera matrices.
// It is bound as a root Constant Buffer View (CBV).
cbuffer SceneConstantBuffer : register(b0)
{
    float4x4 worldToProjection;
    float4x4 previousWorldToProjection;
    // We only need the matrices, so the rest of the struct is omitted for clarity.
};

// Input texture containing the world position for each pixel from the ray tracing pass.
// Bound as a Shader Resource View (SRV).
Texture2D<float4> g_WorldPositionInput : register(t0);

// Output texture for the calculated motion vectors.
// Bound as an Unordered Access View (UAV).
RWTexture2D<float2> g_MotionVectorOutput : register(u0);


// The main function for the compute shader.
[numthreads(8, 8, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    // Read the world position that was stored by the ray tracing pass.
    float4 worldPos = g_WorldPositionInput[dispatchThreadID.xy];

    // Check the 'w' component. If it's 0, the ray missed and hit the sky.
    // In this case, there is no motion, so we write a zero vector.
    if (worldPos.w == 0)
    {
        g_MotionVectorOutput[dispatchThreadID.xy] = float2(0.0f, 0.0f);
        return;
    }

    // Project the 3D world position into the 4D clip space of the current and previous frames.
    float4 currentClipPos = mul(worldPos, worldToProjection);
    float4 previousClipPos = mul(worldPos, previousWorldToProjection);

    // Perform the perspective divide to get Normalized Device Coordinates (NDC) [-1, 1].
    float2 currentNdc = currentClipPos.xy / currentClipPos.w;
    float2 previousNdc = previousClipPos.xy / previousClipPos.w;
    
    // The motion vector is the difference in screen position from the last frame to this one.
    // FSR expects the motion from the current pixel to where it was in the previous frame.
    float2 motionVector = previousNdc - currentNdc;

    // The Y-coordinate in NDC space is typically opposite to screen/texture space.
    motionVector.y = -motionVector.y;

    // Write the final 2D motion vector to the output texture.
    g_MotionVectorOutput[dispatchThreadID.xy] = motionVector;
}