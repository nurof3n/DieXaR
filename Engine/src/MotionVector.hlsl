//*********************************************************
//
// MotionVector.hlsl
//
//*********************************************************

#define HLSL
#include "RaytracingHlslCompat.h"

// Use the full, correct SceneConstantBuffer definition from the include file.
// It is bound as a root Constant Buffer View (CBV) at register b0.
ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);

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
    // Access the matrices from the correctly defined constant buffer.
    float4 currentClipPos = mul(worldPos, g_sceneCB.worldToProjection);
    float4 previousClipPos = mul(worldPos, g_sceneCB.previousWorldToProjection);

    // Perform the perspective divide to get Normalized Device Coordinates (NDC) [-1, 1].
    float2 currentNdc = currentClipPos.xy / currentClipPos.w;
    float2 previousNdc = previousClipPos.xy / previousClipPos.w;

    // Calculate the final motion vector in NDC space.
    // This represents the motion from the current pixel to where its content was in the previous frame.
    float2 motionVector = previousNdc - currentNdc;

    // Write the final UV-space motion vector to the output texture.
    g_MotionVectorOutput[dispatchThreadID.xy] = motionVector;
}