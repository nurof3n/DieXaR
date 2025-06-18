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

#include "RayTracingHlslCompat.h"

namespace GlobalRootSignature {
	namespace Slot {
		enum Enum {
			OutputView = 0,
			AccelerationStructure,
			SceneConstant,
            WorldPositionOutput,
			AABBattributeBuffer,
			VertexBuffers,
			LightBuffer,
			Count
		};
	}
}

namespace LocalRootSignature {
	namespace Type {
		enum Enum {
			Triangle = 0,
			AABB,
			Count
		};
	}
}

namespace LocalRootSignature {
	namespace Triangle {
		namespace Slot {
			enum Enum {
				MaterialConstant = 0,
				GeometryIndex,
				PbrConstant,
				Count
			};
		}
		struct RootArguments {
			PrimitiveConstantBuffer materialCb;
			PrimitiveInstanceConstantBuffer primitiveCB;
			PBRPrimitiveConstantBuffer pbrCb;
		};
	}
}

namespace LocalRootSignature {
	namespace AABB {
		namespace Slot {
			enum Enum {
				MaterialConstant = 0,
				GeometryIndex,
				PbrConstant,
				Count
			};
		}
		struct RootArguments {
			PrimitiveConstantBuffer materialCb;
			PrimitiveInstanceConstantBuffer primitiveCB;
			PBRPrimitiveConstantBuffer pbrCb;
		};
	}
}

namespace LocalRootSignature {
	inline UINT MaxRootArgumentsSize()
	{
		return max(sizeof(Triangle::RootArguments), sizeof(AABB::RootArguments));
	}
}

namespace GeometryType {
	enum Enum {
		Triangle = 0,
		AABB,       // Procedural geometry with an application provided AABB.
		Count
	};
}

namespace GpuTimers {
	enum Enum {
		Raytracing = 0,
		Count
	};
}

// Bottom-level acceleration structures (BottomLevelASType).
// This sample uses two BottomLevelASType, one for AABB and one for Triangle geometry.
// Mixing of geometry types within a BLAS is not supported.
namespace BottomLevelASType = GeometryType;

namespace IntersectionShaderType {
	enum Enum {
		AnalyticPrimitive = 0,
		SignedDistancePrimitive,
		Count
	};
}
