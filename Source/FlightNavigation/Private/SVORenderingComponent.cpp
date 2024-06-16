// Fill out your copyright notice in the Description page of Project Settings.


#include "SVORenderingComponent.h"

#include "MaterialDomain.h"
#include "SparseVoxelOctreeTypes.h"

static constexpr int32 NUM_CUBE_VERTS = 12;
static constexpr int32 NUM_CUBE_TRIANGLES = 12;
static constexpr int32 NUM_CUBE_INDICES = NUM_CUBE_TRIANGLES * 3;

namespace FSVORenderingHelpers
{
	struct FSVOBox
	{
		FVector Centre;
		FCoord Extent;
		FColor Colour;
	
		FSVOBox(const FVector& Centre = FVector::ZeroVector, const float Extent = 0, const FColor& Colour = FColor::White):
			Centre(Centre), Extent(Extent), Colour(Colour)
		{}
		FSVOBox(const FVector& Centre, const FVector& Extent, const FColor& Colour):
			Centre(Centre), Extent(Extent.X), Colour(Colour)
		{}
	
		FSVOBox(const FBoxCenterAndExtent& Dimensions, const FColor& Colour):
			Centre(Dimensions.Center),
			Extent(Dimensions.Extent.X),
			Colour(Colour)
		{}
	};

	struct FVertex
	{
		FVector Pos;
		FVector2D TexCoord;

		FVertex(const FVector& Pos, const FVector2D& TexCoord): Pos(Pos), TexCoord(TexCoord) {}
	};
	
	struct FTriangleIndices
	{
		uint32 V0, V1, V2;
		FTriangleIndices(const uint32 V0, const uint32 V1, const uint32 V2): V0(V0), V1(V1), V2(V2) {}
	};
	
	// Tex coords require duplication of verts
	FVertex CubeVerts[] = {
		FVertex(FVector(-1.f,-1.f,-1.f),FVector2D(0.f, 0.f)), // 0
		FVertex(FVector(-1.f,-1.f, 1.f),FVector2D(1.f, 0.f)), // 1
		FVertex(FVector(-1.f, 1.f,-1.f),FVector2D(0.f, 1.f)), // 2
		FVertex(FVector(-1.f, 1.f, 1.f),FVector2D(1.f, 1.f)), // 3
	
		FVertex(FVector( 1.f,-1.f,-1.f),FVector2D(0.f, 1.f)), // 4
		FVertex(FVector( 1.f,-1.f, 1.f),FVector2D(1.f, 1.f)), // 5
		FVertex(FVector( 1.f, 1.f,-1.f),FVector2D(0.f, 0.f)), // 6
		FVertex(FVector( 1.f, 1.f, 1.f),FVector2D(1.f, 0.f)), // 7

		FVertex(FVector(-1.f, 1.f,-1.f),FVector2D(1.f, 0.f)), // 8
		FVertex(FVector(-1.f, 1.f, 1.f),FVector2D(0.f, 0.f)), // 9
		FVertex(FVector( 1.f, 1.f,-1.f),FVector2D(1.f, 1.f)), // 10
		FVertex(FVector( 1.f, 1.f, 1.f),FVector2D(0.f, 1.f))  // 11
	};

	FTriangleIndices CubeIndices[] = {
		// Back face
		FTriangleIndices(0, 1, 3),
		FTriangleIndices(0, 3, 2),
		// Left Face
		FTriangleIndices(5, 0, 4),
		FTriangleIndices(5, 1, 0),
		// Front face
		FTriangleIndices(5, 4, 6),
		FTriangleIndices(5, 6, 7),
	
		// Top Face
		FTriangleIndices(1, 5, 11),
		FTriangleIndices(1, 11, 9),
		// Right face
		FTriangleIndices(9, 11, 10),
		FTriangleIndices(9, 10, 8),
		// Bottom face
		FTriangleIndices(4, 0, 8),
		FTriangleIndices(4, 8, 10)
	};

	// Subclass for quick transform without rotation
	struct FTransformNoRot : FTransform
	{
		FORCEINLINE FTransformNoRot(const FVector& InTranslation, const FVector& InScale3D):
			FTransform(FQuat::Identity, InTranslation, InScale3D)
		{}
#if ENABLE_VECTORIZED_TRANSFORM
		FORCEINLINE FVector TransformPositionNoRotation(const FVector& V) const
		{
			DiagnosticCheckNaN_All();

			// UE5 Note: VectorRegister, VectorLoadFloat3_W0 etc. use doubles
			const VectorRegister InputVectorW0 = VectorLoadFloat3_W0(&V);
			const VectorRegister ScaledVec = VectorMultiply(Scale3D, InputVectorW0);
			const VectorRegister TranslatedVec = VectorAdd(ScaledVec, Translation);

			FVector Result;
			VectorStoreFloat3(TranslatedVec, &Result);
			return Result;
		}
#else // ENABLE_VECTORIZED_TRANSFORM
		FORCEINLINE FVector TransformPositionNoRotation(const FVector& V) const
		{
			DiagnosticCheckNaN_All();

			return Scale3D*V + Translation;
		}
#endif // ENABLE_VECTORIZED_TRANSFORM
	};
	
	inline bool HasFlag(int32 Flags, ESVODetailFlags TestFlag)
	{
		return (Flags & (1 << static_cast<int32>(TestFlag))) != 0;
	}
	
	int32 GetDetailFlags(const AFlightNavigationData* NavData)
	{
		if (NavData == nullptr)
		{
			return 0;
		}

		return (NavData->bDrawOctreeNodes ? (1 << static_cast<int32>(ESVODetailFlags::NodeBoxes)) : 0) |
				(NavData->bDrawOctreeSubNodes ? (1 << static_cast<int32>(ESVODetailFlags::SubNodeBoxes)) : 0) |
				(NavData->bDrawOnlyOverlappedSubNodes ? (1 << static_cast<int32>(ESVODetailFlags::OnlyOverlappedSubNodes)) : 0);
	}
}

void FSVOSceneProxyData::Reset()
{
	Vertices.Reset();
	Indices.Reset();
}

uint32 FSVOSceneProxyData::GetAllocatedSize() const
{
	return IntCastChecked<uint32>(
		Vertices.GetAllocatedSize() +
		Indices.GetAllocatedSize());
}

void FSVOSceneProxyData::GatherData(const AFlightNavigationData* NavData, int32 InNavDetailFlags)
{
	FRWScopeLock Lock(NavData->SVODataLock, SLT_ReadOnly);
	const FSVOData& SVOData = NavData->GetSVOData();
	
	Reset();

	const bool bShowNavigation = NavData != nullptr 
#if WITH_EDITORONLY_DATA
		&& (!NavData->GetWorld()->IsGameWorld() || NavData->bAllowDrawingInGameWorld)
#endif
	;
	
	if (bShowNavigation && InNavDetailFlags && SVOData.bValid)
	{
		TArray<FSVORenderingHelpers::FSVOBox> NodeBoxes;
		
		const FCoord LeafNodeSize = SVOData.GetNodeSizeForLayer(0);
		
		const FVector LeafExtent = FVector(LeafNodeSize * 0.5f);
		const FVector SubNodeExtent = SVOData.GetSubNodeExtent();
		
		const FCoord LeafOffset = SVOData.GetNodeOffsetForLayer(0); // TODO
		const FCoord SubNodeOffset = SVOData.GetSubNodeOffset();  // TODO

		const bool bDrawNodeBoxes = FSVORenderingHelpers::HasFlag(InNavDetailFlags, ESVODetailFlags::NodeBoxes);
		const bool bDrawSubNodeBoxes = FSVORenderingHelpers::HasFlag(InNavDetailFlags, ESVODetailFlags::SubNodeBoxes);
		const bool bDrawOnlyOverlappedSubNodes = FSVORenderingHelpers::HasFlag(InNavDetailFlags, ESVODetailFlags::OnlyOverlappedSubNodes);

		if (!bDrawNodeBoxes && !bDrawSubNodeBoxes)
		{
			return;
		}
		
		NavData->BeginBatchQuery();

		// Common function for finding colour for node
		TMap<int32, float> GroupColours;
		auto ColourForNode = [&](const float LayerProp, const FSVOLink& NodeLink) -> FColor
		{
			const float ColourProp = 1.f - LayerProp;
			const float RedProp = FMath::Clamp<float>((1.0f - ColourProp)/0.5f,0.f,1.f);
			const float BlueProp = FMath::Clamp<float>((ColourProp/0.5f),0.f,1.f);
			const int32 R = FMath::TruncToInt(255 * RedProp);
			const int32 G = 0;
			const int32 B = FMath::TruncToInt(255 * BlueProp);
			return FColor(R, G, B);
		};

		const float NumLayersInv = 1.f / static_cast<float>(SVOData.NumNodeLayers);
		
		// Draw leaf nodes:
		if (bDrawSubNodeBoxes)
		{
			for (const FSVONode& LayerOneNode : SVOData.GetLayer(1).Nodes)
			{
				if (LayerOneNode.HasChildren())
				{
					for (int32 LeafIdx = 0; LeafIdx < 8; LeafIdx++)
					{
						// Coordinate
						const FMorton LeafMortonCode = SVO::FirstChildFromParent(LayerOneNode.MortonCode) + LeafIdx;
						const FVector LeafCentre = SVO::MortonToCoord(LeafMortonCode, SVOData.Centre, LeafNodeSize, LeafOffset);
		
						const FSVOLink& LeafLink = LayerOneNode.FirstChild + LeafIdx;
						const int32 NodeIdx = LeafLink.GetNodeIndex();
						const FSVOLeafNode& LeafNode = SVOData.LeafLayer[NodeIdx];
					
						if (LeafNode.IsLeafFree())
						{
							const FColor NodeColour = ColourForNode(NumLayersInv, LeafLink);
							if (bDrawNodeBoxes)
							{
								NodeBoxes.Add(FSVORenderingHelpers::FSVOBox(LeafCentre, LeafExtent, NodeColour));
							}
						}
						else
						{
							// Gather SubNodes
							const FSVOLeafNode& Leaf = SVOData.LeafLayer[NodeIdx];
							for (int32 i = 0; i < 64; i++)
							{
								const FVector SubNodeCentre = SVO::MortonToCoord(static_cast<FMorton>(i), LeafCentre, SVOData.GetSubNodeSize(), SubNodeOffset);
								const bool bOverlap = Leaf.IsVoxelWithCollision(i);
		
								FColor NodeColour;
								if (bOverlap)
								{
									NodeColour = NavData->GetConfig().Color;
								} else
								{
									NodeColour = ColourForNode(0.f, FSVOLink(0, NodeIdx, i));
								}
							
								if ((bOverlap || !bDrawOnlyOverlappedSubNodes) && bDrawSubNodeBoxes)
								{
									NodeBoxes.Add(FSVORenderingHelpers::FSVOBox(SubNodeCentre, SubNodeExtent, NodeColour));
								}
							}
						}
					}
				}
			}
		}

		// Draw other layers
		if (bDrawNodeBoxes)
		{
			if (NavData->bDrawOnlySpecifiedLayer)
			{
				int32 LayerIdx = NavData->SpecifiedLayerBeDrawn;
				if (LayerIdx > 0 && LayerIdx <= SVOData.NumLayers)
				{
					const FCoord VoxelSideLength = LeafNodeSize * (1 << LayerIdx);
					const FVector VoxelExtent = SVOData.GetNodeExtentForLayer(LayerIdx);
					const FCoord VoxelOffset = SVOData.GetNodeOffsetForLayer(LayerIdx);
					const FCoord LayerProp = static_cast<FCoord>(LayerIdx + 1) * NumLayersInv;

					for (int32 NodeIndex = 0; NodeIndex < SVOData.GetLayer(LayerIdx).Num(); NodeIndex++)
					{
						const FSVONode& LayerNode = SVOData.GetLayer(LayerIdx).GetNode(NodeIndex);
						if (NavData->bDrawOnlyOnlyOverlappedNodes && !LayerNode.HasChildren())
						{
							continue;
						}

						const FSVOLink CurrentNodeLink(LayerIdx, NodeIndex, 0);
						const FVector VoxelCentre = SVO::MortonToCoord(LayerNode.MortonCode, SVOData.Centre, VoxelSideLength, VoxelOffset);
						const FColor NodeColour = ColourForNode(LayerProp, CurrentNodeLink);
						if (bDrawNodeBoxes)
						{
							NodeBoxes.Add(FSVORenderingHelpers::FSVOBox(VoxelCentre, VoxelExtent, NodeColour));
						}
					}
				}
			}
			else
			{
				for (int32 LayerIdx = 1; LayerIdx < SVOData.NumNodeLayers; LayerIdx++)
				{
					const FCoord VoxelSideLength = LeafNodeSize * (1 << LayerIdx);
					const FVector VoxelExtent = SVOData.GetNodeExtentForLayer(LayerIdx);
					const FCoord VoxelOffset = SVOData.GetNodeOffsetForLayer(LayerIdx);
					const FCoord LayerProp = static_cast<FCoord>(LayerIdx + 1) * NumLayersInv;

					for (int32 NodeIndex = 0; NodeIndex < SVOData.GetLayer(LayerIdx).Num(); NodeIndex++)
					{
						const FSVONode& LayerNode = SVOData.GetLayer(LayerIdx).GetNode(NodeIndex);
						const FSVOLink CurrentNodeLink(LayerIdx, NodeIndex, 0);
				
						const FVector VoxelCentre = SVO::MortonToCoord(LayerNode.MortonCode, SVOData.Centre, VoxelSideLength, VoxelOffset);
				
						if (!bDrawSubNodeBoxes || !LayerNode.HasChildren())
						{
							const FColor NodeColour = ColourForNode(LayerProp, CurrentNodeLink);
							if (bDrawNodeBoxes)
							{
								NodeBoxes.Add(FSVORenderingHelpers::FSVOBox(VoxelCentre, VoxelExtent, NodeColour));
							}
						}
					}
				}
			}
		}

		// Draw Placeholder root
		if ((NodeBoxes.Num() == 0 || SVOData.IsFree()) && bDrawNodeBoxes)
		{
			NodeBoxes.Add(FSVORenderingHelpers::FSVOBox(SVOData.Centre, SVOData.GetOctreeExtent(), FColor::Purple));
		}
		
		NavData->FinishBatchQuery();

		//******************************************
		// Build render data
		const int64 NumBoxes = NodeBoxes.Num();
		const int64 NumVerts = NumBoxes * NUM_CUBE_VERTS;     // 12 verts per box
		const int64 NumIndices = NumBoxes * NUM_CUBE_INDICES; // 36 indices per box
		
		Vertices.AddUninitialized(NumVerts);
		Indices.AddUninitialized(NumIndices);
		
		// Add each triangle to the vertex/index buffer
		for(int32 BoxIdx = 0; BoxIdx < NumBoxes; BoxIdx++)
		{
			const FSVORenderingHelpers::FSVOBox& Box = NodeBoxes[BoxIdx];
			const FSVORenderingHelpers::FTransformNoRot BoxTransform(Box.Centre,FVector(Box.Extent));

			// Copy over transformed vertices
			FDynamicMeshVertex Vert;
			Vert.Color = Box.Colour;
			
			for (int32 i = 0; i < NUM_CUBE_VERTS; i++)
			{
				// TODO: LWC precision loss
				Vert.Position = FVector3f(BoxTransform.TransformPositionNoRotation(FSVORenderingHelpers::CubeVerts[i].Pos));
				Vert.TextureCoordinate[0] = FVector2f(FSVORenderingHelpers::CubeVerts[i].TexCoord);
				Vertices[BoxIdx * NUM_CUBE_VERTS + i] = Vert;
			}

			// Copy over indices
			for (int32 i = 0; i < NUM_CUBE_TRIANGLES; i++)
			{
				const FSVORenderingHelpers::FTriangleIndices& Tri = FSVORenderingHelpers::CubeIndices[i];
				const int32 BaseIdx = BoxIdx * NUM_CUBE_INDICES + i * 3;
				
				Indices[BaseIdx + 0] = BoxIdx * NUM_CUBE_VERTS + Tri.V0;
				Indices[BaseIdx + 1] = BoxIdx * NUM_CUBE_VERTS + Tri.V1;
				Indices[BaseIdx + 2] = BoxIdx * NUM_CUBE_VERTS + Tri.V2;
			}
		}
	}
}

SIZE_T FSVOSceneProxy::GetTypeHash() const
{
	return FDebugRenderSceneProxy::GetTypeHash();
}

FSVOSceneProxy::FSVOSceneProxy(const USVORenderingComponent* InComponent, FSVOSceneProxyData* InProxyData, bool ForceToRender)
	: FDebugRenderSceneProxy(InComponent)
	, bForceRendering(ForceToRender)
	, VertexFactory(GetScene().GetFeatureLevel(), "FSVOSceneProxy")
{
	if (InProxyData->Vertices.Num() == 0)
	{
		bHasMeshData = false;
		return;
	}
	bHasMeshData = true;

	IndexBuffer.Indices = InProxyData->Indices;
	VertexBuffers.InitFromDynamicVertex(&VertexFactory, InProxyData->Vertices);

	// Enqueue initialization of render resource
	BeginInitResource(&VertexBuffers.PositionVertexBuffer);
	BeginInitResource(&VertexBuffers.StaticMeshVertexBuffer);
	BeginInitResource(&VertexBuffers.ColorVertexBuffer);
	BeginInitResource(&IndexBuffer);
	BeginInitResource(&VertexFactory);

	// Grab material
	Material = InComponent->GetMaterial(0);
	if (Material == nullptr)
	{
		Material = UMaterial::GetDefaultMaterial(MD_Surface);
	}
}

FSVOSceneProxy::~FSVOSceneProxy()
{
	VertexBuffers.PositionVertexBuffer.ReleaseResource();
	VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
	VertexBuffers.ColorVertexBuffer.ReleaseResource();
	IndexBuffer.ReleaseResource();
	VertexFactory.ReleaseResource();
}

void FSVOSceneProxy::GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily,
	uint32 VisibilityMap, FMeshElementCollector& Collector) const
{
	if (!bHasMeshData)
	{
		return;
	}
	
	FMaterialRenderProxy* MaterialProxy = Material->GetRenderProxy();

	for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
	{
		if (VisibilityMap & (1 << ViewIndex))
		{
			// Draw the mesh.
			FMeshBatch& Mesh = Collector.AllocateMesh();
			FMeshBatchElement& BatchElement = Mesh.Elements[0];
			BatchElement.IndexBuffer = &IndexBuffer;
			//Mesh.bWireframe = bWireframe;
			Mesh.VertexFactory = &VertexFactory;
			Mesh.MaterialRenderProxy = MaterialProxy;

			bool bHasPrecomputedVolumetricLightmap;
			FMatrix PreviousLocalToWorld;
			int32 SingleCaptureIndex;
			bool bOutputVelocity;
			GetScene().GetPrimitiveUniformShaderParameters_RenderThread(GetPrimitiveSceneInfo(), bHasPrecomputedVolumetricLightmap, PreviousLocalToWorld, SingleCaptureIndex, bOutputVelocity);

			FDynamicPrimitiveUniformBuffer& DynamicPrimitiveUniformBuffer = Collector.AllocateOneFrameResource<FDynamicPrimitiveUniformBuffer>();
			DynamicPrimitiveUniformBuffer.Set(Collector.GetRHICommandList(), GetLocalToWorld(), PreviousLocalToWorld, GetBounds(), GetLocalBounds(), true, bHasPrecomputedVolumetricLightmap, false);

			BatchElement.PrimitiveUniformBufferResource = &DynamicPrimitiveUniformBuffer.UniformBuffer;

			BatchElement.FirstIndex = 0;
			BatchElement.NumPrimitives = IndexBuffer.Indices.Num() / 3;
			BatchElement.MinVertexIndex = 0;
			BatchElement.MaxVertexIndex = VertexBuffers.PositionVertexBuffer.GetNumVertices() - 1;
			Mesh.ReverseCulling = IsLocalToWorldDeterminantNegative();
			Mesh.Type = PT_TriangleList;
			Mesh.DepthPriorityGroup = SDPG_World;
			Mesh.bCanApplyViewModeOverrides = false;
			Collector.AddMesh(ViewIndex, Mesh);
		}
	}
}

FPrimitiveViewRelevance FSVOSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	const bool bVisible = !!View->Family->EngineShowFlags.Navigation || bForceRendering;
		
	// From UCustomMeshComponent::GetViewRelevance
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance = bVisible && IsShown(View);
	Result.bShadowRelevance = false;
	Result.bDynamicRelevance = true;
	Result.bStaticRelevance = false;
	Result.bRenderInMainPass = ShouldRenderInMainPass();
	Result.bUsesLightingChannels = false;
	Result.bRenderCustomDepth = ShouldRenderCustomDepth();
	Result.bTranslucentSelfShadow = false;
	Result.bVelocityRelevance = false;

	if (bHasMeshData)
	{
		const FMaterialRelevance& MaterialRelevance = Material->GetRelevance_Concurrent(GetScene().GetFeatureLevel());
		MaterialRelevance.SetPrimitiveViewRelevance(Result);
	}
	return Result;
}

bool FSVOSceneProxy::CanBeOccluded() const
{
	return false;
}

uint32 FSVOSceneProxy::GetAllocatedSizeInternal() const
{
	return FPrimitiveSceneProxy::GetAllocatedSize() +
		IndexBuffer.Indices.GetAllocatedSize() +
		VertexBuffers.PositionVertexBuffer.GetNumVertices() * VertexBuffers.PositionVertexBuffer.GetStride() +
		VertexBuffers.StaticMeshVertexBuffer.GetResourceSize() +
		VertexBuffers.ColorVertexBuffer.GetNumVertices() * VertexBuffers.ColorVertexBuffer.GetStride();
}

bool USVORenderingComponent::IsNavigationShowFlagSet(const UWorld* World)
{
	bool bShowNavigation = false;

	FWorldContext* WorldContext = GEngine->GetWorldContextFromWorld(World);

#if WITH_EDITOR
	if (GEditor && WorldContext && WorldContext->WorldType != EWorldType::Game)
	{
		bShowNavigation = WorldContext->GameViewport != nullptr && WorldContext->GameViewport->EngineShowFlags.Navigation;
		if (bShowNavigation == false)
		{
			// we have to check all viewports because we can't to distinguish between SIE and PIE at this point.
			for (FEditorViewportClient* CurrentViewport : GEditor->GetAllViewportClients())
			{
				if (CurrentViewport && CurrentViewport->EngineShowFlags.Navigation)
				{
					bShowNavigation = true;
					break;
				}
			}
		}
	}
	else
#endif //WITH_EDITOR
	{
		bShowNavigation = WorldContext && WorldContext->GameViewport && WorldContext->GameViewport->EngineShowFlags.Navigation;
	}

	return bShowNavigation;
}

#if WITH_EDITOR
namespace
{
	bool AreAnyViewportsRelevant(const UWorld* World)
	{
		FWorldContext* WorldContext = GEngine->GetWorldContextFromWorld(World);
		if (WorldContext && WorldContext->GameViewport)
		{
			return true;
		}
		
		for (FEditorViewportClient* CurrentViewport : GEditor->GetAllViewportClients())
		{
			if (CurrentViewport && CurrentViewport->IsVisible())
			{
				return true;
			}
		}

		return false;
	}
}
#endif

USVORenderingComponent::USVORenderingComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	bIsEditorOnly = true;
	bSelectable = false;
	bCollectNavigationData = false;
	bForceUpdate = false;

	static const ConstructorHelpers::FObjectFinder<UMaterial> WireMat(TEXT("Material'/FlightNavigation/M_Wireframe.M_Wireframe'"));
	
	WireMaterial = UMaterialInstanceDynamic::Create(WireMat.Object, this, TEXT("WireMaterial"));
	WireMaterial->SetScalarParameterValue(TEXT("Thickness"), FMath::Clamp(0.2f, 0.f, 1.f));
	Super::SetMaterial(0, WireMaterial);
}

void USVORenderingComponent::OnRegister()
{
	Super::OnRegister();
	
#if UE_ENABLE_DEBUG_DRAWING
	// it's a kind of HACK but there is no event or other information that show flag was changed by user => we have to check it periodically
#if WITH_EDITOR
	if (GEditor)
	{
		GEditor->GetTimerManager()->SetTimer(TimerHandle, FTimerDelegate::CreateUObject(this, &USVORenderingComponent::TimerFunction), 1, true);
	}
	else
#endif //WITH_EDITOR
	{
		GetWorld()->GetTimerManager().SetTimer(TimerHandle, FTimerDelegate::CreateUObject(this, &USVORenderingComponent::TimerFunction), 1, true);
	}
#endif //UE_ENABLE_DEBUG_DRAWING
}

void USVORenderingComponent::OnUnregister()
{
#if UE_ENABLE_DEBUG_DRAWING
	// it's a kind of HACK but there is no event or other information that show flag was changed by user => we have to check it periodically
#if WITH_EDITOR
	if (GEditor)
	{
		GEditor->GetTimerManager()->ClearTimer(TimerHandle);
	}
	else
#endif //WITH_EDITOR
	{
		GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
	}
#endif //UE_ENABLE_DEBUG_DRAWING
	Super::OnUnregister();
}

FPrimitiveSceneProxy* USVORenderingComponent::CreateSceneProxy()
{
	FSVOSceneProxy* SVOSceneProxy = nullptr;

	const bool bShowNavigation = IsNavigationShowFlagSet(GetWorld());

	bCollectNavigationData = bShowNavigation;

	if (bCollectNavigationData && IsVisible())
	{
		const AFlightNavigationData* NavData = Cast<AFlightNavigationData>(GetOwner());
		if (NavData && NavData->IsDrawingEnabled())
		{
			FSVOSceneProxyData ProxyData;
			GatherData(*NavData, ProxyData);

			SVOSceneProxy = new FSVOSceneProxy(this, &ProxyData);
		}
	}

	return SVOSceneProxy;
}

FBoxSphereBounds USVORenderingComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	FBox BoundingBox(ForceInit);

	AFlightNavigationData* NavData = Cast<AFlightNavigationData>(GetOwner());
	if (NavData)
	{
		BoundingBox = NavData->GetFlightBounds();
	}

	return FBoxSphereBounds(BoundingBox);
}

int32 USVORenderingComponent::GetNumMaterials() const
{
	return 1;
}

void USVORenderingComponent::GatherData(const AFlightNavigationData& NavData, FSVOSceneProxyData& OutProxyData) const
{
	const int32 DetailFlags = FSVORenderingHelpers::GetDetailFlags(&NavData);
	OutProxyData.GatherData(&NavData, DetailFlags);
}


void USVORenderingComponent::TimerFunction()
{
	const UWorld* World = GetWorld();
#if WITH_EDITOR
	if (GEditor && (AreAnyViewportsRelevant(World) == false))
	{
		// unable to tell if the flag is on or not
		return;
	}
#endif // WITH_EDITOR

	const bool bShowNavigation = bForceUpdate || IsNavigationShowFlagSet(World);

	if (bShowNavigation != !!bCollectNavigationData && bShowNavigation == true)
	{
		bForceUpdate = false;
		bCollectNavigationData = bShowNavigation;
		MarkRenderStateDirty();
	}
}