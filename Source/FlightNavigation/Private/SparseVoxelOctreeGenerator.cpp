// Fill out your copyright notice in the Description page of Project Settings.


#include "SparseVoxelOctreeGenerator.h"

#include "NavigationOctree.h"
#include "NavigationSystem.h"
#include "AABBTriangleOverlap.h"
#include "AI/NavigationSystemHelpers.h"

struct FSVOGeometryExport final : FNavigableGeometryExport
{
	FSVOGeometryExport(FNavigationRelevantData& InData) : Data(&InData) 
	{
	}

	FNavigationRelevantData* Data;
	
	virtual void ExportChaosTriMesh(const Chaos::FTriangleMeshImplicitObject* const TriMesh, const FTransform& LocalToWorld) override;
	virtual void ExportChaosConvexMesh(const FKConvexElem* const Convex, const FTransform& LocalToWorld) override;
	virtual void ExportChaosHeightField(const Chaos::FHeightField* const Heightfield, const FTransform& LocalToWorld) override;
	virtual void ExportChaosHeightFieldSlice(const FNavHeightfieldSamples& PrefetchedHeightfieldSamples, const int32 NumRows, const int32 NumCols, const FTransform& LocalToWorld, const FBox& SliceBox) override;
	virtual void ExportCustomMesh(const FVector* InVertices, int32 NumVerts, const int32* InIndices, int32 NumIndices, const FTransform& LocalToWorld) override;
	virtual void ExportRigidBodySetup(UBodySetup& BodySetup, const FTransform& LocalToWorld) override;
	virtual void AddNavModifiers(const FCompositeNavModifier& Modifiers) override;
	virtual void SetNavDataPerInstanceTransformDelegate(const FNavDataPerInstanceTransformDelegate& InDelegate) override;
};

void FSVOGeometryExport::ExportChaosTriMesh(const Chaos::FTriangleMeshImplicitObject* const TriMesh, const FTransform& LocalToWorld)
{
}

void FSVOGeometryExport::ExportChaosConvexMesh(const FKConvexElem* const Convex, const FTransform& LocalToWorld)
{
}

void FSVOGeometryExport::ExportChaosHeightField(const Chaos::FHeightField* const Heightfield, const FTransform& LocalToWorld)
{
}

void FSVOGeometryExport::ExportChaosHeightFieldSlice(const FNavHeightfieldSamples& PrefetchedHeightfieldSamples, const int32 NumRows, const int32 NumCols, const FTransform& LocalToWorld, const FBox& SliceBox)
{
}

void FSVOGeometryExport::ExportCustomMesh(const FVector* InVertices, int32 NumVerts, const int32* InIndices, int32 NumIndices, const FTransform& LocalToWorld)
{
}

void FSVOGeometryExport::ExportRigidBodySetup(UBodySetup& BodySetup, const FTransform& LocalToWorld)
{
}

void FSVOGeometryExport::AddNavModifiers(const FCompositeNavModifier& Modifiers)
{
}

void FSVOGeometryExport::SetNavDataPerInstanceTransformDelegate(const FNavDataPerInstanceTransformDelegate& InDelegate)
{
	Data->NavDataPerInstanceTransformDelegate = InDelegate;
}

namespace SVOGeometryExport
{
	static void ConvertCoordDataFromRecast(TArray<FCoord>& Coords)
	{
		FCoord* CoordPtr = Coords.GetData();
		const int32 MaxIt = Coords.Num() / 3;
		for (int32 i = 0; i < MaxIt; i++)
		{
			CoordPtr[0] = -CoordPtr[0];

			const FCoord TmpV = CoordPtr[1];
			CoordPtr[1] = -CoordPtr[2];
			CoordPtr[2] = TmpV;

			CoordPtr += 3;
		}
	}
}

struct FSVOGeometryCache
{
	struct FHeader
	{
		FNavigationRelevantData::FCollisionDataHeader Validation;
		
		int32 NumVerts;
		int32 NumFaces;
		struct FWalkableSlopeOverride SlopeOverride;
	};

	FHeader Header;

	/** recast coords of vertices (size: NumVerts * 3) */
	FVector::FReal* Verts;

	/** vert indices for triangles (size: NumFaces * 3) */
	int32* Indices;

	FSVOGeometryCache() {}
	FSVOGeometryCache(const uint8* Memory)
	{
		Header = *((FHeader*)Memory);
		Verts = (FVector::FReal*)(Memory + sizeof(FSVOGeometryCache));
		Indices = (int32*)(Memory + sizeof(FSVOGeometryCache) + (sizeof(FVector::FReal) * Header.NumVerts * 3));
	}
};

FSVOLeafGenerator::FSVOLeafGenerator(FSVOGenerator& InParentGenerator,
	const FBox& InRootBounds, const FMorton InWorkerIdx, const int32 InNodeIdx, const int32 InFirstLeafIdx)
	: ParentGeneratorRef(InParentGenerator)
	, SVODataRef(InParentGenerator.SVODataRef)
	, RootBounds(InRootBounds)
	, WorkerIdx(InWorkerIdx)
	, NodeIdx(InNodeIdx)
	, FirstLeafIdx(InFirstLeafIdx)
	, LayerOne(InParentGenerator.BuildingLayerOne)
	, LeafLayer(InParentGenerator.BuildingLeafLayer)
{
	const float AgentRadius = GetOwner()->AgentRadius;
	const bool bUseAgentRadius = AgentRadius > 0;
	if (bUseAgentRadius)
	{
		RootBounds = RootBounds.ExpandBy(AgentRadius);
	}
	// Used for landscape slicing
    RootBoundExpandedForAgent = RootBounds.ExpandBy(AgentRadius * 2.f);
	
	PrepareGeometrySources();
}

void FSVOLeafGenerator::DoTask()
{
	GatherGeometryFromSources();

	if (RawGeometry.Num() == 0)
	{
		return;
	}
	
	RasteriseLayerOne();
}

const AFlightNavigationData* FSVOLeafGenerator::GetOwner() const
{
	return ParentGeneratorRef.GetOwner();
}

UWorld* FSVOLeafGenerator::GetWorld() const
{
	return ParentGeneratorRef.GetWorld();
}

void FSVOLeafGenerator::PrepareGeometrySources()
{
	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	const FNavigationOctree* NavOctreeInstance = NavSys ? NavSys->GetNavOctree() : nullptr;
	check(NavOctreeInstance);
	NavigationRelevantData.Reset();
	NavSystem = NavSys;
	
	const FNavDataConfig& NavDataConfig = GetOwner()->GetConfig();
	
	NavOctreeInstance->FindElementsWithBoundsTest(RootBounds, [&NavDataConfig, this](const FNavigationOctreeElement& Element)
	{
		// Must have valid owner
		bool bShouldUse = Element.GetOwner() != nullptr && Element.ShouldUseGeometry(NavDataConfig);

		// Don't bother with simulated objects
		if (const UStaticMeshComponent* StaticMeshComp = Cast<const UStaticMeshComponent>(Element.GetOwner()))
		{
			bShouldUse &= !StaticMeshComp->IsSimulatingPhysics();
		}
		// Needs geometry or has lazy geometry pending
		bShouldUse &= Element.Data->HasGeometry() || Element.Data->IsPendingLazyGeometryGathering();

		// Allow multiple sliced objects, otherwise only add one copy
		// bShouldUse &= Element.Data->SupportsGatheringGeometrySlices() || !NavigationRelevantData.Contains(Element.Data);
		
		// Check if we've already added this data
		if (bShouldUse)
		{
			NavigationRelevantData.Add(Element.Data);
		}
	});
}

void FSVOLeafGenerator::GatherGeometryFromSources()
{
	if (!NavSystem.IsValid())
	{
		return;
	}

	for (TSharedRef<FNavigationRelevantData, ESPMode::ThreadSafe>& ElementData : NavigationRelevantData)
	{
		if (ElementData->IsPendingLazyGeometryGathering() || ElementData->NeedAnyPendingLazyModifiersGathering())
    	{
    		NavSystem->DemandLazyDataGathering(*ElementData);
    	}

		// The only thing that supports slicing is landscapes
		if (ElementData->IsPendingLazyGeometryGathering() && ElementData->SupportsGatheringGeometrySlices())
		{
			FSVOGeometryExport GeomExport(const_cast<FNavigationRelevantData&>(*ElementData));

			INavRelevantInterface* NavRelevant = const_cast<INavRelevantInterface*>(Cast<const INavRelevantInterface>(ElementData->GetOwner()));
			if (NavRelevant)
			{
				NavRelevant->PrepareGeometryExportSync();
				// adding a small bump to avoid special case of zero-expansion when tile bounds
				// overlap landscape's tile bounds
				NavRelevant->GatherGeometrySlice(GeomExport, RootBoundExpandedForAgent);
				
				// TODO: export and append landscape slicing
				// TNavStatArray<uint8> CollisionData;	
				// SVOGeometryExport::StoreCollisionCache(GeomExport, CollisionData);
				// AppendCollisionData(CollisionData, GeomExport.Bounds, false);
			}
			else
			{
				UE_LOG(LogFlightNavigation, Error, TEXT("GatherGeometry: got an invalid NavRelevant instance!"));
			}
		}

		const bool bExportGeometry = ElementData->HasGeometry();
		if (bExportGeometry)
		{
			ValidateAndAppendGeometry(ElementData.Get());
		}
	}

	// Don't need to keep these around
	NavigationRelevantData.Reset();
}

void FSVOLeafGenerator::ValidateAndAppendGeometry(const FNavigationRelevantData& ElementData)
{
	if (ElementData.IsCollisionDataValid())
	{
		AppendGeometry(ElementData, ElementData.NavDataPerInstanceTransformDelegate);
	}
}

void FSVOLeafGenerator::AppendGeometry(const FNavigationRelevantData& ElementData,
	const FNavDataPerInstanceTransformDelegate& InTransformsDelegate)
{
	const TNavStatArray<uint8>& RawCollisionCache = ElementData.CollisionData;
	if (RawCollisionCache.Num() == 0)
	{
		return;
	}
	
	FSVORawGeometryElement GeometryElement;
	
	FSVOGeometryCache CollisionCache(RawCollisionCache.GetData());
	
	// Gather per instance transforms
	if (InTransformsDelegate.IsBound())
	{
		InTransformsDelegate.Execute(RootBounds, GeometryElement.PerInstanceTransform);
		if (GeometryElement.PerInstanceTransform.Num() == 0)
		{
			return;
		}
	}

	const int32 NumCoords = CollisionCache.Header.NumVerts * 3;
	const int32 NumIndices = CollisionCache.Header.NumFaces * 3;
	if (NumIndices > 0)
	{
		UE_LOG(LogFlightNavigation, VeryVerbose, TEXT("%s adding %i vertices from %s."), ANSI_TO_TCHAR(__FUNCTION__), CollisionCache.Header.NumVerts, *GetFullNameSafe(GetOwner()));

		GeometryElement.GeomCoords.SetNumUninitialized(NumCoords);
		GeometryElement.GeomIndices.SetNumUninitialized(NumIndices);

		FMemory::Memcpy(GeometryElement.GeomCoords.GetData(), CollisionCache.Verts, sizeof(FVector::FReal) * NumCoords);
		FMemory::Memcpy(GeometryElement.GeomIndices.GetData(), CollisionCache.Indices, sizeof(int32) * NumIndices);
		
#if WITH_RECAST
		SVOGeometryExport::ConvertCoordDataFromRecast(GeometryElement.GeomCoords);
#endif // WITH_RECAST

		RawGeometry.Add(MoveTemp(GeometryElement));
	}	
}

void FSVOLeafGenerator::RasteriseLayerOne()
{
	FCoord LayerOneNodeSize = SVODataRef->GetNodeSizeForLayer(1);
	FCoord LayerOneNodeOffset = SVODataRef->GetNodeOffsetForLayer(1);
	FVector LayerOneNodeExtent = SVODataRef->GetNodeExtentForLayer(1);
	
	FCoord LayerZeroNodeSize = SVODataRef->GetNodeSizeForLayer(0);
	FCoord LayerZeroNodeOffset = SVODataRef->GetNodeOffsetForLayer(0);

	const FMorton LayerOneNodeMortonCode = WorkerIdx;
	FVector LayerOneNodeCentre = SVO::MortonToCoord(LayerOneNodeMortonCode, SVODataRef->GetCentre(), LayerOneNodeSize, LayerOneNodeOffset);
	
	bool bLayerOneNodeOverlapCollision = NodeOverlapRawGeometry(LayerOneNodeCentre, LayerOneNodeExtent);
	if (bLayerOneNodeOverlapCollision)
	{
		const FMorton FirstChildMortonCode = SVO::FirstChildFromParent(LayerOneNodeMortonCode);

		FSVONode& LayerOneNode = LayerOne[NodeIdx];
		LayerOneNode.MortonCode = LayerOneNodeMortonCode;
		// LayerOneNode.FirstChild = FSVOLink(0, FirstLeafIdx);
		LayerOneNode.bHasChildren = true;
		
		for (FMorton Child = 0; Child < 8; ++Child)
		{
			FMorton ChildMortonCode = FirstChildMortonCode + Child;
			FVector LayerZeroNodeCentre = SVO::MortonToCoord(ChildMortonCode, SVODataRef->GetCentre(), LayerZeroNodeSize, LayerZeroNodeOffset);
			FSVOLeafNode& LeafNode = LeafLayer[FirstLeafIdx + Child];
			// LeafNode.ParentLink = FSVOLink(1, LayerOneNodeIndex);
			RasteriseLeafNode(LeafNode, LayerZeroNodeCentre, SVODataRef->GetSubNodeOffset(), SVODataRef->GetSubNodeExtent());
		}
	}
}

void FSVOLeafGenerator::RasteriseLeafNode(FSVOLeafNode& LeafNode, const FVector& LeafCentre,
                                                   const FCoord SubNodeOffset, const FVector& SubNodeExtent)
{
	FCoord SubNodeSize = SVODataRef->GetSubNodeSize();
	for (FMorton i = 0; i < 64; ++i)
	{
		FVector SubNodeCentre = SVO::MortonToCoord(i, LeafCentre, SubNodeSize, SubNodeOffset);
		const bool bOverlapCollision = NodeOverlapRawGeometry(SubNodeCentre, SubNodeExtent);
		if (bOverlapCollision)
		{
			LeafNode.SetVoxelWithCollision(i);
		}
	}
}

bool FSVOLeafGenerator::NodeOverlapRawGeometry(const FVector& Centre, const FVector& Extent)
{
	bool bOverlap = false;
	for (const FSVORawGeometryElement& Geometry : RawGeometry)
	{
		bOverlap = NodeOverlapGeometry(Geometry, Centre, Extent);
		if (bOverlap)
		{
			break;
		}
	}
	return bOverlap;
}

bool FSVOLeafGenerator::NodeOverlapGeometry(const FSVORawGeometryElement& Geometry,
	const FVector& Centre, const FVector& Extent)
{
	const int32 NumTris = Geometry.GeomIndices.Num() / 3;
	const int32* IndicesPtr = Geometry.GeomIndices.GetData();
	const FCoord* CoordsPtr = Geometry.GeomCoords.GetData();
	// FBox NodeAABB = FBox::BuildAABB(Centre, Extent);
	
	// Coarse bounding box check
	if (Geometry.PerInstanceTransform.Num() > 0)
	{
		for (int32 InstanceIndex = 0; InstanceIndex < Geometry.PerInstanceTransform.Num(); ++InstanceIndex)
		{
			// const FBox& Bounds = Geometry.PerInstanceBounds[InstanceIndex];
			// if (!Bounds.Intersect(NodeAABB))
			// {
			// 	continue;
			// }

			// Check box against each triangle
			const FTransform& Transform = Geometry.PerInstanceTransform[InstanceIndex];
			for (int i = 0; i < NumTris; i++)
			{
				if (SVO::TriBoxOverlap(Centre, Extent, IndicesPtr, CoordsPtr, i, Transform))
				{
					return true;
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < NumTris; i++)
		{
			if (SVO::TriBoxOverlap(Centre, Extent, IndicesPtr, CoordsPtr, i))
			{
				return true;
			}
		}
	}
	
	return false;
}

FSVOGenerator::FSVOGenerator(FSparseVoxelOctreeGenerator& InGenerator)
	: ParentGeneratorRef(InGenerator)
	, SVODataRef(InGenerator.SVOData)
{
	SVODataRef->Clear();
	
	SVODataRef->SetBounds(InGenerator.TotalBounds);

	const FCoord MaxVoxelSize = ParentGeneratorRef.DestNavData->MaxVoxelSize;

	NumLayers = SVO::GetNumLayers(SVODataRef->SideSize, MaxVoxelSize * 4);
	SVODataRef->NumLayers = NumLayers;
	SVODataRef->NumNodeLayers = SVODataRef->NumLayers - 2;
	
	SVODataRef->SubNodeSize = SVODataRef->SideSize / (1 << (NumLayers + 2));
}

const AFlightNavigationData* FSVOGenerator::GetOwner() const
{
	return ParentGeneratorRef.GetOwner();
}

UWorld* FSVOGenerator::GetWorld() const
{
	return ParentGeneratorRef.GetWorld();
}

void FSVOGenerator::DoTask()
{
	if (!SVODataRef->bValid)
	{
		return;
	}
	
	const int32 NumLayerTwoNodes =  1 << (3 * (NumLayers - 2));
	FCoord LayerOneNodeSize = SVODataRef->GetNodeSizeForLayer(1);
	FCoord LayerOneNodeOffset = SVODataRef->GetNodeOffsetForLayer(1);
	FVector LayerOneNodeExtent = SVODataRef->GetNodeExtentForLayer(1);
	BuildingLayerOne.Reserve(NumLayerTwoNodes >> 3);
	BuildingLeafLayer.Reserve(NumLayerTwoNodes);
	UE_LOG(LogFlightNavigation, Display, TEXT("Sparse Voxel Octree Subnode Generate Task LayerOneNodeSize: %f, LayerOneNodeOffset: %f"), LayerOneNodeSize, LayerOneNodeOffset);

	TArray<FMorton> LayerOneNodeMortonArray, LayerOneNodeRelevantMortonArray;
	LayerOneNodeMortonArray.Reserve(NumLayerTwoNodes >> 3);
	LayerOneNodeRelevantMortonArray.Reserve(NumLayerTwoNodes >> 3);
	for (FMorton LayerTwoNodeMortonCode = 0; LayerTwoNodeMortonCode < NumLayerTwoNodes; ++LayerTwoNodeMortonCode)
	{
		FMorton FirstLayerOneNodeMortonCode = LayerTwoNodeMortonCode << 3;
		bool bLayerTwoNodeValid = false;
		for (FMorton Index = 0; Index < 8; ++Index)
		{
			bool bLayerOneNodeValid = false;
			FMorton LayerOneNodeMortonCode = FirstLayerOneNodeMortonCode + Index;
			FVector LayerOneNodeCentre = SVO::MortonToCoord(LayerOneNodeMortonCode, SVODataRef->GetCentre(), LayerOneNodeSize, LayerOneNodeOffset);

			FBox LayerOneNodeBounds = FBox::BuildAABB(LayerOneNodeCentre, LayerOneNodeExtent);
			for (const FBox& Bounds : ParentGeneratorRef.InclusionBounds)
			{
				FBox Intersect = Bounds.Overlap(LayerOneNodeBounds);
				if (Intersect.IsValid)
				{
					bLayerOneNodeValid = true;
					LayerOneNodeRelevantMortonArray.Emplace(LayerOneNodeMortonCode);
					break;
				}
			}

			bLayerTwoNodeValid |= bLayerOneNodeValid;
		}

		if (bLayerTwoNodeValid)
		{
			BuildingLayerOne.AddChildlessNodes(8, FirstLayerOneNodeMortonCode);
			BuildingLeafLayer.AddDefaulted(64);
			for (FMorton Index = 0; Index < 8; ++Index)
			{
				LayerOneNodeMortonArray.Emplace(FirstLayerOneNodeMortonCode + Index);
			}
		}
	}

	int32 RelevantIndex = 0;
	for (int32 Index = 0; Index < LayerOneNodeMortonArray.Num() && RelevantIndex < LayerOneNodeRelevantMortonArray.Num(); ++Index)
	{
		FMorton LayerOneNodeMortonCode = LayerOneNodeMortonArray[Index];
		if (LayerOneNodeMortonCode != LayerOneNodeRelevantMortonArray[RelevantIndex])
		{
			continue;
		}
		++RelevantIndex;
		FVector LayerOneNodeCentre = SVO::MortonToCoord(LayerOneNodeMortonCode, SVODataRef->GetCentre(), LayerOneNodeSize, LayerOneNodeOffset);

		FBox LayerOneNodeBounds = FBox::BuildAABB(LayerOneNodeCentre, LayerOneNodeExtent);
		
		FGraphEventRef Task = TGraphTask<FSparseVoxelOctreeLeafGenerateTask>::CreateTask().ConstructAndDispatchWhenReady(*this, LayerOneNodeBounds, LayerOneNodeMortonCode, Index, Index * 8);
		RunningTasks.Add(Task);
	}

	FTaskGraphInterface::Get().WaitUntilTasksComplete(RunningTasks);
	RunningTasks.Reset();

	FSVOLayer& LayerOne = SVODataRef->AddLayer_GetRef();
	ExcludeLeafNodes(LayerOne, SVODataRef->LeafLayer);

	for (int32 LayerIndex = 2; LayerIndex <= NumLayers; ++LayerIndex)
	{
		RasteriseLayer(LayerIndex);
	}

	for (int32 LayerIndex = 1; LayerIndex < NumLayers; ++LayerIndex)
	{
		GenerateNeighbours(LayerIndex);
	}
	
	UE_LOG(LogFlightNavigation, Display, TEXT("Sparse Voxel Octree Subnode Generate Task Finished"));
}



void FSVOGenerator::RasteriseLayer(const int32 LayerIndex) const
{
	check(LayerIndex < (1 << 4))
	check(LayerIndex > 1)
	check(LayerIndex - 1 == SVODataRef->Layers.Num())

	FSVOLayer& LastLayer = SVODataRef->GetLayer(LayerIndex - 1);
	const int32 NumNodesInLastLayer = LastLayer.Num();
	check(NumNodesInLastLayer % 8 == 0);
	
	FSVOLayer& CurrentLayer = SVODataRef->AddLayer_GetRef();
	// const int32 NumNodesInCurrentLayer = NumNodesInLastLayer >> 3;

	if (LayerIndex == NumLayers)
	{
		CurrentLayer.FillWithChildlessNodes(0, 0);
	}
	else
	{
		TArray<FMorton> FirstMortonCodes, LastMortonCodes;
		FMorton LastMortonCode = 0;
		for (int32 NodeIndexInLastLayer = 0; NodeIndexInLastLayer < NumNodesInLastLayer; NodeIndexInLastLayer += 8)
		{
			FSVONode& NodeInLastLayer = LastLayer[NodeIndexInLastLayer];
			FMorton CurrentMortonCode = SVO::ParentFromAnyChild(NodeInLastLayer.MortonCode);
			if (LastMortonCode != 0 && CurrentMortonCode <= LastMortonCode)
			{
				continue;
			}
		
			FMorton FirstMortonCode = SVO::FirstChildFromAnyChild(CurrentMortonCode);
			LastMortonCode = SVO::LastChildFromAnyChild(CurrentMortonCode);
			FirstMortonCodes.Add(FirstMortonCode);
			LastMortonCodes.Add(LastMortonCode);
		}

		CurrentLayer.Reserve(FirstMortonCodes.Num() * 8);
		for (int32 Index = 0; Index < FirstMortonCodes.Num(); ++Index)
		{
			CurrentLayer.FillWithChildlessNodes(FirstMortonCodes[Index], LastMortonCodes[Index]);
		}
	}
	
	for (int32 FirstNodeIndexInLastLayer = 0; FirstNodeIndexInLastLayer < NumNodesInLastLayer; FirstNodeIndexInLastLayer += 8)
	{
		FMorton FirstNodeInLastLayerMortonCode = LastLayer[FirstNodeIndexInLastLayer].MortonCode;
		
		bool bNodeInCurrentLayerHasChildren = false;
		for (int32 Child = 0; Child < 8; ++Child)
		{
			int32 NodeIndexInLastLayer = FirstNodeIndexInLastLayer + Child;
			bNodeInCurrentLayerHasChildren |= LastLayer[NodeIndexInLastLayer].HasChildren();
		}

		FMorton MortonCodeInCurrentLayer = SVO::ParentFromAnyChild(FirstNodeInLastLayerMortonCode);
		int32 NodeIndexInCurrentLayer = CurrentLayer.FindNode(MortonCodeInCurrentLayer);
		if (bNodeInCurrentLayerHasChildren)
		{
			FSVONode& NodeInCurrentLayer = CurrentLayer[NodeIndexInCurrentLayer];
			NodeInCurrentLayer.FirstChild = FSVOLink(LayerIndex - 1, FirstNodeIndexInLastLayer);
			NodeInCurrentLayer.bHasChildren = true;
		}

		for (int32 Child = 0; Child < 8; ++Child)
		{
			int32 NodeIndexInLastLayer = FirstNodeIndexInLastLayer + Child;
			FSVONode& ChildNode = LastLayer[NodeIndexInLastLayer];
			ChildNode.Parent = FSVOLink(LayerIndex, NodeIndexInCurrentLayer);
		}
	}
}

void FSVOGenerator::ExcludeLeafNodes(FSVOLayer& OutLayerOne, FSVOLeafLayer& OutLeafLayer)
{
	TArray<int32> NodeIndexBeIncluded;
	NodeIndexBeIncluded.Reserve(BuildingLayerOne.Num());
	for (int32 NodeIndex = 0; NodeIndex < BuildingLayerOne.Num(); NodeIndex += 8)
	{
		bool bShouldNodeBeIncluded = false;
		for (int32 Child = 0; Child < 8; ++Child)
		{
			bShouldNodeBeIncluded |= BuildingLayerOne[NodeIndex + Child].IsWithCollision();
		}

		if (bShouldNodeBeIncluded)
		{
			for (int32 Child = 0; Child < 8; ++Child)
			{
				NodeIndexBeIncluded.Emplace(NodeIndex + Child);
			}
		}
	}

	OutLayerOne.Reserve(NodeIndexBeIncluded.Num());
	OutLeafLayer.Reserve(NodeIndexBeIncluded.Num() << 3);
	for (int32 NodeIndex : NodeIndexBeIncluded)
	{
		int32 OutFirstLeafIndex = OutLeafLayer.Num();
		int32 OutLayerOneNodeIndex = OutLayerOne.Num();
		
		FSVONode& Node = OutLayerOne.AddDefaulted_GetRef();
		Node.MortonCode = BuildingLayerOne[NodeIndex].MortonCode;
		Node.bHasChildren = BuildingLayerOne[NodeIndex].bHasChildren;
		Node.FirstChild = FSVOLink(0, OutFirstLeafIndex);

		int32 FirstLeafIndex = NodeIndex << 3;
		for (int32 Child = 0; Child < 8; ++Child)
		{
			FSVOLeafNode& LeafNode = OutLeafLayer.AddLeafNode_GetRef();
			int32 LeafIndex = FirstLeafIndex + Child;
			LeafNode.VoxelGrid = BuildingLeafLayer[LeafIndex].VoxelGrid;
			LeafNode.ParentLink = FSVOLink(1, OutLayerOneNodeIndex);
		}
	}
}

void FSVOGenerator::GenerateNeighbours(const int32 LayerIndex)
{
	FSVOLayer& Layer = SVODataRef->GetLayer(LayerIndex);
	int32 MaxCrood = 1 << (NumLayers - LayerIndex);
	
	for (int32 NodeIdx = 0; NodeIdx < Layer.Num(); ++NodeIdx)
	{
		FSVONode& Node = Layer[NodeIdx];
		FIntVector3 Node3D = SVO::MortonToCoord(Node.MortonCode);
		for (int32 Dir = 0; Dir < 6; ++Dir)
		{
			FIntVector3 Delta = SVO::GetDelta(Dir);
			FIntVector3 Neighbour3D = Node3D + Delta;

			if (Neighbour3D.GetMin() >= 0 && Neighbour3D.GetMax() < MaxCrood)
			{
				FMorton NeighbourCode = SVO::CoordToMorton(Neighbour3D.X, Neighbour3D.Y, Neighbour3D.Z);
				int32 NeighbourIdx = SVODataRef->FindNodeInLayer(LayerIndex, NeighbourCode);
				if (NeighbourIdx != INDEX_NONE)
				{
					Node.Neighbours[Dir] = FSVOLink(LayerIndex, NeighbourIdx);
				}
				else
				{
					for (int32 UpperLayer = LayerIndex + 1; UpperLayer <= NumLayers; ++UpperLayer)
					{
						NeighbourCode = SVO::ParentFromAnyChild(NeighbourCode);
						int32 NeighbourParentIdx = SVODataRef->FindNodeInLayer(UpperLayer, NeighbourCode);
						if (NeighbourParentIdx != INDEX_NONE)
						{
							Node.Neighbours[Dir] = FSVOLink(UpperLayer, NeighbourParentIdx);
							break;
						}
					}

					check(Node.Neighbours[Dir] != FSVOLink::INVALID)
				}
			}
			else
			{
				Node.Neighbours[Dir] = FSVOLink::INVALID;
			}
		}
	}
}

FSparseVoxelOctreeGenerator::FSparseVoxelOctreeGenerator(AFlightNavigationData& InDestNavMesh)
	: DestNavData(&InDestNavMesh)
	, SVOData(InDestNavMesh.GetBuildingSVOData().AsShared())
{
	check(DestNavData.IsValid())
	UpdateNavigationBounds();
}

void FSparseVoxelOctreeGenerator::Init()
{
}

bool FSparseVoxelOctreeGenerator::RebuildAll()
{
	GenerateTask = TGraphTask<FSparseVoxelOctreeGenerateTask>::CreateTask().ConstructAndDispatchWhenReady(*this);
	return true;
}

void FSparseVoxelOctreeGenerator::EnsureBuildCompletion()
{
	if (GenerateTask.IsValid() && !GenerateTask->IsComplete())
	{
		FTaskGraphInterface::Get().WaitUntilTaskCompletes(GenerateTask);
		GenerateTask = nullptr;
		DestNavData->OnOctreeGenerationFinished();
	}
}

void FSparseVoxelOctreeGenerator::CancelBuild()
{
	FNavDataGenerator::CancelBuild();
}

void FSparseVoxelOctreeGenerator::TickAsyncBuild(float DeltaSeconds)
{
	if (GenerateTask.IsValid() && GenerateTask->IsComplete())
	{
		GenerateTask = nullptr;

		DestNavData->OnOctreeGenerationFinished();
	}
}

void FSparseVoxelOctreeGenerator::OnNavigationBoundsChanged()
{
	UpdateNavigationBounds();
}

int32 FSparseVoxelOctreeGenerator::GetNumRemaningBuildTasks() const
{
	return GenerateTask.IsValid() ? 1 : 0;
}

int32 FSparseVoxelOctreeGenerator::GetNumRunningBuildTasks() const
{
	return (GenerateTask.IsValid() && !GenerateTask->IsComplete()) ? 1 : 0;
}

void FSparseVoxelOctreeGenerator::UpdateNavigationBounds()
{
	const UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (NavSys)
	{
		FBox BoundsSum(ForceInit);
		if (DestNavData.IsValid())
		{
			TArray<FBox> SupportedBounds;
			NavSys->GetNavigationBoundsForNavData(*DestNavData, SupportedBounds);
			InclusionBounds.Reset(SupportedBounds.Num());

			for (const FBox& Box : SupportedBounds)
			{
				InclusionBounds.Add(Box);
				BoundsSum += Box;
			}

#if WITH_EDITORONLY_DATA
			DestNavData->UpdateEditorData();
#endif
		}
		TotalBounds = BoundsSum;
	}
	else
	{
		TotalBounds = FBox(ForceInit);
	}
}

