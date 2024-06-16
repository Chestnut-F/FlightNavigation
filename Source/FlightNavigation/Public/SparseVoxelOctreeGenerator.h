// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SparseVoxelOctreeTypes.h"
#include "FlightNavigationData.h"
#include "AI/NavDataGenerator.h"

class UNavigationSystemV1;
class AFlightNavigationData;
class FNavigationOctree;
class FSVOLeafGenerator;
class FSVOGenerator;
class FSparseVoxelOctreeGenerator;

struct FSVORawGeometryElement
{
	// Instance geometry, in unreal coords
	TArray<FCoord>		GeomCoords;
	TArray<int32>		GeomIndices;

	// Per instance transformations in unreal coords
	// When empty, geometry is in world space
	TArray<FTransform>	PerInstanceTransform;
	
	// Tight AABB around Per instance geometry element 
	TArray<FBox> PerInstanceBounds;
};

/**
 * 
 */
class FSVOLeafGenerator : public FNoncopyable
{
public:
	FLIGHTNAVIGATION_API FSVOLeafGenerator(FSVOGenerator& InParentGenerator,
		const FBox& InRootBounds, const FMorton InWorkerIdx, const int32 InNodeIdx, const int32 InFirstLeafIdx);
	
	FLIGHTNAVIGATION_API void DoTask();

	const AFlightNavigationData* GetOwner() const;
	FORCEINLINE class UWorld* GetWorld() const;
	
protected:
	void PrepareGeometrySources();
	void GatherGeometryFromSources();
	
	void ValidateAndAppendGeometry(const FNavigationRelevantData& ElementData);
	void AppendGeometry(const FNavigationRelevantData& ElementData, const FNavDataPerInstanceTransformDelegate& InTransformsDelegate);
	
	void RasteriseLayerOne();
	void RasteriseLeafNode(FSVOLeafNode& LeafNode, const FVector& LeafCentre, const FCoord SubNodeOffset, const FVector& SubNodeExtent);

	bool NodeOverlapRawGeometry(const FVector& Centre, const FVector& Extent);
	bool NodeOverlapGeometry(const FSVORawGeometryElement& Geometry, const FVector& Centre, const FVector& Extent);

protected:
	friend class FSVOGenerator;
	friend class FSparseVoxelOctreeLeafGenerateTask;
	
	FSVOGenerator& ParentGeneratorRef;
	FSVODataRef SVODataRef;

	FBox RootBounds;
	FBox RootBoundExpandedForAgent;
	
	TWeakObjectPtr<UNavigationSystemV1> NavSystem;

	TNavStatArray<TSharedRef<FNavigationRelevantData, ESPMode::ThreadSafe> > NavigationRelevantData;
	TArray<FSVORawGeometryElement> RawGeometry;
	
	FMorton WorkerIdx;
	const int32 NodeIdx;
	const int32 FirstLeafIdx;
	FSVOLayer& LayerOne;
	FSVOLeafLayer& LeafLayer;
};

/**
 * 
 */
class FSparseVoxelOctreeLeafGenerateTask
{
public:
	FSparseVoxelOctreeLeafGenerateTask(FSVOGenerator& InParentGenerator, const FBox& InRootBounds,
		const FMorton InWorkerIdx, const int32 InNodeIdx, const int32 InFirstLeafIdx)
		: LeafGenerator(new FSVOLeafGenerator(InParentGenerator, InRootBounds, InWorkerIdx, InNodeIdx, InFirstLeafIdx))
	{
	}
	
	void DoTask(ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent)
	{
		LeafGenerator->DoTask();
		// UE_LOG(LogFlightNavigation, Display, TEXT("Sparse Voxel Octree Subnode Generate Task [%llu]. Centre: [%f, %f, %f]"), LeafGenerator->WorkerIdx,
		// 	LeafGenerator->RootBounds.GetCenter().X, LeafGenerator->RootBounds.GetCenter().Y, LeafGenerator->RootBounds.GetCenter().Z);
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FSparseVoxelOctreeSubNodeGenerateTask, STATGROUP_TaskGraphTasks);
	}
	
	static ENamedThreads::Type GetDesiredThread() { return ENamedThreads::AnyThread; }
	
	static ESubsequentsMode::Type GetSubsequentsMode() { return ESubsequentsMode::TrackSubsequents; }

private:
	TUniquePtr<FSVOLeafGenerator> LeafGenerator;
};

/**
 * 
 */
class FSVOGenerator : public FNoncopyable
{
public:
	FSVOGenerator(FSparseVoxelOctreeGenerator& InGenerator);
	
	FLIGHTNAVIGATION_API void DoTask();

	const AFlightNavigationData* GetOwner() const;
	FORCEINLINE class UWorld* GetWorld() const;

protected:
	void RasteriseLayer(const int32 LayerIndex) const;
	
	void ExcludeLeafNodes(FSVOLayer& OutLayerOne, FSVOLeafLayer& OutLeafLayer);

	void GenerateNeighbours(const int32 LayerIndex);
	
protected:
	friend class FSVOLeafGenerator;
	
	FSparseVoxelOctreeGenerator& ParentGeneratorRef;
	FSVODataRef SVODataRef;

	int32 NumLayers;
	FGraphEventArray RunningTasks;

	FSVOLayer BuildingLayerOne;
	FSVOLeafLayer BuildingLeafLayer;
};

/**
 * 
 */
class FSparseVoxelOctreeGenerateTask
{
public:
	FSparseVoxelOctreeGenerateTask(FSparseVoxelOctreeGenerator& InGenerator)
		: SVOGenerator(new FSVOGenerator(InGenerator))
	{
	}
	
	void DoTask(ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent)
	{
		SVOGenerator->DoTask();
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FSparseVoxelOctreeGenerateTask, STATGROUP_TaskGraphTasks);
	}
	
	static ENamedThreads::Type GetDesiredThread() { return ENamedThreads::AnyThread; }
	
	static ESubsequentsMode::Type GetSubsequentsMode() { return ESubsequentsMode::TrackSubsequents; }
	
protected:
	TUniquePtr<FSVOGenerator> SVOGenerator;
};

/**
 * 
 */
class FSparseVoxelOctreeGenerator : public FNavDataGenerator
{
public:
	FLIGHTNAVIGATION_API FSparseVoxelOctreeGenerator(AFlightNavigationData& InDestNavMesh);

private:
	/** Prevent copying. */
	FSparseVoxelOctreeGenerator(FSparseVoxelOctreeGenerator const& NoCopy) { check(0); };
	FSparseVoxelOctreeGenerator& operator=(FSparseVoxelOctreeGenerator const& NoCopy) { check(0); return *this; }
	
public:
	FLIGHTNAVIGATION_API virtual void Init();

	FLIGHTNAVIGATION_API virtual bool RebuildAll() override;
	FLIGHTNAVIGATION_API virtual void EnsureBuildCompletion() override;
	FLIGHTNAVIGATION_API virtual void CancelBuild() override;
	FLIGHTNAVIGATION_API virtual void TickAsyncBuild(float DeltaSeconds) override;
	FLIGHTNAVIGATION_API virtual void OnNavigationBoundsChanged() override;

	FLIGHTNAVIGATION_API virtual int32 GetNumRemaningBuildTasks() const override;
	FLIGHTNAVIGATION_API virtual int32 GetNumRunningBuildTasks() const override;

	const AFlightNavigationData* GetOwner() const { return DestNavData.Get(); }
	FORCEINLINE class UWorld* GetWorld() const { return DestNavData->GetWorld(); }

protected:
	void UpdateNavigationBounds();

	void OnLeafGenerateFinished(int32 WorkIndex, FSVONode&& Node)
	{
		LayerOne[WorkIndex] = MoveTemp(Node);
	}

protected:
	friend class FSVOGenerator;
	friend class FSVOLeafGenerator;
	friend class AFlightNavigationData;

	FBox TotalBounds;
	TNavStatArray<FBox> InclusionBounds;
	
	TWeakObjectPtr<AFlightNavigationData> DestNavData;
	FSVODataRef SVOData;
	
	FGraphEventArray RunningNodeGenerateTasks;

	FGraphEventRef GenerateTask;

	TArray<FSVONode> LayerOne;
};


