// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NavigationData.h"
#include "SparseVoxelOctreeQuery.h"
#include "SparseVoxelOctreeTypes.h"
#include "FlightNavigationData.generated.h"

struct FFlightNavigationPath;
class FSparseVoxelOctreeGenerator;
struct FSparseVoxelOctreeGraph;
struct FSVOPathfindingGraph;

UCLASS()
class FLIGHTNAVIGATION_API AFlightNavigationData : public ANavigationData
{
	GENERATED_BODY()

public:
	AFlightNavigationData();
	
	// Size in Unreal Units (cm) of the smallest details that can be captured.
	UPROPERTY(EditAnywhere, Category = Generation, Config, Meta = (ClampMin = "1", UIMin = "1"))
	double MaxVoxelSize;

#if WITH_EDITORONLY_DATA
	// Actual side length of the smallest voxels. Display only.
	UPROPERTY(VisibleAnywhere, Category = Generation, Transient)
	double ActualVoxelSize;

	// Allow octree visualisation in PIE or Game World. WARNING: Causes performance hit when dynamically rebuilding
	UPROPERTY(EditAnywhere, Category = Display, meta=(DisplayAfter="bEnableDrawing"))
	uint32 bAllowDrawingInGameWorld: 1;

	// Draw NavPath when queried. Editor only.
	UPROPERTY(EditAnywhere, Category = Display, meta=(DisplayAfter="bEnableDrawing"))
	uint32 bDrawDebugPaths: 1;
#endif // WITH_EDITORONLY_DATA

	// Whether to draw the Nodes of the Octree.
	UPROPERTY(EditAnywhere, Category = Display, meta=(EditCondition="bEnableDrawing", DisplayAfter="bEnableDrawing"))
	uint32 bDrawOctreeNodes: 1;
	
	// Draw only Nodes that overlap geometry.
	UPROPERTY(EditAnywhere, Category = Display, meta=(EditCondition="bEnableDrawing && bDrawOctreeNodes", DisplayAfter="bEnableDrawing"))
	uint32 bDrawOnlySpecifiedLayer: 1;
	
	// Draw only Nodes that overlap geometry.
	UPROPERTY(EditAnywhere, Category = Display, meta=(EditCondition="bDrawOnlySpecifiedLayer", DisplayAfter="bDrawOnlySpecifiedLayer"))
	uint32 bDrawOnlyOnlyOverlappedNodes: 1;

	// Draw only Nodes that overlap geometry.
	UPROPERTY(EditAnywhere, Category = Display, meta=(EditCondition="bDrawOnlySpecifiedLayer", ClampMin = "1", ClampMax = "15", DisplayAfter="bDrawOnlySpecifiedLayer"))
	int32 SpecifiedLayerBeDrawn = 1;

	// Whether to draw the SubNodes of the Octree.
	UPROPERTY(EditAnywhere, Category = Display, meta=(EditCondition="bEnableDrawing", DisplayAfter="bEnableDrawing"))
	uint32 bDrawOctreeSubNodes: 1;

	// Draw only SubNodes that overlap geometry.
	UPROPERTY(EditAnywhere, Category = Display, meta=(EditCondition="bEnableDrawing && bDrawOctreeSubNodes", DisplayAfter="bEnableDrawing"))
	uint32 bDrawOnlyOverlappedSubNodes: 1;

	/** Radius of smallest agent to traverse this navmesh */
	UPROPERTY(EditAnywhere, Category = Generation, Config, meta = (ClampMin = "0.0"))
	float AgentRadius;

	// If not defined by a FlyingObject, use these pathfinding query settings.
	UPROPERTY(EditAnywhere, Category = Pathfinding, Config)
	FSparseVoxelOctreeQuery DefaultQuerySettings;

	FORCEINLINE FSVOData& GetSVOData() { return SVOData.Get(); }
	FORCEINLINE const FSVOData& GetSVOData() const { return SVOData.Get(); }
	FORCEINLINE FSVOData& GetBuildingSVOData() { return BuildingSVOData.Get(); }
	FORCEINLINE const FSVOData& GetBuildingSVOData() const { return BuildingSVOData.Get(); }

	mutable FRWLock SVODataLock;
	
	virtual void Serialize(FArchive& Ar) override;

#if WITH_EDITORONLY_DATA
	virtual void UpdateEditorData();
#endif // WITH_EDITOR
	void UpdateDrawing();
	
	void RequestDrawingUpdate(bool bForce = false);

protected:
	FSVODataRef SVOData;
	FSVODataRef BuildingSVOData;

	uint32 SVODataVersion;

	TUniquePtr<const FSparseVoxelOctreeGraph> SVOGraph;
	TUniquePtr<FSVOPathfindingGraph> PathfindingGraph;

	void UpdateSVOData();
	
public:
	virtual UPrimitiveComponent* ConstructRenderingComponent() override;
	
	virtual FBox GetBounds() const override { return GetFlightBounds(); }

	virtual void ConditionalConstructGenerator() override;

	virtual void OnOctreeGenerationFinished();

	FVector GetAdjustedPathPoint(const FVector& PathPoint, const float AgentHalfHeight) const;
	
	static FPathFindingResult FindPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query);
	static bool TestPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query, int32* NumVisitedNodes = nullptr);
	static bool SparseVoxelOctreeRaycast(const ANavigationData* Self, const FVector& RayStart, const FVector& RayEnd, FVector& HitLocation, FSharedConstNavQueryFilter QueryFilter, const UObject* Querier);

	virtual void SetConfig(const FNavDataConfig& Src) override;

	FBox GetFlightBounds() const;

protected:
	virtual FSparseVoxelOctreeGenerator* CreateGeneratorInstance();

#if WITH_EDITORONLY_DATA
	// Line batcher for drawing debug paths
	UPROPERTY(Transient)
	TObjectPtr<class UFlightPathBatchComponent> PathBatcher;

	friend FFlightNavigationPath;
#endif // WITH_EDITORONLY_DATA
};
