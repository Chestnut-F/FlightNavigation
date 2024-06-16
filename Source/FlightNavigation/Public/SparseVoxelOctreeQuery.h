#pragma once

#include "CoreMinimal.h"
#include "SparseVoxelOctreeTypes.h"
#include "SparseVoxelOctreeQuery.generated.h"

USTRUCT(BlueprintType)
struct FLIGHTNAVIGATION_API FSparseVoxelOctreeQuery
{
	GENERATED_BODY()

	FSparseVoxelOctreeQuery();
	
public:
	FORCEINLINE void SetNavData(const FSVODataRef& InNavData) { SVOData = InNavData; }

	float GetHeuristicScale() const;

	ESVOPathfindingAlgorithm GetPathfindingAlgorithm() const;
	
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	ESVOPathfindingAlgorithm PathfindingAlgorithm;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	float HeuristicScale;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	bool bUseUnitCost;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	bool bSmoothPath;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	bool bIncludeStartNodeInPath;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	int32 MaxSearchNodes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	double CostLimit;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pathfinding)
	FLinearColor DebugPathColor;

public:
	FCoord GetTraversalCost(const FSVOLink& StartNode, const FSVOLink& EndNode) const;
	
	FCoord GetHeuristicCost(const FSVOLink& StartNode, const FSVOLink& EndNode) const;
	
	bool IsTraversalAllowed(const FSVOLink& StartNode, const FSVOLink& EndNode) const;

	bool ShouldIncludeStartNodeInPath() const;

	int32 GetMaxSearchNodes() const;

	FCoord GetCostLimit() const;

private:
	FSVODataConstPtr SVOData;
};


