#include "SparseVoxelOctreeQuery.h"


FSparseVoxelOctreeQuery::FSparseVoxelOctreeQuery()
	: PathfindingAlgorithm(ESVOPathfindingAlgorithm::AStar)
	, HeuristicScale(1.f)
	, bUseUnitCost(false)
	, bSmoothPath(false)
	, bIncludeStartNodeInPath(true)
	, MaxSearchNodes(TNumericLimits<int32>::Max())
	, CostLimit(TNumericLimits<FVector::FReal>::Max())
	, DebugPathColor(FLinearColor::Green)
{
}


FCoord FSparseVoxelOctreeQuery::GetTraversalCost(const FSVOLink& StartNode, const FSVOLink& EndNode) const
{
	return bUseUnitCost ? 1. :
		(SVOData->GetPositionForLink(StartNode) - SVOData->GetPositionForLink(EndNode)).Size();
}

FCoord FSparseVoxelOctreeQuery::GetHeuristicCost(const FSVOLink& StartNode, const FSVOLink& EndNode) const
{
	return (SVOData->GetPositionForLink(StartNode) - SVOData->GetPositionForLink(EndNode)).Size();
}

float FSparseVoxelOctreeQuery::GetHeuristicScale() const
{
	return HeuristicScale;
}

ESVOPathfindingAlgorithm FSparseVoxelOctreeQuery::GetPathfindingAlgorithm() const
{
	return PathfindingAlgorithm;
}

bool FSparseVoxelOctreeQuery::IsTraversalAllowed(const FSVOLink& StartNode, const FSVOLink& EndNode) const
{
	check(StartNode.IsValid() && EndNode.IsValid())
	return true;
}

bool FSparseVoxelOctreeQuery::ShouldIncludeStartNodeInPath() const
{
	return bIncludeStartNodeInPath;
}

int32 FSparseVoxelOctreeQuery::GetMaxSearchNodes() const
{
	return MaxSearchNodes;
}

FCoord FSparseVoxelOctreeQuery::GetCostLimit() const
{
	return CostLimit;
}


