// Fill out your copyright notice in the Description page of Project Settings.


#include "SparseVoxelOctreeGraph.h"


void FSparseVoxelOctreeGraph::GetNeighbours(const FSVOLink& NodeRef, TArray<FSVOLink>& Neighbours) const
{
	const int32 LayerIndex = NodeRef.GetLayerIndex();
	const int32 NodeIndex = NodeRef.GetNodeIndex();

	if (LayerIndex == 0)
	{
		const FSVOLeafNode& Node = SVOData->LeafLayer[NodeIndex];
		if (Node.IsLeafFree())
		{
			for (int32 Direction = 0; Direction < 6; ++Direction)
			{
				FSVOLink LeafNeighbour = GetLeafNeighbour(NodeRef, Node, Direction);
				if (!LeafNeighbour.IsValid())
				{
					continue;
				}
				
				if (LeafNeighbour.GetLayerIndex() != 0)
				{
					Neighbours.Add(LeafNeighbour);
					continue;
				}
				
				const int32 LeafNeighbourNodeIndex = LeafNeighbour.GetNodeIndex();
				const FSVOLeafNode& LeafNeighbourNode = SVOData->LeafLayer[LeafNeighbourNodeIndex];
		
				if (LeafNeighbourNode.IsLeafFree())
				{
					Neighbours.Add(LeafNeighbour);
					continue;
				}

				if (!LeafNeighbourNode.IsLeafWithCollision())
				{
					SubdivideNeighbour(LeafNeighbour, Direction, Neighbours);
				}
			}
		}
		else
		{
			for (int32 Direction = 0; Direction < 6; ++Direction)
			{
				FSVOLink SubNodeNeighbour = GetSubNodeNeighbour(NodeRef, Node, Direction);
				if (!SubNodeNeighbour.IsValid())
				{
					continue;
				}

				Neighbours.Add(SubNodeNeighbour);
			}
		}
	}
	else
	{
		const FSVONode& Node = SVOData->GetLayer(LayerIndex).GetNode(NodeIndex);
		
		for (int32 Direction = 0; Direction < 6; ++Direction)
		{
			const FSVOLink& NeighbourLink = Node.Neighbours[Direction];
			if (NeighbourLink.IsValid())
			{
				SubdivideNeighbour(NeighbourLink, Direction, Neighbours);
			}
		}
	}
}

void FSparseVoxelOctreeGraph::SubdivideNeighbour(const FSVOLink& NeighbourNodeRef, int32 Direction,
	TArray<FSVOLink>& Neighbours) const
{
	const int32 NeighbourLayerIndex = NeighbourNodeRef.GetLayerIndex();
	const int32 NeighbourNodeIndex = NeighbourNodeRef.GetNodeIndex();

	if (NeighbourLayerIndex == 0)
	{
		const FSVOLeafNode& NeighbourNode = SVOData->LeafLayer[NeighbourNodeIndex];
		
		if (NeighbourNode.IsLeafFree())
		{
			Neighbours.Add(NeighbourNodeRef);
			return;
		}

		for (FMorton SubNodeMorton : SVO::SubNodeMortonForDirection[Direction])
		{
			if (NeighbourNode.IsVoxelFree(SubNodeMorton))
			{
				Neighbours.Emplace(FSVOLink(0, NeighbourNodeIndex, SubNodeMorton));
			}
		}
		return;
	}

	const FSVONode& NeighbourNode = SVOData->GetLayer(NeighbourLayerIndex).GetNode(NeighbourNodeIndex);
	
	if (!NeighbourNode.HasChildren())
	{
		Neighbours.Add(NeighbourNodeRef);
		return;
	}
	
	const FSVOLink& FirstChild = NeighbourNode.FirstChild;
	
	SubdivideNeighbour(FirstChild + SVO::NeighbourMortonOffset[Direction][0], Direction, Neighbours);
	SubdivideNeighbour(FirstChild + SVO::NeighbourMortonOffset[Direction][1], Direction, Neighbours);
	SubdivideNeighbour(FirstChild + SVO::NeighbourMortonOffset[Direction][2], Direction, Neighbours);
	SubdivideNeighbour(FirstChild + SVO::NeighbourMortonOffset[Direction][3], Direction, Neighbours);
}

FSVOLink FSparseVoxelOctreeGraph::GetLeafNeighbour(const FSVOLink& NodeRef, const FSVOLeafNode& LeafNode,
	const int32 Direction) const
{
	const int32 ParentLayerIndex = LeafNode.ParentLink.GetLayerIndex();
	const int32 ParentNodeIndex = LeafNode.ParentLink.GetNodeIndex();
	const FSVONode& ParentNode = SVOData->GetLayer(ParentLayerIndex).GetNode(ParentNodeIndex);
	
	const int32 NodeIndex = NodeRef.GetNodeIndex();
	FIntVector3 Node3D = SVO::MortonToCoord(NodeIndex - ParentNode.FirstChild.GetNodeIndex());
	check(Node3D.GetMin() >= -1)
	check(Node3D.GetMax() <= 2)
		
	FIntVector3 Delta = SVO::GetDelta(Direction);
	FIntVector3 Neighbour3D = Node3D + Delta;

	if (Neighbour3D.GetMin() >= 0 && Neighbour3D.GetMax() < 2)
	{
		FMorton NeighbourNodeIndex = SVO::CoordToMorton(Neighbour3D.X, Neighbour3D.Y, Neighbour3D.Z);
		return ParentNode.FirstChild + NeighbourNodeIndex;
	}

	const FSVOLink& ParentNeighbour = ParentNode.Neighbours[Direction];
	if (!ParentNeighbour.IsValid())
	{
		return FSVOLink::INVALID;
	}
		
	const int32 ParentNeighbourLayerIndex = ParentNeighbour.GetLayerIndex();
	const int32 ParentNeighbourNodeIndex = ParentNeighbour.GetNodeIndex();
		
	const FSVONode& ParentNeighbourNode = SVOData->GetLayer(ParentNeighbourLayerIndex).GetNode(ParentNeighbourNodeIndex);
		
	if (!ParentNeighbourNode.HasChildren())
	{
		return ParentNeighbour;
	}

	FMorton NeighbourNodeIndex = SVO::CoordToMorton(Neighbour3D.X & 0b1, Neighbour3D.Y & 0b1, Neighbour3D.Z & 0b1);
	return ParentNeighbourNode.FirstChild + NeighbourNodeIndex;
}

FSVOLink FSparseVoxelOctreeGraph::GetSubNodeNeighbour(const FSVOLink& NodeRef, const FSVOLeafNode& LeafNode,
	const int32 Direction) const
{
	const int32 SubNodeIndex = NodeRef.GetSubNodeIndex();
	FIntVector3 SubNode3D = SVO::MortonToCoord(SubNodeIndex);
	check(SubNode3D.GetMin() >= -1)
	check(SubNode3D.GetMax() <= 5)
		
	FIntVector3 Delta = SVO::GetDelta(Direction);
	FIntVector3 Neighbour3D = SubNode3D + Delta;

	if (Neighbour3D.GetMin() >= 0 && Neighbour3D.GetMax() < 4)
	{
		FMorton NeighbourSubNodeIndex = SVO::CoordToMorton(Neighbour3D.X, Neighbour3D.Y, Neighbour3D.Z);
		if (LeafNode.IsVoxelFree(NeighbourSubNodeIndex))
		{
			return FSVOLink(0, NodeRef.GetNodeIndex(), NeighbourSubNodeIndex);
		}

		return FSVOLink::INVALID;
	}
		
	const FSVOLink& ParentNeighbour = GetLeafNeighbour(NodeRef, LeafNode, Direction);
		
	if (!ParentNeighbour.IsValid())
	{
		return FSVOLink::INVALID;
	}
		
	const int32 ParentNeighbourLayerIndex = ParentNeighbour.GetLayerIndex();
		
	if (ParentNeighbourLayerIndex != 0)
	{
		return ParentNeighbour;
	}
		
	const int32 ParentNeighbourNodeIndex = ParentNeighbour.GetNodeIndex();
	const FSVOLeafNode& ParentNeighbourNode = SVOData->LeafLayer[ParentNeighbourNodeIndex];
		
	if (ParentNeighbourNode.IsLeafFree())
	{
		return ParentNeighbour;
	}

	FMorton NeighbourSubNodeIndex = SVO::CoordToMorton(Neighbour3D.X & 0b11, Neighbour3D.Y & 0b11, Neighbour3D.Z & 0b11);
	if (ParentNeighbourNode.IsVoxelFree(NeighbourSubNodeIndex))
	{
		return FSVOLink(0, ParentNeighbourNodeIndex, NeighbourSubNodeIndex);
	}

	return FSVOLink::INVALID;
}

bool FSVOPathfindingGraph::ProcessSingleNodeLazyThetaStar(const FGraphNodeRef& EndNodeRef,
                                                          const FSparseVoxelOctreeQuery& Query, int32& OutBestNodeIndex, FVector::FReal& OutBestNodeCost)
{
	return false;
}

bool FSVOPathfindingGraph::ProcessSingleNodeThetaStar(const FGraphNodeRef& EndNodeRef, const FSparseVoxelOctreeQuery& Query,
	int32& OutBestNodeIndex, FVector::FReal& OutBestNodeCost)
{
	return false;
}

bool FSVOPathfindingGraph::ProcessSingleNodeAStar(const FGraphNodeRef& EndNodeRef, const FSparseVoxelOctreeQuery& Query,
	int32& OutBestNodeIndex, FVector::FReal& OutBestNodeCost)
{
	// Pop next best node and put it on closed list
	const int32 ConsideredNodeIndex = OpenList.PopIndex();
	FSearchNode& ConsideredNodeUnsafe = NodePool[ConsideredNodeIndex];
	ConsideredNodeUnsafe.MarkClosed();

	// We're there, store and move to result composition
	if (ConsideredNodeUnsafe.NodeRef == EndNodeRef)
	{
		OutBestNodeIndex = ConsideredNodeUnsafe.SearchNodeIndex;
		OutBestNodeCost = 0.;
		return false;
	}

	const float HeuristicScale = Query.GetHeuristicScale();

	// consider every neighbor of BestNode
	const FGraphNodeRef CurrentNodeRef = ConsideredNodeUnsafe.NodeRef;
	TArray<FGraphNodeRef> Neighbours;
	GetNeighbours(Graph, CurrentNodeRef, Neighbours);
	const int32 NeighbourCount = Neighbours.Num();

	for (int32 NeighbourNodeIndex = 0; NeighbourNodeIndex < NeighbourCount; ++NeighbourNodeIndex)
	{
		const FGraphNodeRef NeighbourRef = Neighbours[NeighbourNodeIndex];
		
		// invalid neigbour check
		if (Graph.IsValidRef(NeighbourRef) == false)
		{
			// skipping invalid neighbours
			continue;
		}

		// validate and sanitize
		if (NeighbourRef == NodePool[ConsideredNodeIndex].ParentRef
			|| NeighbourRef == NodePool[ConsideredNodeIndex].NodeRef
			|| IsTraversalAllowed(Query, NodePool[ConsideredNodeIndex].NodeRef, NeighbourRef) == false)
		{
			continue;
		}

		// check against max search nodes limit
		FSearchNode* ExistingNeighbourNode = nullptr;
		if(HasReachMaxSearchNodes(Query, (uint32)NodePool.Num()))
		{
			// let's skip this one if it is not already in the NodePool
			ExistingNeighbourNode = NodePool.Find(NeighbourRef);
			if (!ExistingNeighbourNode)
			{
				continue;
			}
		}
		FSearchNode& NeighbourNode = ExistingNeighbourNode ? *ExistingNeighbourNode : NodePool.FindOrAdd(NeighbourRef);

		// check condition to avoid search of closed nodes even if they could have lower cost
		if (NeighbourNode.IsClosed())
		{
			continue;
		}

		// calculate cost and heuristic.
		const FVector::FReal NewTraversalCost = GetTraversalCost(Query, NodePool[ConsideredNodeIndex].NodeRef, NeighbourRef) + NodePool[ConsideredNodeIndex].TraversalCost;
		const FVector::FReal NewHeuristicCost = (NeighbourNode.NodeRef != EndNodeRef)
			? (GetHeuristicCost(Query, NeighbourRef, EndNodeRef) * HeuristicScale)
			: 0.;
		const FVector::FReal NewTotalCost = NewTraversalCost + NewHeuristicCost;

		// check against cost limit
		if (HasExceededCostLimit(Query, NewTotalCost))
		{
			continue;
		}

		// check if this is better then the potential previous approach
		if (NewTotalCost >= NeighbourNode.TotalCost)
		{
			// if not, skip
			continue;
		}

		// fill in
		NeighbourNode.TraversalCost = NewTraversalCost;
		NeighbourNode.TotalCost = NewTotalCost;
		NeighbourNode.ParentRef = NodePool[ConsideredNodeIndex].NodeRef;
		NeighbourNode.ParentNodeIndex = NodePool[ConsideredNodeIndex].SearchNodeIndex;
		NeighbourNode.MarkNotClosed();

		if (NeighbourNode.IsOpened() == false)
		{
			OpenList.Push(NeighbourNode);
		}
		else
		{
			OpenList.Modify(NeighbourNode);
		}

		// in case there's no path let's store information on
		// "closest to goal" node
		// using Heuristic cost here rather than Traversal or Total cost
		// since this is what we'll care about if there's no solution - this node 
		// will be the one estimated-closest to the goal
		if (NewHeuristicCost < OutBestNodeCost)
		{
			OutBestNodeCost = NewHeuristicCost;
			OutBestNodeIndex = NeighbourNode.SearchNodeIndex;
		}
	}

	return true;
}

ENavigationQueryResult::Type FSVOPathfindingGraph::FindPath(const FVector& StartLocation, const FVector& EndLocation,
	const FSparseVoxelOctreeQuery& Query, FFlightNavigationPath& NavPath)
{
	FGraphNodeRef StartNodeRef = GetNodeRefForLocation(Graph, StartLocation);
	FGraphNodeRef EndNodeRef = GetNodeRefForLocation(Graph, EndLocation);

	if (!StartNodeRef.IsValid() || !EndNodeRef.IsValid())
	{
		return ENavigationQueryResult::Invalid;
	}
		
	TArray<FNavPathPoint>& PathPoints = NavPath.GetPathPoints();
	if (StartNodeRef == EndNodeRef)
	{
		if ((StartLocation - EndLocation).IsNearlyZero())
		{
			PathPoints.Add(FNavPathPoint(EndLocation, EndNodeRef.AsNavNodeRef()));
		}
		else
		{
			PathPoints.Add(FNavPathPoint(StartLocation, StartNodeRef.AsNavNodeRef()));
			PathPoints.Add(FNavPathPoint(EndLocation, EndNodeRef.AsNavNodeRef()));
		}

		return ENavigationQueryResult::Success;
	}

	TArray<FGraphNodeRef> NodePath;
	EFPathfindingResult Result = FindPathImpl(StartNodeRef, EndNodeRef, Query, NodePath);

	return PostProcessPath(Query, NodePath, NavPath, Result);
}

EFPathfindingResult FSVOPathfindingGraph::FindPathImpl(const FGraphNodeRef& StartNodeRef, const FGraphNodeRef& EndNodeRef,
                                                       const FSparseVoxelOctreeQuery& Query, TArray<FGraphNodeRef>& OutPath)
{
	if (!Graph.IsValidRef(StartNodeRef) || !Graph.IsValidRef(EndNodeRef))
	{
		return EFPathfindingResult::SearchFail;
	}

	if (StartNodeRef == EndNodeRef)
	{
		return EFPathfindingResult::SearchSuccess;
	}

	OpenList.Reset();
	NodePool.Reset();

	int32 BestNodeIndex = INDEX_NONE;
	FVector::FReal BestNodeCost = TNumericLimits<FVector::FReal>::Max();
	{
		FSearchNode& StartPoolNode = NodePool.FindOrAdd(StartNodeRef);
		StartPoolNode.TraversalCost = 0.f;
		StartPoolNode.TotalCost = GetHeuristicCost(Query, StartPoolNode.NodeRef, EndNodeRef) * Query.GetHeuristicScale();
		
		OpenList.Push(StartPoolNode);

		BestNodeIndex = StartPoolNode.SearchNodeIndex;
		BestNodeCost = StartPoolNode.TotalCost;
	}

	const int32 StartPoolSearchNodeIndex = NodePool[BestNodeIndex].SearchNodeIndex;
	const FGraphNodeRef StartPoolNodeRef = NodePool[BestNodeIndex].NodeRef;
	
	typedef bool (FSVOPathfindingGraph::*ProcessSingleNodeFunc)(const FGraphNodeRef&, const FSparseVoxelOctreeQuery&, int32&, FVector::FReal&);
	ProcessSingleNodeFunc ProcessSingleNode = nullptr;

	ESVOPathfindingAlgorithm PathfindingAlgorithm = Query.GetPathfindingAlgorithm();
	switch (PathfindingAlgorithm)
	{
	case ESVOPathfindingAlgorithm::AStar:
		ProcessSingleNode = &FSVOPathfindingGraph::ProcessSingleNodeAStar;
		break;
	case ESVOPathfindingAlgorithm::LazyThetaStar:
		ProcessSingleNode = &FSVOPathfindingGraph::ProcessSingleNodeLazyThetaStar;
		break;
	case ESVOPathfindingAlgorithm::ThetaStar:
		ProcessSingleNode = &FSVOPathfindingGraph::ProcessSingleNodeThetaStar;
		break;
	}

	bool bProcessNodes = true;
	while (OpenList.Num() > 0 && bProcessNodes)
	{
		bProcessNodes = (this->*ProcessSingleNode)(EndNodeRef, Query, BestNodeIndex, BestNodeCost);
	}

	EFPathfindingResult Result = EFPathfindingResult::SearchSuccess;
	if (BestNodeCost != 0.f)
	{
		Result = EFPathfindingResult::GoalUnreachable;
	}
	
	if (Result == EFPathfindingResult::SearchSuccess)
	{
		int32 SearchNodeIndex = BestNodeIndex;
		int32 PathLength = ShouldIncludeStartNodeInPath(Query) && BestNodeIndex != StartPoolSearchNodeIndex ? 1 : 0;
		do
		{
			++PathLength;
			SearchNodeIndex = NodePool[SearchNodeIndex].ParentNodeIndex;
		}
		while (NodePool.IsValidIndex(SearchNodeIndex) && NodePool[SearchNodeIndex].NodeRef != StartPoolNodeRef && ensure(PathLength < FPathfindingGraphDefaultPolicy::FatalPathLength));

		if (PathLength >= FPathfindingGraphDefaultPolicy::FatalPathLength)
		{
			return EFPathfindingResult::InfiniteLoop;
		}

		OutPath.Reserve(PathLength);
		OutPath.AddZeroed(PathLength);
		
		SearchNodeIndex = BestNodeIndex;
		int32 ResultNodeIndex = PathLength - 1;
		do
		{
			OutPath[ResultNodeIndex] = NodePool[SearchNodeIndex].NodeRef;
			SearchNodeIndex = NodePool[SearchNodeIndex].ParentNodeIndex;
			ResultNodeIndex--;
		}
		while (ResultNodeIndex >= 0);
	}
	
	return Result;
}

ENavigationQueryResult::Type FSVOPathfindingGraph::PostProcessPath(const FSparseVoxelOctreeQuery& Query,
	TArray<FGraphNodeRef>& NodePath, FFlightNavigationPath& NavPath, const EFPathfindingResult& PathResult) const
{
	TArray<FNavPathPoint>& PathPoints = NavPath.GetPathPoints();

	ENavigationQueryResult::Type Result = ENavigationQueryResult::Fail;
		
	if (PathResult == EFPathfindingResult::SearchSuccess)
	{
		for (const FGraphNodeRef& NodeRef : NodePath)
		{
			const FVector PathPoint = GetLocationForNodeRef(Graph, NodeRef);
			PathPoints.Add(FNavPathPoint(PathPoint, NodeRef.AsNavNodeRef()));
		}

		Result = ENavigationQueryResult::Success;
	}
	else if (PathResult == EFPathfindingResult::InfiniteLoop)
	{
		Result = ENavigationQueryResult::Error;
	}

	if (Query.bSmoothPath)
	{
	}

	NavPath.MarkReady();
	return Result;
}

