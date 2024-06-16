// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FlightNavigationPath.h"
#include "SparseVoxelOctreeQuery.h"
#include "SparseVoxelOctreeTypes.h"

struct FPathfindingGraphDefaultPolicy
{
	static const int32 NodePoolSize = 64;
	static const int32 OpenSetSize = 64;
	static const int32 FatalPathLength = 10000;
};

template<typename TGraph>
struct FPathfindingGraphDefaultNode
{
	typedef typename TGraph::FNodeRef FGraphNodeRef;

	const FGraphNodeRef NodeRef;
	FGraphNodeRef ParentRef;
	FVector::FReal TraversalCost;
	FVector::FReal TotalCost;
	int32 SearchNodeIndex;
	int32 ParentNodeIndex;
	uint8 bIsOpened : 1;
	uint8 bIsClosed : 1;

	FORCEINLINE FPathfindingGraphDefaultNode(const FGraphNodeRef& InNodeRef)
		: NodeRef(InNodeRef)
		, ParentRef(FGraphNodeRef::INVALID)
		, TraversalCost(TNumericLimits<FVector::FReal>::Max())
		, TotalCost(TNumericLimits<FVector::FReal>::Max())
		, SearchNodeIndex(INDEX_NONE)
		, ParentNodeIndex(INDEX_NONE)
		, bIsOpened(false)
		, bIsClosed(false)
	{}

	FORCEINLINE void MarkOpened() { bIsOpened = true; }
	FORCEINLINE void MarkNotOpened() { bIsOpened = false; }
	FORCEINLINE void MarkClosed() { bIsClosed = true; }
	FORCEINLINE void MarkNotClosed() { bIsClosed = false; }
	FORCEINLINE bool IsOpened() const { return bIsOpened; }
	FORCEINLINE bool IsClosed() const { return bIsClosed; }
};

enum class EFPathfindingResult
{
	SearchFail,
	SearchSuccess,
	GoalUnreachable,
	InfiniteLoop
};

#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_IMPL( TemplateClass, TemplateClassParameter, ConditionalReturnType, ConditionalFunctionName, DefaultImpl ) \
struct CQuery##ConditionalFunctionName	\
{	\
	template<typename TemplateClass> auto Requires(TemplateClassParameter& Obj) -> decltype(Obj.ConditionalFunctionName());	\
};	\
template <typename TemplateClass> \
static FORCEINLINE decltype(auto) ConditionalFunctionName(TemplateClassParameter & Obj) \
{ \
	if constexpr (TModels_V<CQuery##ConditionalFunctionName, TemplateClass>) \
	{ \
		return Obj.ConditionalFunctionName(); \
	} \
	else \
	{ \
		return (ConditionalReturnType)(DefaultImpl); \
	} \
}
#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION( ConditionalReturnType, ConditionalFunctionName, DefaultImpl )  DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_IMPL( TemplateClass, TemplateClass, ConditionalReturnType, ConditionalFunctionName, DefaultImpl )
#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_CONST( ConditionalReturnType, ConditionalFunctionName, DefaultImpl ) DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_IMPL( TemplateClass, const TemplateClass, ConditionalReturnType, ConditionalFunctionName, DefaultImpl )

#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM_IMPL( TemplateClass, TemplateClassParameter, ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, DefaultImpl) \
struct CQuery##ConditionalFunctionName	\
{	\
	template<typename TemplateClass> auto Requires(TemplateClassParameter& Obj, ConditionalParamType1 Param1) -> decltype(Obj.ConditionalFunctionName(Param1));	\
};	\
template <typename TemplateClass> \
static FORCEINLINE decltype(auto) ConditionalFunctionName(TemplateClassParameter & Obj, ConditionalParamType1 Param1) \
{ \
	if constexpr (TModels_V<CQuery##ConditionalFunctionName, TemplateClass>) \
	{ \
		return Obj.ConditionalFunctionName(Param1); \
	} \
	else \
	{ \
		return (ConditionalReturnType)(DefaultImpl); \
	} \
}
#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM( ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, DefaultImpl) DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM_IMPL( TemplateClass, TemplateClass, ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, DefaultImpl) 
#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM_CONST( ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, DefaultImpl) DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM_IMPL( TemplateClass, const TemplateClass, ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, DefaultImpl) 

#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_IMPL( TemplateClass, TemplateClassParameter, ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, ConditionalParamType2, DefaultImpl) \
struct CQuery##ConditionalFunctionName	\
{	\
	template<typename TemplateClass> auto Requires(TemplateClassParameter& Obj, ConditionalParamType1 Param1, ConditionalParamType2 Param2) -> decltype(Obj.ConditionalFunctionName(Param1,Param2));	\
};	\
template <typename TemplateClass> \
static FORCEINLINE decltype(auto) ConditionalFunctionName(TemplateClassParameter & Obj, ConditionalParamType1 Param1, ConditionalParamType2 Param2) \
{ \
	if constexpr (TModels_V<CQuery##ConditionalFunctionName, TemplateClass>) \
	{ \
		return Obj.ConditionalFunctionName(Param1, Param2); \
	} \
	else \
	{ \
		return (ConditionalReturnType)(DefaultImpl); \
	} \
}
#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS( ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, ConditionalParamType2, DefaultImpl) DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_IMPL( TemplateClass, TemplateClass, ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, ConditionalParamType2, DefaultImpl)
#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_CONST( ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, ConditionalParamType2, DefaultImpl) DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_IMPL(TemplateClass, const TemplateClass, ConditionalReturnType, ConditionalFunctionName, ConditionalParamType1, ConditionalParamType2, DefaultImpl)

#define DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_CUSTOM( ConditionalFunctionName, QueryReturnType, QueryFunctionName, QueryParam, QueryDefaultImpl, QueryImpl) \
struct CQuery##QueryFunctionName	\
{	\
	template<typename TemplateClass> auto Requires(const TemplateClass& Obj) -> decltype(Obj.ConditionalFunctionName());	\
};	\
template <typename TemplateClass> \
static FORCEINLINE QueryReturnType QueryFunctionName(const TemplateClass& Obj, QueryParam) \
{ \
	if constexpr (TModels_V<CQuery##QueryFunctionName, TemplateClass>) \
	{ \
		return QueryImpl; \
	} \
	else \
	{ \
		return QueryDefaultImpl; \
	} \
}

template<typename TGraph, typename Policy = FPathfindingGraphDefaultPolicy, typename TSearchNode = FPathfindingGraphDefaultNode<TGraph>>
struct FPathfindingGraph
{
	typedef typename TGraph::FNodeRef FGraphNodeRef;
	typedef TSearchNode FSearchNode;

	using FNodeArray = TArray<FSearchNode>;
	using FNodeMap = TMap<FGraphNodeRef, int32>;
	using FIndexArray = TArray<int32>;
	
	struct FNodeSorter
	{
		const FNodeArray& NodePool;

		FNodeSorter(const FNodeArray& InNodePool)
			: NodePool(InNodePool)
		{}

		FORCEINLINE bool operator()(const int32 A, const int32 B) const
		{
			return NodePool[A].TotalCost < NodePool[B].TotalCost;
		}
	};

	struct FNodePool : FNodeArray
	{
		typedef FNodeArray Super;
		FNodeMap NodeMap;

		FNodePool()
		{
			Super::Reserve(Policy::NodePoolSize);
			NodeMap.Reserve(FMath::RoundUpToPowerOfTwo(Policy::NodePoolSize / 4));
		}

		FORCEINLINE FSearchNode& Add(const FSearchNode& SearchNode)
		{
			FSearchNode& NewNode = Super::Emplace_GetRef(SearchNode);
			NewNode.SearchNodeIndex = UE_PTRDIFF_TO_INT32(&NewNode - Super::GetData());
			NodeMap.Add(SearchNode.NodeRef, NewNode.SearchNodeIndex);
			return NewNode;
		}

		FORCEINLINE_DEBUGGABLE FSearchNode& FindOrAdd(const FGraphNodeRef NodeRef)
		{
			// first find if node already exist in node map
			const int32 NotInMapIndex = -1;
			int32& Index = NodeMap.FindOrAdd(NodeRef, NotInMapIndex);
			if (Index != NotInMapIndex)
			{
				return (*this)[Index];
			}

			// node not found, add it and setup index in node map
			FSearchNode& NewNode = Super::Emplace_GetRef(NodeRef);
			NewNode.SearchNodeIndex = UE_PTRDIFF_TO_INT32(&NewNode - Super::GetData());
			Index = NewNode.SearchNodeIndex;

			return NewNode;
		}

		FORCEINLINE FSearchNode* Find(const FGraphNodeRef NodeRef)
		{
			const int32* IndexPtr = NodeMap.Find(NodeRef);
			return IndexPtr ? &(*this)[*IndexPtr] : nullptr;
		}

		FORCEINLINE void Reset()
		{
			Super::Reset(Policy::NodePoolSize);
			NodeMap.Reset();
		}
	};

	struct FOpenList : FIndexArray
	{
		typedef FIndexArray Super;
		FNodeArray& NodePool;
		const FNodeSorter& NodeSorter;

		FOpenList(FNodeArray& InNodePool, const FNodeSorter& InNodeSorter)
			: NodePool{ InNodePool }, NodeSorter{ InNodeSorter }
		{
			Super::Reserve(Policy::OpenSetSize);
		}

		void Push(FSearchNode& SearchNode)
		{
			Super::HeapPush(SearchNode.SearchNodeIndex, NodeSorter);
			SearchNode.MarkOpened();
		}

		void Modify(FSearchNode& SearchNode)
		{
			for (int32& NodeIndex : *this)
			{
				if (NodeIndex == SearchNode.SearchNodeIndex)
				{
					AlgoImpl::HeapSiftUp(Super::GetData(), 0, UE_PTRDIFF_TO_INT32(&NodeIndex - Super::GetData()), FIdentityFunctor(), NodeSorter);
					return;
				}
			}
			check(false); // We should never reach here.
		}

		int32 PopIndex()
		{
			int32 SearchNodeIndex = INDEX_NONE;

			// During A* we grow the array as needed and it does not make sense to shrink in the process.
			Super::HeapPop(SearchNodeIndex, NodeSorter, EAllowShrinking::No);
			NodePool[SearchNodeIndex].MarkNotOpened();
			return SearchNodeIndex;
		}
	};

	const TGraph& Graph;
	FNodePool NodePool;
	FNodeSorter NodeSorter;
	FOpenList OpenList;

	// TGraph optionally implemented wrapper methods
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM_CONST(bool, IsValidRef, const FGraphNodeRef&, Obj.IsValidRef(Param1));
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_CONST(void, GetNeighbours, const FGraphNodeRef&, TArray<FGraphNodeRef>&, Obj.GetNeighbours(Param1,Param2));
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM_CONST(FGraphNodeRef, GetNodeRefForLocation, const FVector&, Obj.GetNodeRefForLocation(Param1));
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_1PARAM_CONST(FVector, GetLocationForNodeRef, const FGraphNodeRef&, Obj.GetLocationForNodeRef(Param1));
	
	// TQueryFilter optionally implemented wrapper methods
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_CONST(FVector::FReal, GetTraversalCost, const FGraphNodeRef&, const FGraphNodeRef&, Obj.GetTraversalCost(Param1, Param2));
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_CONST(FVector::FReal, GetHeuristicCost, const FGraphNodeRef&, const FGraphNodeRef&, Obj.GetHeuristicCost(Param1, Param2));
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_2PARAMS_CONST(bool, IsTraversalAllowed, const FGraphNodeRef&, const FGraphNodeRef&, Obj.IsTraversalAllowed(Param1, Param2));
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_CONST(bool, ShouldIncludeStartNodeInPath, false);
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_CONST(FVector::FReal, GetCostLimit, TNumericLimits<FVector::FReal>::Max());
	// Custom methods implemented over TQueryFilter optionally implemented methods
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_CUSTOM(GetMaxSearchNodes, bool, HasReachMaxSearchNodes, int32 NodeCount, false, NodeCount >= Obj.GetMaxSearchNodes());
	DECLARE_OPTIONALLY_IMPLEMENTED_TEMPLATE_CLASS_FUNCTION_CUSTOM(GetCostLimit, bool, HasExceededCostLimit, FVector::FReal Cost, false, Cost > Obj.GetCostLimit());

	FPathfindingGraph(const TGraph& InGraph)
		: Graph{ InGraph }, NodeSorter{ NodePool }, OpenList{ NodePool, NodeSorter }
	{
		NodePool.Reserve(Policy::NodePoolSize);
	}
};


struct FSVOPathfindingGraph;

struct FSparseVoxelOctreeGraph
{
public:
	FSparseVoxelOctreeGraph(const FSVOData& InSVOData)
		: SVOData(InSVOData.AsShared())
	{
	}
	
	void UpdateNavData(const FSVOData& InSVOData) const
	{
		SVOData = InSVOData.AsShared();
	}
	
	friend FSVOPathfindingGraph;
	typedef FSVOLink FNodeRef;

	bool IsValidRef(const FSVOLink& NodeRef) const { return NodeRef.IsValid(); }
	
	void GetNeighbours(const FSVOLink& NodeRef, TArray<FSVOLink>& Neighbours) const;
	
	FNodeRef GetNodeRefForLocation(const FVector& StartLocation) const
	{
		return SVOData->GetNodeLinkForPosition(StartLocation);
	}

	FVector GetLocationForNodeRef(const FNodeRef& NodeRef) const
	{
		return SVOData->GetPositionForLink(NodeRef);
	}

private:
	void SubdivideNeighbour(const FSVOLink& NeighbourNodeRef, int32 Direction, TArray<FSVOLink>& Neighbours) const;
	
	FSVOLink GetLeafNeighbour(const FSVOLink& NodeRef, const FSVOLeafNode& LeafNode, const int32 Direction) const;
	
	FSVOLink GetSubNodeNeighbour(const FSVOLink& NodeRef, const FSVOLeafNode& LeafNode, const int32 Direction) const;

protected:
	mutable FSVODataConstRef SVOData;
};


struct FSVOPathfindingGraph
	: FPathfindingGraph<FSparseVoxelOctreeGraph, FPathfindingGraphDefaultPolicy, FPathfindingGraphDefaultNode<FSparseVoxelOctreeGraph>>
{
public:
	FORCEINLINE_DEBUGGABLE bool ProcessSingleNodeLazyThetaStar(const FGraphNodeRef& EndNodeRef, const FSparseVoxelOctreeQuery& Query, int32& OutBestNodeIndex, FVector::FReal& OutBestNodeCost);
	
	FORCEINLINE_DEBUGGABLE bool ProcessSingleNodeThetaStar(const FGraphNodeRef& EndNodeRef, const FSparseVoxelOctreeQuery& Query, int32& OutBestNodeIndex, FVector::FReal& OutBestNodeCost);
	
	FORCEINLINE_DEBUGGABLE bool ProcessSingleNodeAStar(const FGraphNodeRef& EndNodeRef, const FSparseVoxelOctreeQuery& Query, int32& OutBestNodeIndex, FVector::FReal& OutBestNodeCost);
	
	ENavigationQueryResult::Type FindPath(const FVector& StartLocation, const FVector& EndLocation, const FSparseVoxelOctreeQuery& Query, FFlightNavigationPath& NavPath);

private:
	EFPathfindingResult FindPathImpl(const FGraphNodeRef& StartNodeRef, const FGraphNodeRef& EndNodeRef, const FSparseVoxelOctreeQuery& Query, TArray<FGraphNodeRef>& OutPath);

	ENavigationQueryResult::Type PostProcessPath(const FSparseVoxelOctreeQuery& Query, TArray<FGraphNodeRef>& NodePath, FFlightNavigationPath& NavPath, const EFPathfindingResult& PathResult) const;
};
