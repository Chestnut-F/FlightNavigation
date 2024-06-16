// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FlightNavigation/Thirdparty/Morton/morton.h"

DECLARE_LOG_CATEGORY_EXTERN(LogFlightNavigation, Log, All);

#define SVODATA_VER_LATEST				2
#define SVODATA_VER_MIN_COMPATIBLE		2

#define MIN_LAYERS 4
#define MAX_LAYERS 15

#define LEAF_FREE 0
#define LEAF_WITH_COLLISION UINT64_MAX

typedef uint_fast64_t FMorton;
typedef FVector::FReal FCoord;
#define INVALID_MORTON UINT64_MAX

UENUM(BlueprintType)
enum class ESVOPathfindingAlgorithm: uint8
{
	AStar 			UMETA(DisplayName = "A *"),
	LazyThetaStar 	UMETA(DisplayName = "Lazy Theta *"),
	ThetaStar 		UMETA(DisplayName = "Theta *")
};

namespace SVO
{
	FORCEINLINE FVector MortonToCoord(const FMorton Code, const FVector& SystemCentre, const FCoord VoxelSideLength, const FCoord VoxelOffset)
	{
		uint_fast32_t X, Y, Z;
		libmorton::morton3D_64_decode(Code, X, Y, Z);
		return SystemCentre + FVector(X, Y, Z) * VoxelSideLength - VoxelOffset;
	}

	FORCEINLINE FIntVector MortonToCoord(const FMorton Code)
	{
		uint_fast32_t X, Y, Z;
		libmorton::morton3D_64_decode(Code, X, Y, Z);
		return FInt32Vector(X, Y, Z);
	}

	FORCEINLINE FMorton CoordToMorton(const FIntVector3& Coord)
	{
		return libmorton::morton3D_64_encode(static_cast<uint_fast32_t>(Coord.X), static_cast<uint_fast32_t>(Coord.Y), static_cast<uint_fast32_t>(Coord.Z));
	}

	FORCEINLINE FMorton CoordToMorton(const int_fast32_t X, const int_fast32_t Y, const int_fast32_t Z)
	{
		return libmorton::morton3D_64_encode(static_cast<uint_fast32_t>(X), static_cast<uint_fast32_t>(Y), static_cast<uint_fast32_t>(Z));
	}
	
	FORCEINLINE const FVector& CoordToVec(const FCoord* Coords, const int32 Index)
	{
		return *reinterpret_cast<const FVector*>(&Coords[3 * Index]);
	}
	
	FORCEINLINE FMorton ParentFromAnyChild(const FMorton& ChildCode) { /* Remove 3 lsb */ return ChildCode >> 3; }
	FORCEINLINE FMorton FirstChildFromParent(const FMorton& ParentCode) { /* Create blank 3 lsb */ return ParentCode << 3; }
	FORCEINLINE FMorton FirstChildFromAnyChild(const FMorton& ChildCode) { /* Clear 3 lsb */ return ChildCode & ~static_cast<FMorton>(0x7); }
	FORCEINLINE FMorton LastChildFromAnyChild(const FMorton& ChildCode) { /* Set 3 lsb */ return ChildCode | static_cast<FMorton>(0x7); }

	FORCEINLINE int32 GetNumLayers(const FCoord SideLength, const FCoord MaxVoxelSize)
	{
		return FMath::Clamp(FMath::CeilToInt(FMath::Log2(SideLength / MaxVoxelSize)), MIN_LAYERS, MAX_LAYERS);
	}
	
	static constexpr int32 DeltaX[6] = {1,-1, 0, 0, 0, 0};
	static constexpr int32 DeltaY[6] = {0, 0, 1,-1, 0, 0};
	static constexpr int32 DeltaZ[6] = {0, 0, 0, 0, 1,-1};
	
	FORCEINLINE FIntVector GetDelta(const int32 Direction)
	{
		return FIntVector(DeltaX[Direction], DeltaY[Direction], DeltaZ[Direction]);
	}

	static constexpr FMorton SubNodeMortonForDirection[6][16] =
{
		{ 0x00, 0x04, 0x20, 0x24, 0x02, 0x06, 0x22, 0x26, 0x10, 0x14, 0x30, 0x34, 0x12, 0x16, 0x32, 0x36 },
		{ 0x09, 0x0d, 0x29, 0x2d, 0x0b, 0x0f, 0x2b, 0x2f, 0x19, 0x1d, 0x39, 0x3d, 0x1b, 0x1f, 0x3b, 0x3f },
		{ 0x00, 0x04, 0x20, 0x24, 0x01, 0x05, 0x21, 0x25, 0x08, 0x0c, 0x28, 0x2c, 0x09, 0x0d, 0x29, 0x2d },
		{ 0x12, 0x16, 0x32, 0x36, 0x13, 0x17, 0x33, 0x37, 0x1a, 0x1e, 0x3a, 0x3e, 0x1b, 0x1f, 0x3b, 0x3f },
		{ 0x00, 0x02, 0x10, 0x12, 0x01, 0x03, 0x11, 0x13, 0x08, 0x0a, 0x18, 0x1a, 0x09, 0x0b, 0x19, 0x1b },
		{ 0x24, 0x26, 0x34, 0x36, 0x25, 0x27, 0x35, 0x37, 0x2c, 0x2e, 0x3c, 0x3e, 0x2d, 0x2f, 0x3d, 0x3f }
};

	static constexpr int32 NeighbourMortonOffset[6][4] = {
		{ 0, 2, 4, 6 },
		{ 1, 3, 5, 7 },
		{ 0, 1, 4, 5 },
		{ 2, 3, 6, 7 },
		{ 0, 1, 2, 3 },
		{ 4, 5, 6, 7 }
	};
}

/**
 * NodeLink:
 * 4 bits  — Layer index 0 to 15. 0 = Leaf Node Layer, 1-15 Normal Node Layers
 * 22 bits — Node index 0 to 2^22
 * 6 bits — SubNode index 0 to 63
 */
struct FSVOLink
{
	uint32 Link;

	FSVOLink(const uint32 InLink = MAX_uint32) : Link(InLink) {}

	FSVOLink(const uint32 InLayerIndex, const uint32 InNodeIndex, const uint32 InSubNodeIndex = 0) : Link()
	{
		checkf(InLayerIndex < (1 << 4), TEXT("Large Layer Index: %d"), InLayerIndex)
		checkf(InNodeIndex < (1 << 22), TEXT("Large Node Index: %d"), InNodeIndex)
		checkf(InSubNodeIndex < (1 << 6), TEXT("Large SubNode Index: %d"), InSubNodeIndex)
		Link = InLayerIndex << 28;
		Link |= InNodeIndex << 6;
		Link |= InSubNodeIndex;
	}

	FSVOLink(const FSVOLink& InLink) : Link(InLink.Link) {}
	FSVOLink& operator=(const FSVOLink& InLink) { Link = InLink.Link; return *this; }

	bool IsValid() const
	{
		return Link != INVALID;
	}
	
	uint32 GetLayerIndex() const
	{
		return Link >> 28;
	}
	
	uint32 GetNodeIndex() const
	{
		return (Link & 0x0fffffc0) >> 6;
	}
	
	uint32 GetSubNodeIndex() const
	{
		return (Link & 0x0000003f);
	}

	void SetNodeIndex(const uint64 InNodeIndex)
	{
		checkf(InNodeIndex < (1 << 22), TEXT("Large Node Index: %d"), InNodeIndex)
		ClearNodeIndex();
		Link |= InNodeIndex << 6;
	}

	void ClearNodeIndex()
	{
		Link = (Link & ~0x0fffffc0);
	}

	FString ToString() const
	{
		return FString::Printf(TEXT("Layer=%d Node=%d SubNode=%d"), GetLayerIndex(), GetNodeIndex(), GetSubNodeIndex());
	}
	
	NavNodeRef AsNavNodeRef() const
	{
		return static_cast<NavNodeRef>(Link);
	}

	friend bool operator==(const FSVOLink& A, const FSVOLink& B) { return A.Link == B.Link; }
	friend bool operator!=(const FSVOLink& A, const FSVOLink& B) { return A.Link != B.Link; }
	friend bool operator<(const FSVOLink& A, const FSVOLink& B)  { return A.Link < B.Link; } // Useful for sorting by layer

	friend FSVOLink operator+(const FSVOLink& Link, const int32 NodeOffset)
	{
		FSVOLink NewLink(Link);
		NewLink.SetNodeIndex(Link.GetNodeIndex() + NodeOffset);
		return NewLink;
	}
	
	friend FArchive& operator<<(FArchive& Ar, FSVOLink& SVOLink)
	{
		Ar << SVOLink.Link;
		return Ar;
	}

	friend uint32 GetTypeHash(const FSVOLink& SVOLink)
	{
		return GetTypeHash(SVOLink.Link);
	}

	static const FSVOLink INVALID;
};

struct FSVOLeafNode
{
	uint64 VoxelGrid;
	FSVOLink ParentLink;

	FSVOLeafNode() : VoxelGrid(0) {}

	FSVOLeafNode(const FSVOLeafNode& InLeafNode) : VoxelGrid(InLeafNode.VoxelGrid), ParentLink(InLeafNode.ParentLink) {}
	FSVOLeafNode& operator=(const FSVOLeafNode& InLeafNode)
	{
		VoxelGrid = InLeafNode.VoxelGrid;
		ParentLink = InLeafNode.ParentLink;
		return *this;
	}

	FORCEINLINE void SetVoxelWithCollision(const FMorton Index)
	{
		VoxelGrid |= (1ULL << Index);
	}

	FORCEINLINE void SetVoxelFree(const FMorton Index)
	{
		VoxelGrid &= ~(1ULL << Index);
	}

	FORCEINLINE bool IsVoxelWithCollision(const FMorton Index) const
	{
		return (1ULL << Index) & VoxelGrid;
	}

	FORCEINLINE bool IsVoxelFree(const FMorton Index) const
	{
		return !((1ULL << Index) & VoxelGrid);
	}
	
	FORCEINLINE bool IsLeafWithCollision() const { return VoxelGrid == LEAF_WITH_COLLISION; }
	
	FORCEINLINE bool IsLeafFree() const { return VoxelGrid == LEAF_FREE; }

	friend uint32 GetTypeHash(const FSVOLeafNode& SVOLeafNode)
	{
		return GetTypeHash(SVOLeafNode.VoxelGrid);
	}

	FORCEINLINE friend FArchive& operator<<(FArchive& Ar, FSVOLeafNode& LeafNode)
	{
		Ar << LeafNode.VoxelGrid;
		Ar << LeafNode.ParentLink;
		return Ar;
	}
};

struct FSVONode
{
	FMorton MortonCode;
	FSVOLink FirstChild;
	FSVOLink Parent;
	FSVOLink Neighbours[6];

	bool bHasChildren;

	FSVONode()
		: MortonCode(INVALID_MORTON)
		, FirstChild(FSVOLink::INVALID)
		, Parent(FSVOLink::INVALID)
		, bHasChildren (false) {}

	FSVONode(const FSVONode& InNode)
		: MortonCode(InNode.MortonCode)
		, FirstChild(InNode.FirstChild)
		, Parent(InNode.Parent)
		, bHasChildren(InNode.bHasChildren)
	{
		FMemory::Memcpy(Neighbours, InNode.Neighbours, 6);
	}

	FSVONode& operator=(const FSVONode& InNode)
	{
		MortonCode = InNode.MortonCode;
		FirstChild = InNode.FirstChild;
		Parent = InNode.Parent;
		FMemory::Memcpy(Neighbours, InNode.Neighbours, 6);
		bHasChildren = InNode.bHasChildren;
		return *this;
	}

	explicit FSVONode(const FMorton InMortonCode): MortonCode(InMortonCode) {}
	
	FORCEINLINE bool HasChildren() const { return bHasChildren; }
	
	FORCEINLINE bool IsWithCollision() const { return bHasChildren; }
	FORCEINLINE bool IsFree() const { return !bHasChildren; }

	// Sort by morton code
	FORCEINLINE friend bool operator<(const FSVONode& A, const FSVONode& B) { return A.MortonCode < B.MortonCode; }

	FORCEINLINE friend FArchive& operator<<(FArchive& Ar, FSVONode& Node)
	{
		Ar << Node.bHasChildren;
		if (Node.bHasChildren)
		{
			Ar << Node.FirstChild;
		}
		Ar << Node.Parent;
		for (int i = 0; i < 6; i++)
		{
			Ar << Node.Neighbours[i];
		}
		Ar << Node.MortonCode;
	
		return Ar;
	}
};

struct FSVOLeafLayer
{
	TArray<FSVOLeafNode> LeafNodes;

	FSVOLeafNode& AddLeafNode_GetRef()
	{
		return LeafNodes.AddDefaulted_GetRef();
	}
	
	int32 AddDefaulted(int32 Count)
	{
		return LeafNodes.AddDefaulted(Count);
	}

	void Clear()
	{
		LeafNodes.Reset();
	}
	
	FSVOLeafNode& operator[](const int32 Index) { return LeafNodes[Index]; }
	const FSVOLeafNode& operator[](const int32 Index) const { return LeafNodes[Index]; }

	void Reserve(const int32 Number) { LeafNodes.Reserve(Number); }
	void Empty(const int32 Number) { LeafNodes.Empty(Number); }
	int32 Num() const { return LeafNodes.Num(); }
	
	FORCEINLINE friend FArchive& operator<<(FArchive& Ar, FSVOLeafLayer& LeafLayerData)
	{
		Ar << LeafLayerData.LeafNodes;
		return Ar;
	}
};

struct FSVOLayer
{
	TArray<FSVONode> Nodes;

	int32 AddDefaulted() { return Nodes.AddDefaulted(); }
	FSVONode& AddDefaulted_GetRef() { return Nodes.AddDefaulted_GetRef(); }
	int32 Num() const { return Nodes.Num(); }

	FSVONode& GetNode(const int32 Index) { return Nodes[Index]; }
	const FSVONode& GetNode(const int32 Index) const { return Nodes[Index]; }
	FSVONode& operator[](const int32 Index) { return Nodes[Index]; }
	const FSVONode& operator[](const int32 Index) const { return Nodes[Index]; }
	FSVONode& GetLastNode(const int32 IndexFromTheEnd = 0) { return Nodes.Last(IndexFromTheEnd); }
	const FSVONode& GetLastNode(const int32 IndexFromTheEnd = 0) const { return Nodes.Last(IndexFromTheEnd); }

	void Reserve(const int32 Number) { Nodes.Reserve(Number); }
	void Empty(const int32 Number) { Nodes.Empty(Number); }

	void AddChildlessNodes(const int32 Num, const FMorton StartCode)
	{
		for (int32 i = 0; i < Num; ++i)
		{
			FSVONode& ChildlessNode = AddDefaulted_GetRef();
			ChildlessNode.MortonCode = StartCode + i;
			ChildlessNode.bHasChildren = false;
		}
	}

	void FillWithChildlessNodes(const FMorton FirstMorton, const FMorton LastMorton)
	{
		check(SVO::ParentFromAnyChild(FirstMorton) == SVO::ParentFromAnyChild(LastMorton))
		AddChildlessNodes(LastMorton - FirstMorton + 1, FirstMorton);
	}
	
	int32 FindNode(const FMorton& NodeMorton) const
	{
		return Algo::BinarySearch<const TArray<FSVONode>, FSVONode>(Nodes, FSVONode(NodeMorton));
	}

	FORCEINLINE friend FArchive& operator<<(FArchive& Ar, FSVOLayer& LayerData)
	{
		Ar << LayerData.Nodes;
		return Ar;
	}
};

struct FSVOData : TSharedFromThis<FSVOData, ESPMode::ThreadSafe>
{
	FSVOLeafLayer LeafLayer;
	TArray<FSVOLayer> Layers;
	FBox Bounds;
	FVector Centre;
	FCoord SideSize;
	FCoord SubNodeSize;
	int32 NumLayers;
	int32 NumNodeLayers;
	bool bValid;

	bool IsFree() const { return bValid && Layers.Num() == 1; }

	FSVOLeafNode& AddLeafNode_GetRef()
	{
		return LeafLayer.AddLeafNode_GetRef();
	}
	
	FSVOLayer& AddLayer_GetRef()
	{
		return Layers.AddDefaulted_GetRef();
	}
	
	FSVOLayer& GetLayer(const int32 LayerNum)
	{
		check(0 < LayerNum && LayerNum <= Layers.Num())
		return Layers[LayerNum-1];
	}
	const FSVOLayer& GetLayer(const int32 LayerNum) const
	{
		check(0 < LayerNum && LayerNum <= Layers.Num())
		return Layers[LayerNum-1];
	}

	void Clear()
	{
		LeafLayer.Clear();
		Layers.Reset();
		bValid = false;
	}

	void SetBounds(const FBox& InBounds)
	{
		Centre = InBounds.GetCenter();
		SideSize = InBounds.GetSize().GetMax();
		Bounds = FBox::BuildAABB(Centre, FVector(SideSize * 0.5));
		bValid = InBounds.IsValid != 0;
	}

	const FSVONode& GetRoot() const
	{
		check(bValid && Layers.Num() > 0)
		return GetLayer(Layers.Num())[0];
	}
	FSVOLink GetRootLink() const
	{
		check(Layers.Num() > 0)
		return FSVOLink(Layers.Num(), 0);
	}
	
	FVector GetCentre() const { return Centre; }

	FCoord GetSubNodeOffset() const { return SubNodeSize * 1.5; }
	FCoord GetNodeOffsetForLayer(const int32 Layer) const { return (SideSize - GetNodeSizeForLayer(Layer)) * 0.5; }

	FCoord GetSubNodeSize() const { return SubNodeSize; }
	FCoord GetNodeSizeForLayer(const int32 Layer) const { return SubNodeSize * (4 << Layer); }

	FVector GetSubNodeExtent() const{ return FVector(SubNodeSize * 0.5f); }
	FVector GetNodeExtentForLayer(const int32 Layer) const { return FVector(SubNodeSize * (2 << Layer)); }
	FVector GetOctreeExtent() const { return FVector(SideSize * 0.5f); }

	FVector GetPositionForLink(const FSVOLink& NodeRef, bool bSearchSubNodes = true) const
	{
		const int32 LayerIdx = NodeRef.GetLayerIndex();
		const int32 NodeIdx = NodeRef.GetNodeIndex();

		if (LayerIdx == 0)
		{
			const FSVOLeafNode& Leaf = LeafLayer[NodeIdx];
			const FSVONode& LeafParent = GetLayer(1).GetNode(Leaf.ParentLink.GetNodeIndex());
			const int32 ChildIdx = NodeIdx - LeafParent.FirstChild.GetNodeIndex();
			const FMorton LeafMorton = SVO::FirstChildFromParent(LeafParent.MortonCode) + ChildIdx;
			
			const FVector LeafPosition = SVO::MortonToCoord(LeafMorton, Centre, GetNodeSizeForLayer(0), GetNodeOffsetForLayer(0));

			if (Leaf.IsLeafFree() || !bSearchSubNodes)
			{
				return LeafPosition;
			}
			
			// Find position of SubNode
			return SVO::MortonToCoord(NodeRef.GetSubNodeIndex(), LeafPosition, SubNodeSize, GetSubNodeOffset());
		}
		else
		{
			const FSVONode& Node = GetLayer(LayerIdx).GetNode(NodeIdx);
			return SVO::MortonToCoord(Node.MortonCode, Centre, GetNodeSizeForLayer(LayerIdx), GetNodeOffsetForLayer(LayerIdx));
		}
	}
	
	FVector GetExtentForLink(const FSVOLink& NodeRef, bool bSearchSubNodes = true) const
	{
		const int32 LayerIdx = NodeRef.GetLayerIndex();
		if (LayerIdx == 0)
		{
			const int32 NodeIdx = NodeRef.GetNodeIndex();
			if (LeafLayer[NodeIdx].IsLeafFree() || !bSearchSubNodes)
			{
				return GetNodeExtentForLayer(0);
			}
			else
			{
				return GetSubNodeExtent();
			}
		}
		else
		{
			return GetNodeExtentForLayer(LayerIdx);
		}
	}
	
	FBox GetNodeBoxForLink(const FSVOLink& NodeRef, bool bSearchSubNodes = true) const
	{
		return FBox::BuildAABB(GetPositionForLink(NodeRef, bSearchSubNodes), GetExtentForLink(NodeRef, bSearchSubNodes));
	}
	
	int32 FindNodeInLayer(const int32 LayerNum, const FMorton& NodeMorton) const
	{
		return GetLayer(LayerNum).FindNode(NodeMorton);
	}

	FVector VoxelSnap(const FVector& Position) const
	{
		return Centre + (Position - Centre + GetSubNodeExtent()).GridSnap(SubNodeSize) - GetSubNodeExtent();
	}

	FSVOLink GetNodeLinkForPosition(const FVector& Position) const
	{
		if (!Bounds.IsInside(Position) || !bValid)
		{
			return FSVOLink::INVALID;
		}

		FVector SnapPosition = VoxelSnap(Position);
		
		FSVOLink ParentLink = GetRootLink();
		FSVOLink Result = FSVOLink::INVALID;
		for (int32 LayerIndex = ParentLink.GetLayerIndex() - 1; LayerIndex >= 0; --LayerIndex)
		{
			const FSVONode& ParentNode = GetLayer(ParentLink.GetLayerIndex()).GetNode(ParentLink.GetNodeIndex());
			
			if (ParentNode.HasChildren())
			{
				const int32 FirstChildIdx = ParentNode.FirstChild.GetNodeIndex();
                const int32 ChildLayerIdx = ParentNode.FirstChild.GetLayerIndex();

				check(ChildLayerIdx == LayerIndex);
				
				bool bChildFound = false;
	
				for (int32 Child = 0; Child < 8; ++Child)
				{
					const FSVOLink ChildLink(ChildLayerIdx, FirstChildIdx + Child);

					const FBox NodeBounds = GetNodeBoxForLink(ChildLink, false);
					if (NodeBounds.IsInsideOrOn(SnapPosition))
					{
						ParentLink = ChildLink;
						bChildFound = true;
						break;
					}
				}

				check(bChildFound);
			}
			else
			{
				Result = ParentLink;
				break;
			}
		}

		if (Result.IsValid())
		{
			return Result;
		}

		check(ParentLink.GetLayerIndex() == 0)

		const int32 LeafNodeIndex = ParentLink.GetNodeIndex();
		const FSVOLeafNode& LeafNode = LeafLayer[LeafNodeIndex];
		if (LeafNode.IsLeafWithCollision())
		{
			return FSVOLink::INVALID;
		}

		if (LeafNode.IsLeafFree())
		{
			return ParentLink;
		}
		
		for (int32 Child = 0; Child < 64; ++Child)
		{
			if (LeafNode.IsVoxelWithCollision(Child))
			{
				continue;
			}
			
			const FSVOLink SubNodeLink(0, LeafNodeIndex, Child);
			FBox SubNodeBounds = GetNodeBoxForLink(SubNodeLink);
			if (SubNodeBounds.IsInsideOrOn(SnapPosition))
			{
				return SubNodeLink;
			}
		}

		return FSVOLink::INVALID;
	}

	FORCEINLINE friend FArchive& operator<<(FArchive& Ar, FSVOData& Data)
	{
		Ar << Data.LeafLayer;
		Ar << Data.Layers;
		Ar << Data.Bounds;
		Ar << Data.Centre;
		Ar << Data.SideSize;
		Ar << Data.SubNodeSize;
		Ar << Data.NumLayers;
		Ar << Data.NumNodeLayers;
		Ar << Data.bValid;

		if (Ar.IsLoading())
		{
			if (Data.Layers.Num() > 0 && Data.Bounds.IsValid)
			{
				Data.bValid = true;
			}
			else
			{
				Data.Clear();
			}
		}

		return Ar;
	}
};


typedef TSharedRef<		 FSVOData, ESPMode::ThreadSafe>	FSVODataRef;
typedef TSharedRef<const FSVOData, ESPMode::ThreadSafe> FSVODataConstRef;
typedef TSharedPtr<		 FSVOData, ESPMode::ThreadSafe>	FSVODataPtr;
typedef TSharedPtr<const FSVOData, ESPMode::ThreadSafe> FSVODataConstPtr;
