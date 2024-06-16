// Fill out your copyright notice in the Description page of Project Settings.


#include "FlightNavigationData.h"

#include "FlightNavigationInterface.h"
#include "FlightNavigationPath.h"
#include "FlightPathBatchComponent.h"
#include "NavigationSystem.h"
#include "SparseVoxelOctreeGenerator.h"
#include "SparseVoxelOctreeGraph.h"
#include "SparseVoxelOctreeQuery.h"
#include "SVORenderingComponent.h"
#include "AI/NavDataGenerator.h"
#include "AI/NavigationSystemBase.h"


AFlightNavigationData::AFlightNavigationData()
	: MaxVoxelSize(512.f)
#if WITH_EDITORONLY_DATA
	, ActualVoxelSize(512.f)
	, bAllowDrawingInGameWorld(false)
	, bDrawDebugPaths(false)
#endif // WITH_EDITORONLY_DATA
	, bDrawOctreeNodes(false)
	, bDrawOnlySpecifiedLayer(false)
	, bDrawOnlyOnlyOverlappedNodes(true)
	, bDrawOctreeSubNodes(false)
	, bDrawOnlyOverlappedSubNodes(true)
	, SVOData(new FSVOData)
	, BuildingSVOData(new FSVOData)
	, SVODataVersion(SVODATA_VER_LATEST)
{
	if (HasAnyFlags(RF_ClassDefaultObject) == false)
	{
		FindPathImplementation = FindPath;
		FindHierarchicalPathImplementation = FindPath;

		TestPathImplementation = TestPath;
		TestHierarchicalPathImplementation = TestPath;

		RaycastImplementation = SparseVoxelOctreeRaycast;
		
		SVOGraph = MakeUnique<const FSparseVoxelOctreeGraph>(SVOData.Get());
		PathfindingGraph = MakeUnique<FSVOPathfindingGraph>(*SVOGraph);

		DefaultQueryFilter->SetFilterType<FFlightNavigationQueryFilter>();

#if WITH_EDITORONLY_DATA
		// Create line batcher component
		PathBatcher = CreateEditorOnlyDefaultSubobject<UFlightPathBatchComponent>(TEXT("FlyingNavPathBatcher"), true);

		if (PathBatcher)
		{
			PathBatcher->bCalculateAccurateBounds = false;
		}
#endif
	}
}

void AFlightNavigationData::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);

	Ar << SVODataVersion;
	
	uint32 SVODataSizeBytes = 0;
	const int64 SVODataSizePos = Ar.Tell();
	Ar << SVODataSizeBytes;

	if (Ar.IsLoading())
	{
		if (SVODataVersion < SVODATA_VER_MIN_COMPATIBLE )
		{
			SVODataVersion = SVODATA_VER_LATEST;
			
			Ar.Seek(SVODataSizePos + SVODataSizeBytes);
	
			return;
		}
	}

	if (!Ar.IsTransacting())
	{
		Ar << SVOData.Get();
	}

	if (Ar.IsSaving())
	{
		const int64 CurPos = Ar.Tell();
		SVODataSizeBytes = CurPos - SVODataSizePos;
		Ar.Seek(SVODataSizePos);
		Ar << SVODataSizeBytes;
		Ar.Seek(CurPos);
	}
}

#if WITH_EDITORONLY_DATA
void AFlightNavigationData::UpdateEditorData()
{
}
#endif

void AFlightNavigationData::UpdateDrawing()
{
#if !UE_BUILD_SHIPPING
	USVORenderingComponent* OctreeRenderComp = Cast<USVORenderingComponent>(RenderingComp);
	if (OctreeRenderComp != nullptr && OctreeRenderComp->GetVisibleFlag() && (OctreeRenderComp->IsForcingUpdate() || USVORenderingComponent::IsNavigationShowFlagSet(GetWorld())))
	{
		RenderingComp->MarkRenderStateDirty();
	}
#endif // !UE_BUILD_SHIPPING
}

void AFlightNavigationData::RequestDrawingUpdate(bool bForce)
{
#if !UE_BUILD_SHIPPING
	if (bForce || USVORenderingComponent::IsNavigationShowFlagSet(GetWorld()))
	{
		if (bForce)
		{
			USVORenderingComponent* OctreeRenderComp = Cast<USVORenderingComponent>(RenderingComp);
			if (OctreeRenderComp)
			{
				OctreeRenderComp->ForceUpdate();
			}
		}

		DECLARE_CYCLE_STAT(TEXT("FSimpleDelegateGraphTask.Requesting Octree Redraw"),
			STAT_FSimpleDelegateGraphTask_RequestingOctreeRedraw,
			STATGROUP_TaskGraphTasks);

		FSimpleDelegateGraphTask::CreateAndDispatchWhenReady(
FSimpleDelegateGraphTask::FDelegate::CreateUObject(this, &AFlightNavigationData::UpdateDrawing),
			 GET_STATID(STAT_FSimpleDelegateGraphTask_RequestingOctreeRedraw), nullptr, ENamedThreads::GameThread);
	}
#endif // !UE_BUILD_SHIPPING
}

void AFlightNavigationData::UpdateSVOData()
{
	// Swap references, clear but don't release resources
	{
		FRWScopeLock Lock(SVODataLock, SLT_Write);

		const FSVODataRef Temp = SVOData;
		SVOData = BuildingSVOData;
		BuildingSVOData = Temp;
		BuildingSVOData->Clear();

		SVOGraph->UpdateNavData(SVOData.Get());
	}

#if WITH_EDITORONLY_DATA
	UpdateEditorData();
#endif
}

UPrimitiveComponent* AFlightNavigationData::ConstructRenderingComponent()
{
	return NewObject<USVORenderingComponent>(this, TEXT("SVORenderingComponent"), RF_Transient);
}

void AFlightNavigationData::ConditionalConstructGenerator()
{
	if (NavDataGenerator.IsValid())
	{
		NavDataGenerator->CancelBuild();
		NavDataGenerator.Reset();
	}

	UWorld* World = GetWorld();
	check(World);
	const bool bRequiresGenerator = SupportsRuntimeGeneration() || !World->IsGameWorld();
	if (bRequiresGenerator)
	{
		FSparseVoxelOctreeGenerator* Generator = CreateGeneratorInstance();
		if (Generator)
		{
			NavDataGenerator = MakeShareable((FNavDataGenerator*)Generator);
			Generator->Init();
		}
	}
}

void AFlightNavigationData::OnOctreeGenerationFinished()
{
	UWorld* World = GetWorld();

	if (World != nullptr && IsValid(World))
	{
		// Swaps building navdata into current navdata. I'm hoping 
		UpdateSVOData();
		
#if WITH_EDITOR
		// Force navmesh drawing update
		RequestDrawingUpdate(true);		
#endif// WITH_EDITOR

		UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
		if (NavSys)
		{
			NavSys->OnNavigationGenerationFinished(*this);
		}
	}
}

FVector AFlightNavigationData::GetAdjustedPathPoint(const FVector& PathPoint, const float AgentHalfHeight) const
{
	const FVector HalfHeightPoint = PathPoint + FVector(0.f, 0.f, AgentHalfHeight);
	return HalfHeightPoint;
}

FPathFindingResult AFlightNavigationData::FindPath(const FNavAgentProperties& AgentProperties,
                                                   const FPathFindingQuery& Query)
{
	const ANavigationData* Self = Query.NavData.Get();
	check(Cast<const AFlightNavigationData>(Self));

	const AFlightNavigationData* FlightNavData = (const AFlightNavigationData*)Self;
	if (Self == nullptr || !FlightNavData->PathfindingGraph.IsValid())
	{
		return ENavigationQueryResult::Error;
	}
		
	FPathFindingResult Result(ENavigationQueryResult::Error);

	FNavigationPath* NavPath = Query.PathInstanceToFill.Get();
	FFlightNavigationPath* FlightNavPath = NavPath ? NavPath->CastPath<FFlightNavigationPath>() : nullptr;

	if (FlightNavPath)
	{
		Result.Path = Query.PathInstanceToFill;
		FlightNavPath->ResetForRepath();
	}
	else
	{
		Result.Path = Self->CreatePathInstance<FFlightNavigationPath>(Query);
		NavPath = Result.Path.Get();
		FlightNavPath = NavPath ? NavPath->CastPath<FFlightNavigationPath>() : nullptr;
	}
	
	if (FlightNavPath)
	{
		FlightNavPath->ApplyFlags(Query.NavDataFlags);

		FSparseVoxelOctreeQuery SVOQuery;
		const UObject* Owner = Query.Owner.Get();
		if (Owner && Owner->Implements<UFlightNavigationInterface>())
		{
			SVOQuery = IFlightNavigationInterface::Execute_GetFlightNavigationQuery(Owner);
		} else
		{
			SVOQuery = FlightNavData->DefaultQuerySettings;
		}
		SVOQuery.SetNavData(FlightNavData->SVOData);
		
		float AgentHalfHeight = AgentProperties.AgentHeight * 0.5f;
		// const FVector AdjustedStartLocation = FlightNavData->GetAdjustedPathPoint(Query.StartLocation, AgentHalfHeight); 
		const FVector AdjustedEndLocation   = FlightNavData->GetAdjustedPathPoint(Query.EndLocation, AgentHalfHeight);
		if ((Query.StartLocation - AdjustedEndLocation).IsNearlyZero() == true)
		{
			Result.Path->GetPathPoints().Reset();
			Result.Path->GetPathPoints().Add(FNavPathPoint(AdjustedEndLocation));
			Result.Result = ENavigationQueryResult::Success;
		}
		else
		{
			Result.Result = FlightNavData->PathfindingGraph->FindPath(Query.StartLocation, AdjustedEndLocation, SVOQuery, *FlightNavPath);

			const bool bPartialPath = Result.IsPartial();
			if (bPartialPath)
			{
				Result.Result = Query.bAllowPartialPaths ? ENavigationQueryResult::Success : ENavigationQueryResult::Fail;
			}
		}

#if WITH_EDITORONLY_DATA
		if (FlightNavData->bDrawDebugPaths && FlightNavData->PathBatcher)
		{
			FlightNavPath->DrawIndex = FlightNavData->PathBatcher->AddPath(
				*FlightNavPath,
				FVector(0.f, 0.f, AgentHalfHeight),
				SVOQuery.DebugPathColor.ToFColor(true),
				FlightNavPath->DrawIndex);
		}
#endif // WITH_EDITORONLY_DATA
	}
	
	return Result;
}

bool AFlightNavigationData::TestPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query,
	int32* NumVisitedNodes)
{
	return false;
}

bool AFlightNavigationData::SparseVoxelOctreeRaycast(const ANavigationData* Self, const FVector& RayStart,
	const FVector& RayEnd, FVector& HitLocation, FSharedConstNavQueryFilter QueryFilter, const UObject* Querier)
{
	return false;
}

void AFlightNavigationData::SetConfig(const FNavDataConfig& Src)
{
	NavDataConfig = Src;
	AgentRadius = NavDataConfig.AgentRadius;
}

FBox AFlightNavigationData::GetFlightBounds() const
{
	FBox Bbox(ForceInit);

	if (SVOData->bValid)
	{
		Bbox = SVOData->Bounds;
	}
	
	return Bbox;
}

FSparseVoxelOctreeGenerator* AFlightNavigationData::CreateGeneratorInstance()
{
	return new FSparseVoxelOctreeGenerator(*this);
}


