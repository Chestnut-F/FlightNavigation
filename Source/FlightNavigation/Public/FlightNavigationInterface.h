// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SparseVoxelOctreeQuery.h"
#include "UObject/Interface.h"
#include "FlightNavigationInterface.generated.h"

UINTERFACE()
class UFlightNavigationInterface : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class FLIGHTNAVIGATION_API IFlightNavigationInterface
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Pathfinding")
	FSparseVoxelOctreeQuery GetFlightNavigationQuery() const;

	virtual FSparseVoxelOctreeQuery GetFlightNavigationQuery_Implementation() const = 0;
};

/**
 * 
 */
struct FLIGHTNAVIGATION_API FFlightNavigationQueryFilter : public INavigationQueryFilterInterface
{
public:
	FFlightNavigationQueryFilter() {}
	virtual ~FFlightNavigationQueryFilter() override {}

public:
	virtual void Reset() override {}

	virtual void SetAreaCost(uint8 AreaType, float Cost) override {}
	virtual void SetFixedAreaEnteringCost(uint8 AreaType, float Cost) override {}
	virtual void SetExcludedArea(uint8 AreaType) override {}
	virtual void SetAllAreaCosts(const float* CostArray, const int32 Count) override {}
	virtual void GetAllAreaCosts(float* CostArray, float* FixedCostArray, const int32 Count) const override {}
	virtual void SetBacktrackingEnabled(const bool bBacktracking) override {}
	virtual bool IsBacktrackingEnabled() const override { return false; }
	virtual float GetHeuristicScale() const override { return 1.f; }
	virtual bool IsEqual(const INavigationQueryFilterInterface* Other) const override { return true; }
	virtual void SetIncludeFlags(uint16 Flags) override {}
	virtual uint16 GetIncludeFlags() const override { return 1; }
	virtual void SetExcludeFlags(uint16 Flags) override {}
	virtual uint16 GetExcludeFlags() const override { return 1; }
	virtual FVector GetAdjustedEndLocation(const FVector& EndLocation) const { return EndLocation; }

	virtual INavigationQueryFilterInterface* CreateCopy() const { return new FFlightNavigationQueryFilter(); }
};