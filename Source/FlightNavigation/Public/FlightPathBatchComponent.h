// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/LineBatchComponent.h"
#include "FlightPathBatchComponent.generated.h"


struct FFlightNavigationPath;

struct FBatchedPath
{
	explicit FBatchedPath(const FColor InColor) :
		Color(InColor),
		ID(++NextUniqueId)
	{}

	void ApplyWorldOffset(const FVector& InOffset);

	TArray<FVector> PathPoints;
	const FColor Color;
	const int32 ID;

private:
	static int32 NextUniqueId;
};

UCLASS(MinimalAPI)
class UFlightPathBatchComponent : public ULineBatchComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UFlightPathBatchComponent();

	// Adds a path to be drawn, returning the index of that path (so it can be removed later) (thread safe)
	int32 AddPath(const FFlightNavigationPath& NavPath, const FVector& PathOffset, const FColor PathColor, const int32 Index = INDEX_NONE);

	// Remove path by index (thread safe)
	void RemovePath(const int32 Index);

	//~ Begin UActorComponent Interface.
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override {} // Don't remove lines
	virtual void ApplyWorldOffset(const FVector& InOffset, bool bWorldShift) override;
	//~ End UActorComponent Interface.

protected:
	TArray<FBatchedPath> BatchedPaths;
	FThreadSafeBool bQueuedRedraw;
	FCriticalSection BatchedPathsLock;

	// Flush and draw paths, threadsafe
	void Redraw();

	// Draw single path, called from game thread
	void DrawPath(const FBatchedPath& BatchedPath);

	void ConvertNavPathPointsToVector(const TArray<FNavPathPoint>& In, TArray<FVector>& Out);
};
