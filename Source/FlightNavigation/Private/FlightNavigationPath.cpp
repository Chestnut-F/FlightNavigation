// Fill out your copyright notice in the Description page of Project Settings.


#include "FlightNavigationPath.h"

#include "FlightNavigationData.h"
#include "FlightPathBatchComponent.h"

FFlightNavigationPath::FFlightNavigationPath()
	: DrawIndex(INDEX_NONE)
{
}

FFlightNavigationPath::~FFlightNavigationPath()
{
#if WITH_EDITORONLY_DATA
	AFlightNavigationData* NavData = Cast<AFlightNavigationData>(NavigationDataUsed.Get());

	if (NavData && NavData->PathBatcher)
	{
		// Remove path when deleted
		NavData->PathBatcher->RemovePath(DrawIndex);
	}
#endif // WITH_EDITORONLY_DATA
}

void FFlightNavigationPath::ApplyFlags(int32 NavDataFlags)
{
}
