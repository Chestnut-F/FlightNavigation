// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NavigationPath.h"

/**
 * 
 */
struct FLIGHTNAVIGATION_API FFlightNavigationPath : FNavigationPath
{
	typedef FNavigationPath Super;

	FFlightNavigationPath();
	virtual ~FFlightNavigationPath() override;

	void ApplyFlags(int32 NavDataFlags);

	int32 DrawIndex;
};