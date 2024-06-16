// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Debug/DebugDrawComponent.h"
#include "FlightNavigationData.h"
#include "SVORenderingComponent.generated.h"

class USVORenderingComponent;

enum class ESVODetailFlags : uint8
{
	NodeBoxes,
	SubNodeBoxes,
	OnlyOverlappedSubNodes
};

// exported to API for GameplayDebugger module
struct FSVOSceneProxyData : public TSharedFromThis<FSVOSceneProxyData, ESPMode::ThreadSafe>
{
	// Processed rendering data
	TArray<FDynamicMeshVertex> Vertices;
	TArray<uint32> Indices;

	FLIGHTNAVIGATION_API void Reset();
	FLIGHTNAVIGATION_API uint32 GetAllocatedSize() const;
	
	FLIGHTNAVIGATION_API void GatherData(const AFlightNavigationData* NavData, int32 InNavDetailFlags);
};

// exported to API for GameplayDebugger module
class FSVOSceneProxy final : public FDebugRenderSceneProxy, public FNoncopyable
{
public:
    FLIGHTNAVIGATION_API virtual SIZE_T GetTypeHash() const override;

    FLIGHTNAVIGATION_API FSVOSceneProxy(const USVORenderingComponent* InComponent, FSVOSceneProxyData* InProxyData, bool ForceToRender = false);
    FLIGHTNAVIGATION_API virtual ~FSVOSceneProxy() override;

	FLIGHTNAVIGATION_API virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override;

protected:
	FLIGHTNAVIGATION_API virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override;
	FLIGHTNAVIGATION_API virtual bool CanBeOccluded() const override;

	virtual uint32 GetMemoryFootprint(void) const override { return sizeof(*this) + GetAllocatedSizeInternal(); }
	FLIGHTNAVIGATION_API uint32 GetAllocatedSizeInternal(void) const;

	uint32 bForceRendering : 1;

private:
	uint32 bHasMeshData: 1;
	
	UMaterialInterface* Material;
	FStaticMeshVertexBuffers VertexBuffers;
	FDynamicMeshIndexBuffer32 IndexBuffer;
	FLocalVertexFactory VertexFactory;
};

UCLASS(editinlinenew, ClassGroup = Debug, MinimalAPI)
class USVORenderingComponent : public UMeshComponent
{
	GENERATED_BODY()

public:
	USVORenderingComponent(const FObjectInitializer& ObjectInitializer);
	
	void ForceUpdate() { bForceUpdate = true; }
	bool IsForcingUpdate() const { return bForceUpdate; }
	
	static FLIGHTNAVIGATION_API bool IsNavigationShowFlagSet(const UWorld* World);

protected:
	FLIGHTNAVIGATION_API virtual void OnRegister()  override;
	FLIGHTNAVIGATION_API virtual void OnUnregister()  override;
	
#if UE_ENABLE_DEBUG_DRAWING
	FLIGHTNAVIGATION_API virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
#endif // UE_ENABLE_DEBUG_DRAWING
	
	FLIGHTNAVIGATION_API virtual FBoxSphereBounds CalcBounds(const FTransform &LocalToWorld) const override;
	
	FLIGHTNAVIGATION_API virtual int32 GetNumMaterials() const override;
	
	/** Gathers drawable information from NavMesh and puts it in OutProxyData. 
	*	Override to add additional information to OutProxyData.*/
	FLIGHTNAVIGATION_API virtual void GatherData(const AFlightNavigationData& NavData, FSVOSceneProxyData& OutProxyData) const;

	FLIGHTNAVIGATION_API void TimerFunction();

protected:
	UPROPERTY()
	UMaterialInstanceDynamic* WireMaterial;
	
	uint32 bCollectNavigationData : 1;
	uint32 bForceUpdate : 1;
	FTimerHandle TimerHandle;
};
