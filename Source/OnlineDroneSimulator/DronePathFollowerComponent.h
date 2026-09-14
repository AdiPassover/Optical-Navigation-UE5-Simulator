#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "DronePathFollowerComponent.generated.h"

class AActor;
class UPrimitiveComponent;
class USplineComponent;


UENUM(BlueprintType)
enum class EDronePathYawMode : uint8
{
    KeepInitial UMETA(DisplayName = "Keep Initial Yaw"),
    FollowMovement UMETA(DisplayName = "Follow Movement")
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(
    FOnDronePathFinished,
    AActor*,
    FinishedPath
);


UCLASS(
    ClassGroup = (Custom),
    meta = (BlueprintSpawnableComponent)
)
class ONLINEDRONESIMULATOR_API UDronePathFollowerComponent
    : public UActorComponent
{
    GENERATED_BODY()

public:
    UDronePathFollowerComponent();

protected:
    virtual void BeginPlay() override;

public:

    virtual void TickComponent(
        float DeltaTime,
        ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction
    ) override;


    UFUNCTION(BlueprintCallable, Category = "Drone Path Following")
    void Initialize(UPrimitiveComponent* InPhysicsBody);


    UFUNCTION(BlueprintCallable, Category = "Drone Path Following")
    void StartFollowingPath(
        AActor* PathActor,
        float CruiseSpeedMps
    );


    UFUNCTION(BlueprintCallable, Category = "Drone Path Following")
    void StopFollowingPath();


    UFUNCTION(BlueprintPure, Category = "Drone Path Following")
    bool GetIsFollowing() const
    {
        return bIsFollowing;
    }


    UFUNCTION(BlueprintPure, Category = "Drone Path Following")
    AActor* GetActivePath() const
    {
        return ActivePath;
    }


    UFUNCTION(BlueprintPure, Category = "Drone Path Following")
    bool ShouldBlockManualYaw() const;


    UFUNCTION(BlueprintCallable, Category = "Drone Path Following")
    void ReleaseYawHold();


    // ---------------------------------------------------------
    // Path following tuning
    // ---------------------------------------------------------

    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Tuning",
        meta = (ClampMin = "1.0")
    )
    float LookAheadDistanceCm = 500.0f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Tuning",
        meta = (ClampMin = "0.0")
    )
    float MaxAccelerationCmPerSec2 = 300.0f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Tuning",
        meta = (ClampMin = "1.0")
    )
    float BrakingDistanceCm = 1000.0f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Tuning",
        meta = (ClampMin = "0.0")
    )
    float ArrivalToleranceCm = 100.0f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Tuning",
        meta = (ClampMin = "0.0")
    )
    float StartArrivalToleranceCm = 100.0f;


    /*
     * Resolution used when locally searching for the nearest
     * forward point on the spline.
     *
     * This replaces the global
     * FindInputKeyClosestToWorldLocation() search.
     */
    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Tuning",
        meta = (ClampMin = "1.0")
    )
    float ProgressSearchStepCm = 25.0f;

    UPROPERTY(
        BlueprintAssignable,
        Category = "Drone Path"
    )
    FOnDronePathFinished OnPathFinished;


    // ---------------------------------------------------------
    // Yaw
    // ---------------------------------------------------------

    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Yaw"
    )
    EDronePathYawMode YawMode =
        EDronePathYawMode::FollowMovement;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Yaw",
        meta = (ClampMin = "0.0")
    )
    float YawGain = 1.0f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Yaw",
        meta = (ClampMin = "0.0")
    )
    float MaxYawRateDegPerSec = 45.0f;


    // ---------------------------------------------------------
    // Noise
    // ---------------------------------------------------------

    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Noise"
    )
    bool bEnableNoise = false;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Noise"
    )
    int32 NoiseSeed = 42;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Noise",
        meta = (ClampMin = "0.0")
    )
    float HorizontalNoiseMps = 0.5f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Noise",
        meta = (ClampMin = "0.0")
    )
    float VerticalNoiseMps = 0.0f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Noise",
        meta = (ClampMin = "0.01")
    )
    float NoiseUpdateIntervalS = 1.0f;


    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Drone Path Following|Noise",
        meta = (ClampMin = "0.0")
    )
    float NoiseSmoothingSpeed = 2.0f;


private:

    UPROPERTY()
    TObjectPtr<UPrimitiveComponent> PhysicsBody = nullptr;

    UPROPERTY()
    TObjectPtr<AActor> ActivePath = nullptr;

    UPROPERTY()
    TObjectPtr<USplineComponent> ActiveSpline = nullptr;


    bool bIsFollowing = false;

    bool bHoldFinalYaw = false;

    bool bReachedPathStart = false;


    /*
     * Monotonically increasing progress along the spline.
     *
     * This is the important addition for self-intersecting paths.
     */
    float CurrentSplineDistanceCm = 0.0f;


    float ActiveCruiseSpeedMps = 0.0f;

    float InitialYawDeg = 0.0f;


    void UpdatePathFollowing(float DeltaTime);


    /*
     * Find the closest point only within a small FORWARD interval
     * from CurrentSplineDistanceCm.
     *
     * We never perform a global closest-point search once path
     * following has started.
     */
    float FindClosestForwardSplineDistance(
        const FVector& DroneLocation,
        float DeltaTime
    ) const;


    void UpdateYaw(
        float DeltaTime,
        const FVector& DesiredDirection
    );


    // ---------------------------------------------------------
    // Noise state
    // ---------------------------------------------------------

    FRandomStream NoiseRandomStream;

    FVector CurrentNoiseVelocityCmPerSec =
        FVector::ZeroVector;

    FVector TargetNoiseVelocityCmPerSec =
        FVector::ZeroVector;

    float NoiseTimeUntilNextTargetS = 0.0f;


    void ResetNoise();

    FVector GenerateNoiseTargetVelocity();

    FVector UpdateNoise(float DeltaTime);
};