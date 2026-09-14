#include "DronePathFollowerComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Components/SplineComponent.h"
#include "GameFramework/Actor.h"


UDronePathFollowerComponent::UDronePathFollowerComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}


void UDronePathFollowerComponent::BeginPlay()
{
    Super::BeginPlay();
}


void UDronePathFollowerComponent::Initialize(
    UPrimitiveComponent* InPhysicsBody)
{
    if (!InPhysicsBody)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("DronePathFollower: Initialize received null PhysicsBody")
        );

        return;
    }

    PhysicsBody = InPhysicsBody;

    if (!PhysicsBody->IsSimulatingPhysics())
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "DronePathFollower: %s is not currently simulating physics"
            ),
            *PhysicsBody->GetName()
        );
    }

    UE_LOG(
        LogTemp,
        Log,
        TEXT(
            "DronePathFollower initialized with physics body: %s"
        ),
        *PhysicsBody->GetName()
    );
}


void UDronePathFollowerComponent::StartFollowingPath(
    AActor* PathActor,
    float CruiseSpeedMps)
{
    if (!PhysicsBody)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "DronePathFollower: cannot start path; "
                "Initialize() has not been called"
            )
        );

        return;
    }


    if (!PathActor)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "DronePathFollower: cannot start path; "
                "PathActor is null"
            )
        );

        return;
    }


    USplineComponent* Spline =
        PathActor->FindComponentByClass<USplineComponent>();


    if (!Spline)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "DronePathFollower: actor %s contains no SplineComponent"
            ),
            *PathActor->GetName()
        );

        return;
    }


    if (Spline->GetNumberOfSplinePoints() < 2)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "DronePathFollower: path %s must contain "
                "at least two spline points"
            ),
            *PathActor->GetName()
        );

        return;
    }


    if (CruiseSpeedMps <= 0.0f)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "DronePathFollower: CruiseSpeedMps must be > 0"
            )
        );

        return;
    }


    ActivePath = PathActor;
    ActiveSpline = Spline;
    ActiveCruiseSpeedMps = CruiseSpeedMps;


    /*
     * Every path always begins at spline distance zero.
     */
    bReachedPathStart = false;

    CurrentSplineDistanceCm = 0.0f;


    InitialYawDeg =
        PhysicsBody->GetComponentRotation().Yaw;


    bHoldFinalYaw = false;
    bIsFollowing = true;


    UE_LOG(
        LogTemp,
        Warning,
        TEXT(
            "DronePathFollower started path %s at %.2f m/s"
        ),
        *ActivePath->GetName(),
        ActiveCruiseSpeedMps
    );
}


void UDronePathFollowerComponent::StopFollowingPath()
{
    if (!bIsFollowing)
    {
        return;
    }


    bReachedPathStart = false;

    bIsFollowing = false;

    bHoldFinalYaw = true;

    CurrentSplineDistanceCm = 0.0f;


    if (PhysicsBody)
    {
        PhysicsBody->SetPhysicsLinearVelocity(
            FVector::ZeroVector,
            false
        );


        FVector AngularVelocity =
            PhysicsBody->GetPhysicsAngularVelocityInDegrees();

        AngularVelocity.Z = 0.0f;


        PhysicsBody->SetPhysicsAngularVelocityInDegrees(
            AngularVelocity,
            false
        );
    }


    UE_LOG(
        LogTemp,
        Warning,
        TEXT("DronePathFollower stopped")
    );


    ActivePath = nullptr;
    ActiveSpline = nullptr;
    ActiveCruiseSpeedMps = 0.0f;


    CurrentNoiseVelocityCmPerSec =
        FVector::ZeroVector;

    TargetNoiseVelocityCmPerSec =
        FVector::ZeroVector;

    NoiseTimeUntilNextTargetS = 0.0f;
}


void UDronePathFollowerComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(
        DeltaTime,
        TickType,
        ThisTickFunction
    );


    if (!bIsFollowing)
    {
        return;
    }


    UpdatePathFollowing(DeltaTime);
}


void UDronePathFollowerComponent::UpdatePathFollowing(
    float DeltaTime)
{
    if (!PhysicsBody || !ActiveSpline || !ActivePath)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "DronePathFollower lost required references; stopping"
            )
        );

        StopFollowingPath();

        return;
    }


    if (DeltaTime <= SMALL_NUMBER)
    {
        return;
    }


    const FVector DroneLocation =
        PhysicsBody->GetComponentLocation();


    const float SplineLength =
        ActiveSpline->GetSplineLength();


    const FVector StartLocation =
        ActiveSpline->GetLocationAtDistanceAlongSpline(
            0.0f,
            ESplineCoordinateSpace::World
        );


    /*
     * ------------------------------------------------------------
     * 1. Reach the beginning of the spline first.
     * ------------------------------------------------------------
     */

    if (!bReachedPathStart)
    {
        const float DistanceToStart =
            FVector::Distance(
                DroneLocation,
                StartLocation
            );


        if (DistanceToStart <= StartArrivalToleranceCm)
        {
            bReachedPathStart = true;

            /*
             * Path progress starts exactly at zero.
             */
            CurrentSplineDistanceCm = 0.0f;


            /*
             * The deterministic disturbance sequence also begins
             * exactly at the start of the experimental path.
             */
            ResetNoise();


            UE_LOG(
                LogTemp,
                Warning,
                TEXT(
                    "DronePathFollower reached start of path %s"
                ),
                *ActivePath->GetName()
            );

            return;
        }


        const FVector ToStart =
            StartLocation - DroneLocation;


        const FVector DesiredDirection =
            ToStart.GetSafeNormal();


        if (DesiredDirection.IsNearlyZero())
        {
            return;
        }


        const float CruiseSpeedCmPerSec =
            ActiveCruiseSpeedMps * 100.0f;


        const FVector DesiredVelocity =
            DesiredDirection * CruiseSpeedCmPerSec;


        UpdateYaw(
            DeltaTime,
            DesiredDirection
        );


        const FVector CurrentVelocity =
            PhysicsBody->GetPhysicsLinearVelocity();


        FVector VelocityChange =
            DesiredVelocity - CurrentVelocity;


        const float MaxVelocityChangeThisFrame =
            MaxAccelerationCmPerSec2 * DeltaTime;


        VelocityChange =
            VelocityChange.GetClampedToMaxSize(
                MaxVelocityChangeThisFrame
            );


        const FVector NewVelocity =
            CurrentVelocity + VelocityChange;


        PhysicsBody->SetPhysicsLinearVelocity(
            NewVelocity,
            false
        );


        return;
    }


    /*
     * ------------------------------------------------------------
     * 2. Update spline progress.
     *
     * IMPORTANT:
     *
     * We no longer call:
     *
     *     FindInputKeyClosestToWorldLocation()
     *
     * across the entire spline.
     *
     * Instead, progress may only move forward through a small
     * physically plausible interval.
     *
     * Therefore overlapping sections cannot make the drone jump
     * backward or suddenly jump several legs ahead.
     * ------------------------------------------------------------
     */

    CurrentSplineDistanceCm =
        FindClosestForwardSplineDistance(
            DroneLocation,
            DeltaTime
        );


    const float RemainingSplineDistanceCm =
        FMath::Max(
            SplineLength - CurrentSplineDistanceCm,
            0.0f
        );


    /*
     * ------------------------------------------------------------
     * 3. Determine endpoint and completion.
     * ------------------------------------------------------------
     */

    const FVector EndLocation =
        ActiveSpline->GetLocationAtDistanceAlongSpline(
            SplineLength,
            ESplineCoordinateSpace::World
        );


    const float DistanceToEnd =
        FVector::Distance(
            DroneLocation,
            EndLocation
        );


    /*
     * Require BOTH:
     *
     *  - spline progress near the end
     *  - physical position near the end
     *
     * This matters for self-intersecting paths where the endpoint
     * might also exist somewhere earlier in the trajectory.
     */
    const float EndProgressToleranceCm =
        FMath::Max(
            ArrivalToleranceCm,
            ProgressSearchStepCm * 2.0f
        );


    if (
        RemainingSplineDistanceCm <= EndProgressToleranceCm &&
        DistanceToEnd <= ArrivalToleranceCm
        )
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "DronePathFollower reached end of path %s"
            ),
            *ActivePath->GetName()
        );


        /*
         * Save the path reference because StopFollowingPath()
         * clears ActivePath.
         */
        AActor* FinishedPath =
            ActivePath;


        StopFollowingPath();


        /*
         * Fired only for a naturally completed path.
         *
         * Manual StopFollowingPath() and error-triggered stops
         * do not fire this event.
         */
        OnPathFinished.Broadcast(
            FinishedPath
        );


        return;
    }


    /*
     * ------------------------------------------------------------
     * 4. Look ahead from OUR KNOWN PROGRESS.
     *
     * This is now completely independent of any other overlapping
     * parts of the spline.
     * ------------------------------------------------------------
     */

    const float TargetDistanceAlongSpline =
        FMath::Min(
            CurrentSplineDistanceCm + LookAheadDistanceCm,
            SplineLength
        );


    const FVector TargetLocation =
        ActiveSpline->GetLocationAtDistanceAlongSpline(
            TargetDistanceAlongSpline,
            ESplineCoordinateSpace::World
        );


    const FVector ToTarget =
        TargetLocation - DroneLocation;


    const FVector DesiredDirection =
        ToTarget.GetSafeNormal();


    if (DesiredDirection.IsNearlyZero())
    {
        return;
    }


    /*
     * ------------------------------------------------------------
     * 5. Cruise speed.
     * ------------------------------------------------------------
     */

    const float CruiseSpeedCmPerSec =
        ActiveCruiseSpeedMps * 100.0f;


    /*
     * Brake according to REMAINING SPLINE DISTANCE rather than
     * Euclidean distance to the endpoint.
     *
     * This is also important for paths that cross near their
     * endpoint earlier in the trajectory.
     */
    const float SafeBrakingDistance =
        FMath::Max(
            BrakingDistanceCm,
            1.0f
        );


    const float SpeedScale =
        FMath::Clamp(
            RemainingSplineDistanceCm / SafeBrakingDistance,
            0.0f,
            1.0f
        );


    const float DesiredSpeedCmPerSec =
        CruiseSpeedCmPerSec * SpeedScale;


    /*
     * Nominal spline-following velocity.
     */
    const FVector NominalDesiredVelocity =
        DesiredDirection * DesiredSpeedCmPerSec;


    /*
     * Seeded smooth disturbance.
     */
    const FVector NoiseVelocity =
        UpdateNoise(DeltaTime);


    /*
     * Fade noise while braking at the final endpoint.
     */
    const FVector EffectiveNoiseVelocity =
        NoiseVelocity * SpeedScale;


    const FVector DesiredVelocity =
        NominalDesiredVelocity +
        EffectiveNoiseVelocity;


    /*
     * Yaw follows final commanded movement direction.
     */
    if (!DesiredVelocity.IsNearlyZero())
    {
        UpdateYaw(
            DeltaTime,
            DesiredVelocity.GetSafeNormal()
        );
    }


    /*
     * ------------------------------------------------------------
     * 6. Acceleration limiting.
     * ------------------------------------------------------------
     */

    const FVector CurrentVelocity =
        PhysicsBody->GetPhysicsLinearVelocity();


    FVector VelocityChange =
        DesiredVelocity - CurrentVelocity;


    const float MaxVelocityChangeThisFrame =
        MaxAccelerationCmPerSec2 * DeltaTime;


    VelocityChange =
        VelocityChange.GetClampedToMaxSize(
            MaxVelocityChangeThisFrame
        );


    const FVector NewVelocity =
        CurrentVelocity + VelocityChange;


    PhysicsBody->SetPhysicsLinearVelocity(
        NewVelocity,
        false
    );
}


float UDronePathFollowerComponent::FindClosestForwardSplineDistance(
    const FVector& DroneLocation,
    float DeltaTime) const
{
    if (!ActiveSpline)
    {
        return CurrentSplineDistanceCm;
    }


    const float SplineLength =
        ActiveSpline->GetSplineLength();


    const float SearchStart =
        FMath::Clamp(
            CurrentSplineDistanceCm,
            0.0f,
            SplineLength
        );


    /*
     * Search only a physically plausible amount ahead.
     *
     * At 15 m/s and ~60 FPS:
     *
     *     speed per frame ~= 25 cm
     *
     * We allow several times that amount plus a minimum 100 cm
     * window for robustness.
     *
     * Critically, we are NOT searching thousands of metres of
     * overlapping spline.
     */
    const float ExpectedTravelThisFrameCm =
        ActiveCruiseSpeedMps *
        100.0f *
        DeltaTime;


    const float SearchAheadCm =
        FMath::Max(
            100.0f,
            ExpectedTravelThisFrameCm * 4.0f
        );


    const float SearchEnd =
        FMath::Min(
            SearchStart + SearchAheadCm,
            SplineLength
        );


    const float SafeStepCm =
        FMath::Max(
            ProgressSearchStepCm,
            1.0f
        );


    float BestDistance =
        SearchStart;


    FVector BestLocation =
        ActiveSpline->GetLocationAtDistanceAlongSpline(
            BestDistance,
            ESplineCoordinateSpace::World
        );


    float BestDistanceSquared =
        FVector::DistSquared(
            DroneLocation,
            BestLocation
        );


    /*
     * Strict '<' is intentional.
     *
     * If two overlapping parts of the spline are exactly equally
     * close, the EARLIER forward position wins instead of jumping
     * farther ahead.
     */
    for (
        float TestDistance = SearchStart + SafeStepCm;
        TestDistance < SearchEnd;
        TestDistance += SafeStepCm
        )
    {
        const FVector TestLocation =
            ActiveSpline->GetLocationAtDistanceAlongSpline(
                TestDistance,
                ESplineCoordinateSpace::World
            );


        const float DistanceSquared =
            FVector::DistSquared(
                DroneLocation,
                TestLocation
            );


        if (DistanceSquared < BestDistanceSquared)
        {
            BestDistanceSquared =
                DistanceSquared;

            BestDistance =
                TestDistance;
        }
    }


    /*
     * Explicitly test the end of the search interval as well.
     */
    if (SearchEnd > SearchStart)
    {
        const FVector SearchEndLocation =
            ActiveSpline->GetLocationAtDistanceAlongSpline(
                SearchEnd,
                ESplineCoordinateSpace::World
            );


        const float SearchEndDistanceSquared =
            FVector::DistSquared(
                DroneLocation,
                SearchEndLocation
            );


        if (SearchEndDistanceSquared < BestDistanceSquared)
        {
            BestDistance =
                SearchEnd;
        }
    }


    /*
     * By construction:
     *
     *     BestDistance >= CurrentSplineDistanceCm
     *
     * so progress can never move backward.
     */
    return BestDistance;
}


void UDronePathFollowerComponent::UpdateYaw(
    float DeltaTime,
    const FVector& DesiredDirection)
{
    if (!PhysicsBody)
    {
        return;
    }


    FVector HorizontalDirection(
        DesiredDirection.X,
        DesiredDirection.Y,
        0.0f
    );


    const FRotator CurrentRotation =
        PhysicsBody->GetComponentRotation();


    float DesiredYawDeg =
        CurrentRotation.Yaw;


    switch (YawMode)
    {
    case EDronePathYawMode::KeepInitial:
    {
        DesiredYawDeg =
            InitialYawDeg;

        break;
    }


    case EDronePathYawMode::FollowMovement:
    {
        if (HorizontalDirection.IsNearlyZero())
        {
            return;
        }


        HorizontalDirection.Normalize();


        DesiredYawDeg =
            HorizontalDirection.Rotation().Yaw;


        break;
    }
    }


    const float YawErrorDeg =
        FMath::FindDeltaAngleDegrees(
            CurrentRotation.Yaw,
            DesiredYawDeg
        );


    const float DesiredYawRateDegPerSec =
        FMath::Clamp(
            YawErrorDeg * YawGain,
            -MaxYawRateDegPerSec,
            MaxYawRateDegPerSec
        );


    FVector AngularVelocityDegPerSec =
        PhysicsBody->GetPhysicsAngularVelocityInDegrees();


    AngularVelocityDegPerSec.Z =
        DesiredYawRateDegPerSec;


    PhysicsBody->SetPhysicsAngularVelocityInDegrees(
        AngularVelocityDegPerSec,
        false
    );
}


void UDronePathFollowerComponent::ReleaseYawHold()
{
    bHoldFinalYaw = false;
}


bool UDronePathFollowerComponent::ShouldBlockManualYaw() const
{
    return bIsFollowing || bHoldFinalYaw;
}


void UDronePathFollowerComponent::ResetNoise()
{
    NoiseRandomStream.Initialize(NoiseSeed);


    CurrentNoiseVelocityCmPerSec =
        FVector::ZeroVector;


    TargetNoiseVelocityCmPerSec =
        FVector::ZeroVector;


    NoiseTimeUntilNextTargetS =
        FMath::Max(
            NoiseUpdateIntervalS,
            0.01f
        );


    if (bEnableNoise)
    {
        TargetNoiseVelocityCmPerSec =
            GenerateNoiseTargetVelocity();
    }
}


FVector UDronePathFollowerComponent::GenerateNoiseTargetVelocity()
{
    const float AngleRad =
        NoiseRandomStream.FRandRange(
            0.0f,
            2.0f * PI
        );


    const float HorizontalMagnitudeCmPerSec =
        NoiseRandomStream.FRandRange(
            0.0f,
            HorizontalNoiseMps * 100.0f
        );


    const float X =
        FMath::Cos(AngleRad) *
        HorizontalMagnitudeCmPerSec;


    const float Y =
        FMath::Sin(AngleRad) *
        HorizontalMagnitudeCmPerSec;


    const float VerticalLimitCmPerSec =
        VerticalNoiseMps * 100.0f;


    const float Z =
        NoiseRandomStream.FRandRange(
            -VerticalLimitCmPerSec,
            VerticalLimitCmPerSec
        );


    return FVector(X, Y, Z);
}


FVector UDronePathFollowerComponent::UpdateNoise(
    float DeltaTime)
{
    if (!bEnableNoise)
    {
        CurrentNoiseVelocityCmPerSec =
            FVector::ZeroVector;

        return FVector::ZeroVector;
    }


    NoiseTimeUntilNextTargetS -=
        DeltaTime;


    const float SafeUpdateInterval =
        FMath::Max(
            NoiseUpdateIntervalS,
            0.01f
        );


    while (NoiseTimeUntilNextTargetS <= 0.0f)
    {
        TargetNoiseVelocityCmPerSec =
            GenerateNoiseTargetVelocity();


        NoiseTimeUntilNextTargetS +=
            SafeUpdateInterval;
    }


    const float Alpha =
        1.0f -
        FMath::Exp(
            -NoiseSmoothingSpeed * DeltaTime
        );


    CurrentNoiseVelocityCmPerSec =
        FMath::Lerp(
            CurrentNoiseVelocityCmPerSec,
            TargetNoiseVelocityCmPerSec,
            Alpha
        );


    return CurrentNoiseVelocityCmPerSec;
}