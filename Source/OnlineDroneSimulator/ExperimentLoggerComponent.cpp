#include "ExperimentLoggerComponent.h"

#include "DronePathFollowerComponent.h"

#include "Components/PrimitiveComponent.h"

#include "HAL/FileManager.h"

#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/EngineVersion.h"

#include "Kismet/KismetRenderingLibrary.h"
#include "Kismet/GameplayStatics.h"

#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Engine/EngineTypes.h"

#include "Camera/CameraTypes.h"

#include "UnrealClient.h"
#include "RHITypes.h"

#include "ImageUtils.h"
#include "ImageCore.h"


namespace
{
    constexpr float SkylineSkyDepthThresholdCm =
        99000000.0f;


    FString PathYawModeToString(
        const EDronePathYawMode YawMode)
    {
        switch (YawMode)
        {
        case EDronePathYawMode::KeepInitial:
            return TEXT("keep_initial");

        case EDronePathYawMode::FollowMovement:
            return TEXT("follow_movement");

        default:
            return TEXT("unknown");
        }
    }


    /*
     * Enough escaping for the strings we write into settings.json.
     */
    FString EscapeJsonForSettings(
        const FString& Input)
    {
        FString Result = Input;

        Result.ReplaceInline(
            TEXT("\\"),
            TEXT("\\\\")
        );

        Result.ReplaceInline(
            TEXT("\""),
            TEXT("\\\"")
        );

        Result.ReplaceInline(
            TEXT("\n"),
            TEXT("\\n")
        );

        Result.ReplaceInline(
            TEXT("\r"),
            TEXT("\\r")
        );

        Result.ReplaceInline(
            TEXT("\t"),
            TEXT("\\t")
        );

        return Result;
    }
}


UExperimentLoggerComponent::UExperimentLoggerComponent()
{
    PrimaryComponentTick.bCanEverTick = true;

    /*
     * Path-following commands are normally issued before physics.
     *
     * Logging after physics means the camera pose and body pose
     * represent the resolved physics state for this simulation
     * step.
     */
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
}


void UExperimentLoggerComponent::BeginPlay()
{
    Super::BeginPlay();


    AActor* Owner = GetOwner();

    if (!Owner)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("ExperimentLogger has no owner")
        );

        return;
    }


    /*
     * ------------------------------------------------------------
     * Find SceneCapture components by tag.
     * ------------------------------------------------------------
     */

    TArray<USceneCaptureComponent2D*> CaptureComponents;

    Owner->GetComponents<USceneCaptureComponent2D>(
        CaptureComponents
    );


    for (USceneCaptureComponent2D* Capture : CaptureComponents)
    {
        if (!Capture)
        {
            continue;
        }


        if (Capture->ComponentHasTag(TEXT("NorthCapture")))
        {
            NorthCapture = Capture;
        }


        if (Capture->ComponentHasTag(TEXT("WestCapture")))
        {
            WestCapture = Capture;
        }


        if (Capture->ComponentHasTag(TEXT("NadirCapture")))
        {
            NadirCapture = Capture;
        }


        if (Capture->ComponentHasTag(TEXT("NorthDepthCapture")))
        {
            NorthDepthCapture = Capture;
        }


        if (Capture->ComponentHasTag(TEXT("WestDepthCapture")))
        {
            WestDepthCapture = Capture;
        }
    }


    /*
     * ------------------------------------------------------------
     * Find physical drone body.
     *
     * Preferred:
     *     Component Tag = DroneBody
     *
     * There is also a fallback for your existing BodyMesh_1 name.
     * ------------------------------------------------------------
     */

    TArray<UPrimitiveComponent*> PrimitiveComponents;

    Owner->GetComponents<UPrimitiveComponent>(
        PrimitiveComponents
    );


    for (UPrimitiveComponent* Primitive : PrimitiveComponents)
    {
        if (
            Primitive &&
            Primitive->ComponentHasTag(TEXT("DroneBody"))
            )
        {
            BodyComponent = Primitive;
            break;
        }
    }


    /*
     * Temporary convenience fallback.
     *
     * I still recommend adding the DroneBody tag in Blueprint.
     */
    if (!BodyComponent)
    {
        for (UPrimitiveComponent* Primitive : PrimitiveComponents)
        {
            if (
                Primitive &&
                Primitive->GetName().Equals(TEXT("BodyMesh_1"))
                )
            {
                BodyComponent = Primitive;

                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT(
                        "ExperimentLogger found BodyMesh_1 by name. "
                        "Add Component Tag 'DroneBody' for robustness."
                    )
                );

                break;
            }
        }
    }

    /*
     * ------------------------------------------------------------
     * Prevent navigation cameras from seeing the UAV itself.
     *
     * This affects only these SceneCapture components.
     * The drone remains visible in the normal game/editor view
     * and physics/collision are unaffected.
     * ------------------------------------------------------------
     */

    auto HideOwnerFromCapture =
        [Owner](
            USceneCaptureComponent2D* Capture,
            const TCHAR* CaptureName
            )
        {
            if (!Capture)
            {
                return;
            }

            /*
             * true:
             * Also hide primitive components belonging to child actors.
             *
             * This is useful if propellers, payloads, etc. are separate
             * child actors attached to the drone.
             */
            Capture->HideActorComponents(
                Owner,
                true
            );

            UE_LOG(
                LogTemp,
                Warning,
                TEXT(
                    "ExperimentLogger: hidden UAV from %s"
                ),
                CaptureName
            );
        };


    HideOwnerFromCapture(
        NorthCapture,
        TEXT("NorthCapture")
    );

    HideOwnerFromCapture(
        NorthDepthCapture,
        TEXT("NorthDepthCapture")
    );

    HideOwnerFromCapture(
        WestCapture,
        TEXT("WestCapture")
    );

    HideOwnerFromCapture(
        WestDepthCapture,
        TEXT("WestDepthCapture")
    );

    HideOwnerFromCapture(
        NadirCapture,
        TEXT("NadirCapture")
    );


    /*
     * ------------------------------------------------------------
     * Find path follower automatically.
     * ------------------------------------------------------------
     */

    PathFollower =
        Owner->FindComponentByClass<UDronePathFollowerComponent>();


    /*
     * ------------------------------------------------------------
     * Diagnostics.
     * ------------------------------------------------------------
     */

    if (NorthCapture)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Found north capture: %s"
            ),
            *NorthCapture->GetName()
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("Could not find NorthCapture")
        );
    }


    if (WestCapture)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Found west capture: %s"
            ),
            *WestCapture->GetName()
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("Could not find WestCapture")
        );
    }


    if (NadirCapture)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Found nadir capture: %s"
            ),
            *NadirCapture->GetName()
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("Could not find NadirCapture")
        );
    }


    if (NorthDepthCapture)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Found north depth capture: %s"
            ),
            *NorthDepthCapture->GetName()
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("Could not find NorthDepthCapture")
        );
    }


    if (WestDepthCapture)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Found west depth capture: %s"
            ),
            *WestDepthCapture->GetName()
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("Could not find WestDepthCapture")
        );
    }


    if (BodyComponent)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Found drone body: %s"
            ),
            *BodyComponent->GetName()
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Could not find drone body. "
                "Add Component Tag 'DroneBody' to BodyMesh_1."
            )
        );
    }


    if (PathFollower)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Found DronePathFollowerComponent: %s"
            ),
            *PathFollower->GetName()
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "ExperimentLogger could not find "
                "DronePathFollowerComponent"
            )
        );
    }
}


void UExperimentLoggerComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(
        DeltaTime,
        TickType,
        ThisTickFunction
    );


    if (!bExperimentRunning)
    {
        return;
    }

    SimulationTickIndex++;


    /*
     * ------------------------------------------------------------
     * Sanity-check the assumed fixed timestep.
     *
     * We check the first 30 simulation ticks.
     * ------------------------------------------------------------
     */

    if (
        !bTickRateValidationDone &&
        ExpectedSimulationRateHz > SMALL_NUMBER
        )
    {
        const double ExpectedDeltaTime =
            1.0 /
            static_cast<double>(
                ExpectedSimulationRateHz
                );


        const double Error =
            FMath::Abs(
                static_cast<double>(DeltaTime) -
                ExpectedDeltaTime
            );


        MaxTickDeltaErrorS =
            FMath::Max(
                MaxTickDeltaErrorS,
                Error
            );


        TickValidationCount++;


        if (TickValidationCount >= 30)
        {
            bTickRateValidationDone = true;


            /*
             * 1% tolerance.
             *
             * With UE fixed frame rate configured correctly,
             * DeltaTime should effectively be constant.
             */
            const double AllowedError =
                ExpectedDeltaTime * 0.01;


            if (MaxTickDeltaErrorS > AllowedError)
            {
                UE_LOG(
                    LogTemp,
                    Error,
                    TEXT(
                        "VO logging expected fixed %.3f Hz simulation "
                        "but DeltaTime varied. "
                        "Max error over first 30 ticks = %.9f s. "
                        "Check Project Settings -> Fixed Frame Rate."
                    ),
                    ExpectedSimulationRateHz,
                    MaxTickDeltaErrorS
                );
            }
            else
            {
                UE_LOG(
                    LogTemp,
                    Warning,
                    TEXT(
                        "VO fixed-timestep check passed: %.3f Hz"
                    ),
                    ExpectedSimulationRateHz
                );
            }
        }
    }


    /*
     * ------------------------------------------------------------
     * Full-rate GT.
     *
     * One sample every simulation tick.
     * ------------------------------------------------------------
     */

    WriteGroundTruthSample();


    /*
     * ------------------------------------------------------------
     * Nadir image decimation.
     * ------------------------------------------------------------
     */

    if (!bAutomaticNadirCapture)
    {
        return;
    }

    if (
        bAutomaticNadirCapture &&
        SimulationTickIndex % FMath::Max(NadirCaptureEveryNTicks, 1) == 0
        )
    {
        CaptureNadirObservation();
    }
}


bool UExperimentLoggerComponent::ComputeCameraCalibration(
    USceneCaptureComponent2D* Capture,
    const TCHAR* CameraName,
    FExperimentCameraCalibration& OutCalibration) const
{
    if (!Capture)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot calibrate %s: capture is null"
            ),
            CameraName
        );

        return false;
    }


    if (!Capture->TextureTarget)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot calibrate %s: no TextureTarget"
            ),
            CameraName
        );

        return false;
    }


    if (
        Capture->ProjectionType !=
        ECameraProjectionMode::Perspective
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot calibrate %s: "
                "camera must use Perspective projection"
            ),
            CameraName
        );

        return false;
    }


    if (Capture->bUseCustomProjectionMatrix)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot calibrate %s: "
                "custom projection matrix is enabled"
            ),
            CameraName
        );

        return false;
    }


    OutCalibration.Width =
        Capture->TextureTarget->SizeX;


    OutCalibration.Height =
        Capture->TextureTarget->SizeY;


    OutCalibration.HorizontalFovDeg =
        Capture->FOVAngle;


    const double HorizontalFovRad =
        FMath::DegreesToRadians(
            OutCalibration.HorizontalFovDeg
        );


    const double AspectRatio =
        static_cast<double>(
            OutCalibration.Width
            ) /
        static_cast<double>(
            OutCalibration.Height
            );


    const double VerticalFovRad =
        2.0 *
        FMath::Atan(
            FMath::Tan(
                HorizontalFovRad * 0.5
            ) /
            AspectRatio
        );


    OutCalibration.VerticalFovDeg =
        FMath::RadiansToDegrees(
            VerticalFovRad
        );


    OutCalibration.Fx =
        static_cast<double>(
            OutCalibration.Width
            ) /
        (
            2.0 *
            FMath::Tan(
                HorizontalFovRad * 0.5
            )
            );


    OutCalibration.Fy =
        static_cast<double>(
            OutCalibration.Height
            ) /
        (
            2.0 *
            FMath::Tan(
                VerticalFovRad * 0.5
            )
            );


    OutCalibration.Cx =
        static_cast<double>(
            OutCalibration.Width
            ) * 0.5;


    OutCalibration.Cy =
        static_cast<double>(
            OutCalibration.Height
            ) * 0.5;


    return true;
}


double UExperimentLoggerComponent::GetExperimentSimTimeSeconds() const
{
    if (ExpectedSimulationRateHz <= SMALL_NUMBER)
    {
        return 0.0;
    }

    return
        static_cast<double>(SimulationTickIndex) /
        static_cast<double>(ExpectedSimulationRateHz);
}


bool UExperimentLoggerComponent::TraceNadirSurface(
    double& OutAglM,
    double& OutTerrainElevationM) const
{
    if (!NadirCapture || !GetWorld())
    {
        return false;
    }


    const FVector TraceStart =
        NadirCapture->GetComponentLocation();


    /*
     * UE SceneCapture camera optical axis is local +X.
     *
     * Therefore GetForwardVector() is the principal ray.
     */
    const FVector TraceDirection =
        NadirCapture
        ->GetForwardVector()
        .GetSafeNormal();


    if (TraceDirection.IsNearlyZero())
    {
        return false;
    }


    const FVector TraceEnd =
        TraceStart +
        TraceDirection *
        GroundTraceDistanceCm;


    FCollisionQueryParams QueryParams;

    QueryParams.bTraceComplex = false;


    if (GetOwner())
    {
        /*
         * Prevent the ray from hitting the UAV itself.
         */
        QueryParams.AddIgnoredActor(
            GetOwner()
        );
    }


    FHitResult HitResult;


    const bool bHit =
        GetWorld()->LineTraceSingleByChannel(
            HitResult,
            TraceStart,
            TraceEnd,
            ECC_Visibility,
            QueryParams
        );


    if (!bHit)
    {
        return false;
    }


    /*
     * True AGL is principal-ray distance from the optical centre
     * to the first blocking imaged surface.
     *
     * Under exact nadir this is the normal vertical AGL.
     */
    OutAglM =
        FVector::Distance(
            TraceStart,
            HitResult.ImpactPoint
        ) /
        100.0;


    /*
     * Same UE world-Z datum as up_m.
     *
     * In urban scenes this may be a roof or other visible surface,
     * not necessarily geological terrain. This is intentional:
     * it is the surface intersected by the camera principal ray.
     */
    OutTerrainElevationM =
        HitResult.ImpactPoint.Z /
        100.0;


    return true;
}


double UExperimentLoggerComponent::ComputeNadirHeadingDeg() const
{
    if (!NadirCapture)
    {
        return 0.0;
    }


    /*
     * At exact nadir, the optical/forward vector is vertical and
     * therefore cannot define heading.
     *
     * UE camera local +Z corresponds to image-up.
     *
     * Project that image-up direction onto the horizontal plane.
     */
    FVector HeadingVector =
        NadirCapture->GetUpVector();


    HeadingVector.Z = 0.0f;


    /*
     * Fallback in case the camera reaches an unusual orientation.
     */
    if (
        HeadingVector.IsNearlyZero() &&
        BodyComponent
        )
    {
        HeadingVector =
            BodyComponent->GetForwardVector();

        HeadingVector.Z = 0.0f;
    }


    if (HeadingVector.IsNearlyZero())
    {
        return 0.0;
    }


    HeadingVector.Normalize();


    /*
     * Project convention:
     *
     * +X = North
     * +Y = East
     *
     * atan2(East, North) therefore gives compass heading:
     *
     * North =   0 deg
     * East  =  90 deg
     * South = 180 deg
     * West  = 270 deg
     */
    double HeadingDeg =
        FMath::RadiansToDegrees(
            FMath::Atan2(
                static_cast<double>(
                    HeadingVector.Y
                    ),
                static_cast<double>(
                    HeadingVector.X
                    )
            )
        );


    HeadingDeg =
        FMath::Fmod(
            HeadingDeg + 360.0,
            360.0
        );


    return HeadingDeg;
}


double UExperimentLoggerComponent::ComputeNadirTiltDeg() const
{
    if (!NadirCapture)
    {
        return 0.0;
    }


    /*
     * Camera optical axis = local +X.
     *
     * Tilt is angle between optical axis and world-down.
     *
     * Exact nadir:
     *     tilt = 0 deg.
     */
    const FVector OpticalDirection =
        NadirCapture
        ->GetForwardVector()
        .GetSafeNormal();


    const FVector WorldDown =
        FVector(0.0f, 0.0f, -1.0f);


    const double Dot =
        FMath::Clamp(
            static_cast<double>(
                FVector::DotProduct(
                    OpticalDirection,
                    WorldDown
                )
                ),
            -1.0,
            1.0
        );


    return
        FMath::RadiansToDegrees(
            FMath::Acos(Dot)
        );
}


bool UExperimentLoggerComponent::CollectVoSample(
    FExperimentVoSample& OutSample) const
{
    if (!NadirCapture || !BodyComponent)
    {
        return false;
    }


    OutSample.SimTimeS =
        GetExperimentSimTimeSeconds();


    /*
     * Camera pose.
     *
     * This is the actual NadirCapture world transform.
     *
     * Position follows the drone hierarchy, while orientation is
     * world-stabilized by the nadir gimbal setup.
     */

    OutSample.CameraLocationCm =
        NadirCapture->GetComponentLocation();


    OutSample.CameraQuaternion =
        NadirCapture
        ->GetComponentQuat()
        .GetNormalized();


    /*
     * ------------------------------------------------------------
     * Physical body / FC pose.
     * ------------------------------------------------------------
     */

    OutSample.BodyLocationCm =
        BodyComponent->GetComponentLocation();


    OutSample.BodyQuaternion =
        BodyComponent
        ->GetComponentQuat()
        .GetNormalized();


    /*
     * ------------------------------------------------------------
     * UE -> local ENU conversion.
     *
     * Project-wide invariant:
     *
     * UE +X = North
     * UE +Y = East
     * UE +Z = Up
     *
     * ENU is therefore:
     *
     * east  = UE Y
     * north = UE X
     * up    = UE Z
     *
     * Position refers to the camera centre because VO estimates
     * camera motion.
     * ------------------------------------------------------------
     */

    OutSample.EastM =
        OutSample.CameraLocationCm.Y /
        100.0;


    OutSample.NorthM =
        OutSample.CameraLocationCm.X /
        100.0;


    OutSample.UpM =
        OutSample.CameraLocationCm.Z /
        100.0;


    /*
     * ------------------------------------------------------------
     * Heading / tilt.
     * ------------------------------------------------------------
     */

    OutSample.HeadingDeg =
        ComputeNadirHeadingDeg();


    OutSample.CameraTiltDeg =
        ComputeNadirTiltDeg();


    /*
     * ------------------------------------------------------------
     * Ideal barometric relative altitude.
     *
     * Body/FC origin.
     *
     * Zero at experiment reference frame.
     * Positive upward.
     * ------------------------------------------------------------
     */

    OutSample.BaroRelativeAltM =
        (
            static_cast<double>(
                OutSample.BodyLocationCm.Z
                ) -
            InitialBodyZCm
            ) /
        100.0;


    /*
     * ------------------------------------------------------------
     * Principal-ray AGL and surface elevation.
     * ------------------------------------------------------------
     */

    double AglM = 0.0;
    double TerrainElevationM = 0.0;


    OutSample.bGroundHit =
        TraceNadirSurface(
            AglM,
            TerrainElevationM
        );


    if (OutSample.bGroundHit)
    {
        OutSample.TrueAglM =
            AglM;

        OutSample.TerrainElevationM =
            TerrainElevationM;
    }


    return true;
}


void UExperimentLoggerComponent::WriteGroundTruthSample()
{
    if (!bExperimentRunning)
    {
        return;
    }


    FExperimentVoSample Sample;


    if (!CollectVoSample(Sample))
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to collect VO ground-truth sample"
            )
        );

        return;
    }


    const FString AglString =
        Sample.bGroundHit
        ? FString::Printf(
            TEXT("%.6f"),
            Sample.TrueAglM
        )
        : TEXT("nan");


    const FString TerrainString =
        Sample.bGroundHit
        ? FString::Printf(
            TEXT("%.6f"),
            Sample.TerrainElevationM
        )
        : TEXT("nan");


    const FString CsvRow =
        FString::Printf(

            TEXT("%d,%.9f,")

            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.9f,%.9f,%.9f,%.9f,")

            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.9f,%.9f,%.9f,%.9f,")

            TEXT("%.9f,%.9f,%.9f,")

            TEXT("%.6f,%.6f,")

            TEXT("%.6f,%s,%s,%d\n"),


            GroundTruthSampleCounter,
            Sample.SimTimeS,


            Sample.CameraLocationCm.X,
            Sample.CameraLocationCm.Y,
            Sample.CameraLocationCm.Z,

            Sample.CameraQuaternion.W,
            Sample.CameraQuaternion.X,
            Sample.CameraQuaternion.Y,
            Sample.CameraQuaternion.Z,


            Sample.BodyLocationCm.X,
            Sample.BodyLocationCm.Y,
            Sample.BodyLocationCm.Z,

            Sample.BodyQuaternion.W,
            Sample.BodyQuaternion.X,
            Sample.BodyQuaternion.Y,
            Sample.BodyQuaternion.Z,


            Sample.EastM,
            Sample.NorthM,
            Sample.UpM,


            Sample.HeadingDeg,
            Sample.CameraTiltDeg,


            Sample.BaroRelativeAltM,

            *AglString,
            *TerrainString,

            Sample.bGroundHit ? 1 : 0
        );


    const bool bWritten =
        FFileHelper::SaveStringToFile(
            CsvRow,
            *VoGroundTruthCsvPath,
            FFileHelper::EEncodingOptions::AutoDetect,
            &IFileManager::Get(),
            FILEWRITE_Append
        );


    if (!bWritten)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to append VO ground truth"
            )
        );

        return;
    }


    GroundTruthSampleCounter++;
}


void UExperimentLoggerComponent::StartExperiment(
    const FString& TimeOfDay,
    float Hour,
    float TimeSpeed,
    const FString& CloudPreset,
    bool bAutomaticSkylineCapture,
    float SkylineCaptureDistanceCm,
    const FString& PathId,
    float CruiseSpeedMps)
{
    /*
     * ------------------------------------------------------------
     * Validation.
     * ------------------------------------------------------------
     */

    if (bExperimentRunning)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("Experiment is already running")
        );

        return;
    }


    if (
        !NorthCapture ||
        !NorthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "NorthCapture is not configured"
            )
        );

        return;
    }


    if (
        !WestCapture ||
        !WestCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "WestCapture is not configured"
            )
        );

        return;
    }


    if (
        !NadirCapture ||
        !NadirCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "NadirCapture is not configured"
            )
        );

        return;
    }


    if (
        !NorthDepthCapture ||
        !NorthDepthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "NorthDepthCapture is not configured"
            )
        );

        return;
    }


    if (
        !WestDepthCapture ||
        !WestDepthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "WestDepthCapture is not configured"
            )
        );

        return;
    }


    if (!BodyComponent)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "drone body is not configured"
            )
        );

        return;
    }


    if (!PathFollower)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "DronePathFollowerComponent is null"
            )
        );

        return;
    }


    if (PathId.IsEmpty())
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: PathId is empty"
            )
        );

        return;
    }


    if (CruiseSpeedMps <= 0.0f)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "CruiseSpeedMps must be > 0"
            )
        );

        return;
    }


    if (ExpectedSimulationRateHz <= 0.0f)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "ExpectedSimulationRateHz must be > 0"
            )
        );

        return;
    }


    if (NadirCaptureEveryNTicks < 1)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "NadirCaptureEveryNTicks must be >= 1"
            )
        );

        return;
    }


    /*
     * North RGB / depth correspondence.
     */
    if (
        NorthDepthCapture->TextureTarget->SizeX !=
        NorthCapture->TextureTarget->SizeX ||

        NorthDepthCapture->TextureTarget->SizeY !=
        NorthCapture->TextureTarget->SizeY
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "north RGB and depth resolutions differ"
            )
        );

        return;
    }


    if (
        !FMath::IsNearlyEqual(
            NorthDepthCapture->FOVAngle,
            NorthCapture->FOVAngle,
            0.001f
        )
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "north RGB and depth FOV differ"
            )
        );

        return;
    }


    /*
     * West RGB / depth correspondence.
     */
    if (
        WestDepthCapture->TextureTarget->SizeX !=
        WestCapture->TextureTarget->SizeX ||

        WestDepthCapture->TextureTarget->SizeY !=
        WestCapture->TextureTarget->SizeY
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "west RGB and depth resolutions differ"
            )
        );

        return;
    }


    if (
        !FMath::IsNearlyEqual(
            WestDepthCapture->FOVAngle,
            WestCapture->FOVAngle,
            0.001f
        )
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: "
                "west RGB and depth FOV differ"
            )
        );

        return;
    }


    /*
     * ------------------------------------------------------------
     * Calibrate all RGB cameras independently.
     * ------------------------------------------------------------
     */

    FExperimentCameraCalibration NorthCalibration;
    FExperimentCameraCalibration WestCalibration;
    FExperimentCameraCalibration NadirCalibration;


    if (
        !ComputeCameraCalibration(
            NorthCapture,
            TEXT("NorthCapture"),
            NorthCalibration
        )
        )
    {
        return;
    }


    if (
        !ComputeCameraCalibration(
            WestCapture,
            TEXT("WestCapture"),
            WestCalibration
        )
        )
    {
        return;
    }


    if (
        !ComputeCameraCalibration(
            NadirCapture,
            TEXT("NadirCapture"),
            NadirCalibration
        )
        )
    {
        return;
    }


    /*
     * ------------------------------------------------------------
     * Define run reference instant.
     * ------------------------------------------------------------
     */

    if (!GetWorld())
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot start experiment: world is null"
            )
        );

        return;
    }


    ExperimentStartWorldTimeS =
        static_cast<double>(
            GetWorld()->GetTimeSeconds()
            );


    InitialBodyZCm =
        static_cast<double>(
            BodyComponent
            ->GetComponentLocation()
            .Z
            );


    /*
     * h0 is measured directly from the simulator at the reference
     * instant and never inferred from the later trajectory.
     */
    double InitialTerrainElevationM = 0.0;


    const bool bInitialAglValid =
        TraceNadirSurface(
            InitialAglM,
            InitialTerrainElevationM
        );


    if (!bInitialAglValid)
    {
        if (bAutomaticNadirCapture)
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT(
                    "Cannot start experiment: "
                    "initial nadir principal-ray trace did not hit a surface"
                )
            );

            return;
        }


        /*
         * Preserve the previous no-automatic-VO behavior: the run may
         * still start even if reference AGL is unavailable. The JSON
         * metadata marks h0 as invalid instead of silently implying
         * that the zero fallback is a measured value.
         */
        InitialAglM = 0.0;

        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Initial nadir principal-ray trace did not hit a surface. "
                "Automatic nadir capture is disabled, so the experiment "
                "will continue with h0 marked invalid."
            )
        );
    }

    /*
     * Warn if this isn't approximately a nadir camera.
     *
     * The line trace still works under tilt.
     */
    const double InitialTiltDeg =
        ComputeNadirTiltDeg();


    if (InitialTiltDeg > 5.0)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Nadir camera initial tilt is %.3f deg. "
                "Expected approximately 0 deg."
            ),
            InitialTiltDeg
        );
    }


    /*
     * ------------------------------------------------------------
     * Create directories.
     * ------------------------------------------------------------
     */

    CurrentRunId =
        TEXT("Run_") +
        FDateTime::Now().ToString(
            TEXT("%Y%m%d_%H%M%S")
        );


    CurrentRunDirectory =
        FPaths::Combine(
            FPaths::ProjectSavedDir(),
            TEXT("SimulatorRuns"),
            CurrentRunId
        );


    /*
     * Skyline.
     */
    SkylineDirectory =
        FPaths::Combine(
            CurrentRunDirectory,
            TEXT("skyline")
        );


    SkylineImagesDirectory =
        FPaths::Combine(
            SkylineDirectory,
            TEXT("images")
        );


    SkylineSimDirectory =
        FPaths::Combine(
            SkylineDirectory,
            TEXT("sim")
        );


    SkylineCsvPath =
        FPaths::Combine(
            SkylineDirectory,
            TEXT("observations.csv")
        );


    /*
     * VO.
     */
    VoDirectory =
        FPaths::Combine(
            CurrentRunDirectory,
            TEXT("vo")
        );


    VoImagesDirectory =
        FPaths::Combine(
            VoDirectory,
            TEXT("images")
        );


    VoFramesCsvPath =
        FPaths::Combine(
            VoDirectory,
            TEXT("frames.csv")
        );


    VoGroundTruthCsvPath =
        FPaths::Combine(
            VoDirectory,
            TEXT("groundtruth.csv")
        );


    IFileManager::Get().MakeDirectory(
        *SkylineImagesDirectory,
        true
    );


    IFileManager::Get().MakeDirectory(
        *SkylineSimDirectory,
        true
    );


    IFileManager::Get().MakeDirectory(
        *VoImagesDirectory,
        true
    );


    /*
     * ------------------------------------------------------------
     * Create skyline CSV.
     * ------------------------------------------------------------
     */

    const FString SkylineCsvHeader =
        TEXT("observation_id,sim_time_s,")
        TEXT("north_ue_x_cm,north_ue_y_cm,north_ue_z_cm,")
        TEXT("north_ue_yaw_deg,north_ue_pitch_deg,north_ue_roll_deg,")
        TEXT("north_ue_quat_w,north_ue_quat_x,north_ue_quat_y,north_ue_quat_z,")
        TEXT("nadir_ue_x_cm,nadir_ue_y_cm,nadir_ue_z_cm,")
        TEXT("nadir_ue_yaw_deg,nadir_ue_pitch_deg,nadir_ue_roll_deg,")
        TEXT("nadir_ue_quat_w,nadir_ue_quat_x,nadir_ue_quat_y,nadir_ue_quat_z,")
        TEXT("image_path,sim_sky_mask_path,")
        TEXT("west_ue_x_cm,west_ue_y_cm,west_ue_z_cm,")
        TEXT("west_ue_yaw_deg,west_ue_pitch_deg,west_ue_roll_deg,")
        TEXT("west_ue_quat_w,west_ue_quat_x,west_ue_quat_y,west_ue_quat_z,")
        TEXT("west_image_path,west_sim_sky_mask_path,")
        TEXT("vo_frame_id,vo_synchronized\n");


    if (
        !FFileHelper::SaveStringToFile(
            SkylineCsvHeader,
            *SkylineCsvPath
        )
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to create skyline CSV"
            )
        );

        return;
    }


    /*
     * ------------------------------------------------------------
     * Create VO frames.csv.
     *
     * One row per actual nadir image.
     * ------------------------------------------------------------
     */

    const FString VoFramesHeader =
        TEXT("frame_id,sim_time_s,image_path,")
        TEXT("cam_ue_x_cm,cam_ue_y_cm,cam_ue_z_cm,")
        TEXT("cam_ue_qw,cam_ue_qx,cam_ue_qy,cam_ue_qz,")
        TEXT("body_ue_x_cm,body_ue_y_cm,body_ue_z_cm,")
        TEXT("body_ue_qw,body_ue_qx,body_ue_qy,body_ue_qz,")
        TEXT("east_m,north_m,up_m,")
        TEXT("heading_deg,camera_tilt_deg,")
        TEXT("baro_relative_alt_m,true_agl_m,terrain_elevation_m,ground_hit\n");


    if (
        !FFileHelper::SaveStringToFile(
            VoFramesHeader,
            *VoFramesCsvPath
        )
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to create VO frames.csv"
            )
        );

        return;
    }


    /*
     * ------------------------------------------------------------
     * Create dense groundtruth.csv.
     *
     * Same physical quantities, no image_path.
     * ------------------------------------------------------------
     */

    const FString VoGroundTruthHeader =
        TEXT("sample_id,sim_time_s,")
        TEXT("cam_ue_x_cm,cam_ue_y_cm,cam_ue_z_cm,")
        TEXT("cam_ue_qw,cam_ue_qx,cam_ue_qy,cam_ue_qz,")
        TEXT("body_ue_x_cm,body_ue_y_cm,body_ue_z_cm,")
        TEXT("body_ue_qw,body_ue_qx,body_ue_qy,body_ue_qz,")
        TEXT("east_m,north_m,up_m,")
        TEXT("heading_deg,camera_tilt_deg,")
        TEXT("baro_relative_alt_m,true_agl_m,terrain_elevation_m,ground_hit\n");


    if (
        !FFileHelper::SaveStringToFile(
            VoGroundTruthHeader,
            *VoGroundTruthCsvPath
        )
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to create VO groundtruth.csv"
            )
        );

        return;
    }


    /*
     * ------------------------------------------------------------
     * Run metadata.
     * ------------------------------------------------------------
     */

    const FString LevelName =
        UGameplayStatics::GetCurrentLevelName(
            this,
            true
        );


    const float SkylineCaptureDistanceM =
        SkylineCaptureDistanceCm /
        100.0f;


    const float LookAheadDistanceM =
        PathFollower->LookAheadDistanceCm /
        100.0f;


    const float MaxAccelerationMps2 =
        PathFollower->MaxAccelerationCmPerSec2 /
        100.0f;


    const float BrakingDistanceM =
        PathFollower->BrakingDistanceCm /
        100.0f;


    const float ArrivalToleranceM =
        PathFollower->ArrivalToleranceCm /
        100.0f;


    const float StartArrivalToleranceM =
        PathFollower->StartArrivalToleranceCm /
        100.0f;


    const float ProgressSearchStepM =
        PathFollower->ProgressSearchStepCm /
        100.0f;


    const FString YawModeString =
        PathYawModeToString(
            PathFollower->YawMode
        );


    const double ImageRateHz =
        static_cast<double>(
            ExpectedSimulationRateHz
            ) /
        static_cast<double>(
            NadirCaptureEveryNTicks
            );


    const double Gsd0MPerPx =
        InitialAglM /
        NadirCalibration.Fx;


    /*
     * Camera-to-body relative transform at the experiment
     * reference frame only.
     *
     * The nadir camera orientation is world-stabilized, so this
     * relative rotation is NOT constant throughout the flight.
     *
     * Per-frame camera and body world poses remain authoritative.
     */
    const FTransform CameraRelativeToBody =
        NadirCapture
        ->GetComponentTransform()
        .GetRelativeTransform(
            BodyComponent->GetComponentTransform()
        );


    const FVector CameraRelativeLocationM =
        CameraRelativeToBody
        .GetLocation() /
        100.0f;


    const FQuat CameraRelativeQuaternion =
        CameraRelativeToBody
        .GetRotation()
        .GetNormalized();


    /*
     * ------------------------------------------------------------
     * Build settings.json.
     *
     * Constructed block-by-block to avoid one enormous Printf.
     * ------------------------------------------------------------
     */

    FString SettingsJson;


    SettingsJson += TEXT("{\n");


    SettingsJson += FString::Printf(
        TEXT("  \"run_id\": \"%s\",\n"),
        *EscapeJsonForSettings(CurrentRunId)
    );


    SettingsJson += FString::Printf(
        TEXT("  \"level\": \"%s\",\n"),
        *EscapeJsonForSettings(LevelName)
    );


    SettingsJson += FString::Printf(
        TEXT("  \"engine_version\": \"%s\",\n"),
        *EscapeJsonForSettings(
            FEngineVersion::Current().ToString()
        )
    );


    /*
     * World convention.
     */
    SettingsJson +=
        TEXT("  \"world_frame\": {\n")
        TEXT("    \"type\": \"local_non_georeferenced\",\n")
        TEXT("    \"ue_units\": \"cm\",\n")
        TEXT("    \"ue_handedness\": \"left\",\n")
        TEXT("    \"north_axis\": \"+X\",\n")
        TEXT("    \"east_axis\": \"+Y\",\n")
        TEXT("    \"up_axis\": \"+Z\",\n")
        TEXT("    \"enu_handedness\": \"right\",\n")
        TEXT("    \"enu_position_source\": \"nadir_camera_center\",\n")
        TEXT("    \"enu_mapping\": {\n")
        TEXT("      \"east_m\": \"ue_y_cm / 100\",\n")
        TEXT("      \"north_m\": \"ue_x_cm / 100\",\n")
        TEXT("      \"up_m\": \"ue_z_cm / 100\"\n")
        TEXT("    },\n")
        TEXT("    \"vertical_datum\": \"UE world Z = 0\",\n")
        TEXT("    \"georeference\": null\n")
        TEXT("  },\n");


    /*
     * Quaternion convention.
     */
    SettingsJson +=
        TEXT("  \"quaternion_convention\": {\n")
        TEXT("    \"order\": \"wxyz\",\n")
        TEXT("    \"rotation_sense\": \"component_local_to_UE_world\",\n")
        TEXT("    \"expressed_in\": \"UE_world\",\n")
        TEXT("    \"normalised\": true\n")
        TEXT("  },\n");


    /*
     * Environment.
     */
    SettingsJson +=
        TEXT("  \"environment\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"time_of_day\": \"%s\",\n"),
        *EscapeJsonForSettings(TimeOfDay)
    );


    SettingsJson += FString::Printf(
        TEXT("    \"hour\": %.3f,\n"),
        Hour
    );


    SettingsJson += FString::Printf(
        TEXT("    \"time_speed\": %.3f,\n"),
        TimeSpeed
    );


    SettingsJson += FString::Printf(
        TEXT("    \"clouds\": \"%s\"\n"),
        *EscapeJsonForSettings(CloudPreset)
    );


    SettingsJson +=
        TEXT("  },\n");


    /*
     * Path/follower.
     */
    SettingsJson +=
        TEXT("  \"path\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"path_id\": \"%s\",\n"),
        *EscapeJsonForSettings(PathId)
    );


    SettingsJson += FString::Printf(
        TEXT("    \"cruise_speed_mps\": %.6f,\n"),
        CruiseSpeedMps
    );


    SettingsJson +=
        TEXT("    \"start_behavior\": \"fly_to_spline_start\",\n")
        TEXT("    \"progress_tracking\": \"monotonic_forward_search\",\n")
        TEXT("    \"follower\": {\n");


    SettingsJson += FString::Printf(
        TEXT("      \"look_ahead_distance_m\": %.6f,\n"),
        LookAheadDistanceM
    );


    SettingsJson += FString::Printf(
        TEXT("      \"max_acceleration_mps2\": %.6f,\n"),
        MaxAccelerationMps2
    );


    SettingsJson += FString::Printf(
        TEXT("      \"braking_distance_m\": %.6f,\n"),
        BrakingDistanceM
    );


    SettingsJson += FString::Printf(
        TEXT("      \"arrival_tolerance_m\": %.6f,\n"),
        ArrivalToleranceM
    );


    SettingsJson += FString::Printf(
        TEXT("      \"start_arrival_tolerance_m\": %.6f,\n"),
        StartArrivalToleranceM
    );


    SettingsJson += FString::Printf(
        TEXT("      \"progress_search_step_m\": %.6f,\n"),
        ProgressSearchStepM
    );


    SettingsJson +=
        TEXT("      \"yaw\": {\n");


    SettingsJson += FString::Printf(
        TEXT("        \"mode\": \"%s\",\n"),
        *YawModeString
    );


    SettingsJson += FString::Printf(
        TEXT("        \"gain\": %.6f,\n"),
        PathFollower->YawGain
    );


    SettingsJson += FString::Printf(
        TEXT("        \"max_rate_deg_per_s\": %.6f\n"),
        PathFollower->MaxYawRateDegPerSec
    );


    SettingsJson +=
        TEXT("      },\n")
        TEXT("      \"noise\": {\n");


    SettingsJson += FString::Printf(
        TEXT("        \"enabled\": %s,\n"),
        PathFollower->bEnableNoise
        ? TEXT("true")
        : TEXT("false")
    );


    SettingsJson +=
        TEXT(
            "        \"model\": "
            "\"seeded_smooth_additive_velocity_disturbance\",\n"
        );


    SettingsJson += FString::Printf(
        TEXT("        \"seed\": %d,\n"),
        PathFollower->NoiseSeed
    );


    SettingsJson += FString::Printf(
        TEXT("        \"horizontal_max_mps\": %.6f,\n"),
        PathFollower->HorizontalNoiseMps
    );


    SettingsJson += FString::Printf(
        TEXT("        \"vertical_max_mps\": %.6f,\n"),
        PathFollower->VerticalNoiseMps
    );


    SettingsJson += FString::Printf(
        TEXT("        \"update_interval_s\": %.6f,\n"),
        PathFollower->NoiseUpdateIntervalS
    );


    SettingsJson += FString::Printf(
        TEXT("        \"smoothing_speed\": %.6f\n"),
        PathFollower->NoiseSmoothingSpeed
    );


    SettingsJson +=
        TEXT("      }\n")
        TEXT("    }\n")
        TEXT("  },\n");


    /*
     * North camera.
     */
    SettingsJson +=
        TEXT("  \"north_camera\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"width_px\": %d,\n"),
        NorthCalibration.Width
    );


    SettingsJson += FString::Printf(
        TEXT("    \"height_px\": %d,\n"),
        NorthCalibration.Height
    );


    SettingsJson +=
        TEXT("    \"projection\": \"perspective\",\n");


    SettingsJson += FString::Printf(
        TEXT("    \"horizontal_fov_deg\": %.9f,\n"),
        NorthCalibration.HorizontalFovDeg
    );


    SettingsJson += FString::Printf(
        TEXT("    \"vertical_fov_deg\": %.9f,\n"),
        NorthCalibration.VerticalFovDeg
    );


    SettingsJson +=
        TEXT("    \"intrinsics_px\": {\n");


    SettingsJson += FString::Printf(
        TEXT("      \"fx\": %.9f,\n"),
        NorthCalibration.Fx
    );


    SettingsJson += FString::Printf(
        TEXT("      \"fy\": %.9f,\n"),
        NorthCalibration.Fy
    );


    SettingsJson += FString::Printf(
        TEXT("      \"cx\": %.9f,\n"),
        NorthCalibration.Cx
    );


    SettingsJson += FString::Printf(
        TEXT("      \"cy\": %.9f\n"),
        NorthCalibration.Cy
    );


    SettingsJson +=
        TEXT("    },\n")
        TEXT("    \"distortion\": \"none\",\n")
        TEXT("    \"heading_mode\": \"world_locked_north\"\n")
        TEXT("  },\n");


    /*
     * West camera.
     */
    SettingsJson +=
        TEXT("  \"west_camera\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"width_px\": %d,\n"),
        WestCalibration.Width
    );


    SettingsJson += FString::Printf(
        TEXT("    \"height_px\": %d,\n"),
        WestCalibration.Height
    );


    SettingsJson +=
        TEXT("    \"projection\": \"perspective\",\n");


    SettingsJson += FString::Printf(
        TEXT("    \"horizontal_fov_deg\": %.9f,\n"),
        WestCalibration.HorizontalFovDeg
    );


    SettingsJson += FString::Printf(
        TEXT("    \"vertical_fov_deg\": %.9f,\n"),
        WestCalibration.VerticalFovDeg
    );


    SettingsJson +=
        TEXT("    \"intrinsics_px\": {\n");


    SettingsJson += FString::Printf(
        TEXT("      \"fx\": %.9f,\n"),
        WestCalibration.Fx
    );


    SettingsJson += FString::Printf(
        TEXT("      \"fy\": %.9f,\n"),
        WestCalibration.Fy
    );


    SettingsJson += FString::Printf(
        TEXT("      \"cx\": %.9f,\n"),
        WestCalibration.Cx
    );


    SettingsJson += FString::Printf(
        TEXT("      \"cy\": %.9f\n"),
        WestCalibration.Cy
    );


    SettingsJson +=
        TEXT("    },\n")
        TEXT("    \"distortion\": \"none\",\n")
        TEXT("    \"heading_mode\": \"world_locked_west\"\n")
        TEXT("  },\n");


    /*
     * Nadir camera.
     */
    SettingsJson +=
        TEXT("  \"nadir_camera\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"width_px\": %d,\n"),
        NadirCalibration.Width
    );


    SettingsJson += FString::Printf(
        TEXT("    \"height_px\": %d,\n"),
        NadirCalibration.Height
    );


    SettingsJson +=
        TEXT("    \"projection\": \"perspective\",\n");


    SettingsJson += FString::Printf(
        TEXT("    \"horizontal_fov_deg\": %.9f,\n"),
        NadirCalibration.HorizontalFovDeg
    );


    SettingsJson += FString::Printf(
        TEXT("    \"vertical_fov_deg\": %.9f,\n"),
        NadirCalibration.VerticalFovDeg
    );


    SettingsJson +=
        TEXT("    \"intrinsics_px\": {\n");


    SettingsJson += FString::Printf(
        TEXT("      \"fx\": %.9f,\n"),
        NadirCalibration.Fx
    );


    SettingsJson += FString::Printf(
        TEXT("      \"fy\": %.9f,\n"),
        NadirCalibration.Fy
    );


    SettingsJson += FString::Printf(
        TEXT("      \"cx\": %.9f,\n"),
        NadirCalibration.Cx
    );


    SettingsJson += FString::Printf(
        TEXT("      \"cy\": %.9f\n"),
        NadirCalibration.Cy
    );


    SettingsJson +=
        TEXT("    },\n")

        TEXT("    \"distortion\": \"none\",\n")
        TEXT("    \"principal_point\": \"image_centre\",\n")

        TEXT("    \"pose_definition\": ")
        TEXT("\"SceneCapture world transform; position follows the drone while orientation is world-stabilized\",\n")

        TEXT("    \"orientation_mode\": \"world_stabilized_nadir_independent_yaw\",\n")
        TEXT("    \"absolute_rotation\": true,\n")
        TEXT("    \"position_reference\": \"attached_to_drone_body_hierarchy\",\n")
        TEXT("    \"orientation_reference\": \"UE_world; optical axis stabilized to world down; yaw may vary\",\n")

        TEXT("    \"ue_camera_axes\": {\n")
        TEXT("      \"optical_axis\": \"+X\",\n")
        TEXT("      \"image_right\": \"+Y\",\n")
        TEXT("      \"image_up\": \"+Z\",\n")
        TEXT("      \"image_down\": \"-Z\"\n")
        TEXT("    },\n")

        TEXT("    \"opencv_axis_mapping\": {\n")
        TEXT("      \"x_right\": \"+Y_ue_camera\",\n")
        TEXT("      \"y_down\": \"-Z_ue_camera\",\n")
        TEXT("      \"z_forward\": \"+X_ue_camera\"\n")
        TEXT("    },\n")

        TEXT("    \"heading_definition\": ")
        TEXT("\"projection of image-up axis onto horizontal plane; ")
        TEXT("clockwise from simulator North\",\n")

        /*
         * This relative transform is only the relationship between
         * camera and body at the experiment reference frame.
         *
         * Because camera orientation is world-stabilized, the relative
         * rotation changes whenever the drone body rotates.
         */
        TEXT("    \"camera_relative_to_body_at_reference_frame\": {\n");


    SettingsJson += FString::Printf(
        TEXT(
            "      \"translation_m\": "
            "[%.9f, %.9f, %.9f],\n"
        ),

        CameraRelativeLocationM.X,
        CameraRelativeLocationM.Y,
        CameraRelativeLocationM.Z
    );


    SettingsJson += FString::Printf(
        TEXT(
            "      \"quaternion_wxyz\": "
            "[%.9f, %.9f, %.9f, %.9f]\n"
        ),

        CameraRelativeQuaternion.W,
        CameraRelativeQuaternion.X,
        CameraRelativeQuaternion.Y,
        CameraRelativeQuaternion.Z
    );


    SettingsJson +=
        TEXT("    }\n")
        TEXT("  },\n");


    /*
     * Compatibility block for the VO ingest/evaluation code.
     *
     * These are explicitly the NADIR camera intrinsics.
     */
    SettingsJson +=
        TEXT("  \"camera_intrinsics\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"fx\": %.9f,\n"),
        NadirCalibration.Fx
    );


    SettingsJson += FString::Printf(
        TEXT("    \"fy\": %.9f,\n"),
        NadirCalibration.Fy
    );


    SettingsJson += FString::Printf(
        TEXT("    \"cx\": %.9f,\n"),
        NadirCalibration.Cx
    );


    SettingsJson += FString::Printf(
        TEXT("    \"cy\": %.9f,\n"),
        NadirCalibration.Cy
    );


    SettingsJson +=
        TEXT("    \"k1\": 0.0,\n")
        TEXT("    \"k2\": 0.0,\n")
        TEXT("    \"k3\": 0.0,\n")
        TEXT("    \"p1\": 0.0,\n")
        TEXT("    \"p2\": 0.0,\n");


    SettingsJson += FString::Printf(
        TEXT("    \"width\": %d,\n"),
        NadirCalibration.Width
    );


    SettingsJson += FString::Printf(
        TEXT("    \"height\": %d,\n"),
        NadirCalibration.Height
    );


    SettingsJson +=
        TEXT(
            "    \"source\": "
            "\"UE5 NadirCapture perspective projection\"\n"
        )
        TEXT("  },\n");


    /*
     * Capture configuration.
     */
    SettingsJson +=
        TEXT("  \"capture\": {\n")
        TEXT("    \"fixed_timestep_required\": true,\n");


    SettingsJson += FString::Printf(
        TEXT("    \"simulation_rate_hz\": %.6f,\n"),
        ExpectedSimulationRateHz
    );


    SettingsJson +=
        TEXT("    \"telemetry_policy\": \"every_simulation_tick\",\n");


    SettingsJson += FString::Printf(
        TEXT("    \"telemetry_rate_hz\": %.6f,\n"),
        ExpectedSimulationRateHz
    );


    SettingsJson += FString::Printf(
        TEXT("    \"nadir_automatic_capture\": %s,\n"),
        bAutomaticNadirCapture
        ? TEXT("true")
        : TEXT("false")
    );


    SettingsJson += FString::Printf(
        TEXT("    \"image_decimation_ticks\": %d,\n"),
        NadirCaptureEveryNTicks
    );


    SettingsJson += FString::Printf(
        TEXT("    \"image_rate_hz\": %.9f,\n"),
        ImageRateHz
    );


    SettingsJson +=
        TEXT("    \"timestamp\": "
            "\"sim_time_s; seconds; origin = frame 0 / experiment start\",\n")
        TEXT("    \"pose_instant\": "
            "\"pose, altitude trace and image captured from same callback\",\n")
        TEXT("    \"image_format\": \"PNG_lossless\"\n")
        TEXT("  },\n");


    /*
     * Height semantics.
     */
    SettingsJson +=
        TEXT("  \"height\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"h0_agl_m\": %.9f,\n"),
        InitialAglM
    );


    SettingsJson += FString::Printf(
        TEXT("    \"h0_valid\": %s,\n"),
        bInitialAglValid
        ? TEXT("true")
        : TEXT("false")
    );


    SettingsJson +=
        TEXT("    \"h0_source\": "
            "\"simulator ground truth principal-ray trace at reference frame; "
            "never fitted from trajectory\",\n");


    SettingsJson += FString::Printf(
        TEXT("    \"gsd0_m_per_px\": %.9f,\n"),
        Gsd0MPerPx
    );


    SettingsJson +=
        TEXT("    \"baro_relative_alt_definition\": "
            "\"body Z minus reference-frame body Z; positive up\",\n")
        TEXT("    \"baro_origin\": \"DroneBody / physics-body origin\",\n")
        TEXT("    \"agl_definition\": "
            "\"nadir camera optical centre to first blocking surface "
            "along camera principal ray\",\n")
        TEXT("    \"terrain_elevation_definition\": "
            "\"UE world Z of that same principal-ray surface hit\",\n")
        TEXT("    \"terrain_elevation_datum\": \"UE world Z = 0\",\n")
        TEXT("    \"ground_trace_channel\": \"Visibility\",\n");


    SettingsJson += FString::Printf(
        TEXT("    \"ground_trace_max_distance_m\": %.3f\n"),
        GroundTraceDistanceCm / 100.0f
    );


    SettingsJson +=
        TEXT("  },\n");


    /*
     * Rendering assumptions for the VO baseline.
     *
     * These must be configured in the project/SceneCapture as
     * described below.
     */
    SettingsJson +=
        TEXT("  \"vo_rendering_baseline\": {\n")
        TEXT("    \"motion_blur\": \"off\",\n")
        TEXT("    \"auto_exposure\": \"fixed\",\n")
        TEXT("    \"temporal_aa_tsr\": \"off\",\n")
        TEXT("    \"lens_distortion\": \"none\",\n")
        TEXT("    \"bloom\": \"off\",\n")
        TEXT("    \"lens_flare\": \"off\",\n")
        TEXT("    \"chromatic_aberration\": \"off\",\n")
        TEXT("    \"depth_of_field\": \"off\"\n")
        TEXT("  },\n");


    /*
     * Skyline.
     */
    SettingsJson +=
        TEXT("  \"skyline\": {\n");


    SettingsJson += FString::Printf(
        TEXT("    \"automatic_capture\": %s,\n"),
        bAutomaticSkylineCapture
        ? TEXT("true")
        : TEXT("false")
    );


    SettingsJson += FString::Printf(
        TEXT("    \"capture_distance_m\": %.6f,\n"),
        SkylineCaptureDistanceM
    );


    SettingsJson +=
        TEXT("    \"views\": [\"north\", \"west\"],\n")
        TEXT("    \"request_api\": \"QueueSkylineCapture\",\n")
        TEXT("    \"capture_pairing\": \"north and west captured in one skyline callback\",\n")
        TEXT("    \"vo_sync_policy\": \"when automatic nadir capture is enabled, one pending skyline request is consumed by the next successful nadir/VO frame; otherwise skyline capture is immediate\",\n")
        TEXT("    \"vo_frame_id_semantics\": \"exact associated vo/frames.csv frame_id; -1 means no synchronized VO frame\",\n")
        TEXT("    \"north\": {\n")
        TEXT("      \"rgb_component_tag\": \"NorthCapture\",\n")
        TEXT("      \"depth_component_tag\": \"NorthDepthCapture\",\n")
        TEXT("      \"rgb_path_pattern\": \"skyline/images/sky_XXXXXX.png\",\n")
        TEXT("      \"sky_mask_path_pattern\": \"skyline/sim/sky_XXXXXX_sky.png\"\n")
        TEXT("    },\n")
        TEXT("    \"west\": {\n")
        TEXT("      \"rgb_component_tag\": \"WestCapture\",\n")
        TEXT("      \"depth_component_tag\": \"WestDepthCapture\",\n")
        TEXT("      \"rgb_path_pattern\": \"skyline/images/sky_XXXXXX_west.png\",\n")
        TEXT("      \"sky_mask_path_pattern\": \"skyline/sim/sky_XXXXXX_west_sky.png\"\n")
        TEXT("    },\n")
        TEXT("    \"sky_mask\": {\n")
        TEXT("      \"depth_channel\": \"R\",\n")
        TEXT("      \"depth_units\": \"cm\",\n");


    SettingsJson += FString::Printf(
        TEXT("      \"sky_threshold_cm\": %.3f,\n"),
        SkylineSkyDepthThresholdCm
    );


    SettingsJson +=
        TEXT("      \"classification\": \"depth >= threshold => sky\"\n")
        TEXT("    }\n")
        TEXT("  }\n")
        TEXT("}\n");


    const FString SettingsPath =
        FPaths::Combine(
            CurrentRunDirectory,
            TEXT("settings.json")
        );


    if (
        !FFileHelper::SaveStringToFile(
            SettingsJson,
            *SettingsPath
        )
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to create settings.json"
            )
        );

        return;
    }


    /*
     * ------------------------------------------------------------
     * Initialize run state.
     * ------------------------------------------------------------
     */

    SkylineObservationCounter = 0;

    bSkylineCapturePending = false;

    VoFrameCounter = 0;

    GroundTruthSampleCounter = 0;

    TicksSinceLastNadirCapture = 0;

    SimulationTickIndex = 0;


    TickValidationCount = 0;

    MaxTickDeltaErrorS = 0.0;

    bTickRateValidationDone = false;


    bExperimentRunning = true;


    /*
     * Frame 0 and GT sample 0 are captured immediately.
     *
     * This makes:
     *
     *     sim_time_s = 0
     *
     * the reference frame for:
     *
     *     h0
     *     baro_relative_alt
     *     VO frame_id 0
     */
    WriteGroundTruthSample();

    CaptureNadirObservation();


    UE_LOG(
        LogTemp,
        Warning,
        TEXT(
            "Experiment started: %s"
        ),
        *CurrentRunDirectory
    );


    UE_LOG(
        LogTemp,
        Warning,
        TEXT(
            "VO: telemetry %.3f Hz, "
            "nadir images %.3f Hz, "
            "h0 = %.3f m"
        ),
        ExpectedSimulationRateHz,
        ImageRateHz,
        InitialAglM
    );
}


void UExperimentLoggerComponent::CaptureNadirObservation()
{
    if (!bExperimentRunning)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Cannot capture nadir observation: "
                "no experiment is running"
            )
        );

        return;
    }


    if (
        !NadirCapture ||
        !NadirCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot capture nadir observation: "
                "NadirCapture is not configured"
            )
        );

        return;
    }


    /*
     * Collect telemetry at this exact capture callback.
     */
    FExperimentVoSample Sample;


    if (!CollectVoSample(Sample))
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to collect nadir VO sample"
            )
        );

        return;
    }


    const int32 FrameId =
        VoFrameCounter;


    const FString ImageFileName =
        FString::Printf(
            TEXT("frame_%06d.png"),
            FrameId
        );


    /*
     * Render from the current camera transform.
     */
    NadirCapture->CaptureScene();


    /*
     * Lossless image file.
     */
    UKismetRenderingLibrary::ExportRenderTarget(
        this,
        NadirCapture->TextureTarget,
        VoImagesDirectory,
        ImageFileName
    );


    const FString RelativeImagePath =
        FPaths::Combine(
            TEXT("vo"),
            TEXT("images"),
            ImageFileName
        );


    const FString AglString =
        Sample.bGroundHit
        ? FString::Printf(
            TEXT("%.6f"),
            Sample.TrueAglM
        )
        : TEXT("nan");


    const FString TerrainString =
        Sample.bGroundHit
        ? FString::Printf(
            TEXT("%.6f"),
            Sample.TerrainElevationM
        )
        : TEXT("nan");


    const FString CsvRow =
        FString::Printf(

            TEXT("%d,%.9f,%s,")

            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.9f,%.9f,%.9f,%.9f,")

            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.9f,%.9f,%.9f,%.9f,")

            TEXT("%.9f,%.9f,%.9f,")

            TEXT("%.6f,%.6f,")

            TEXT("%.6f,%s,%s,%d\n"),


            FrameId,
            Sample.SimTimeS,
            *RelativeImagePath,


            Sample.CameraLocationCm.X,
            Sample.CameraLocationCm.Y,
            Sample.CameraLocationCm.Z,

            Sample.CameraQuaternion.W,
            Sample.CameraQuaternion.X,
            Sample.CameraQuaternion.Y,
            Sample.CameraQuaternion.Z,


            Sample.BodyLocationCm.X,
            Sample.BodyLocationCm.Y,
            Sample.BodyLocationCm.Z,

            Sample.BodyQuaternion.W,
            Sample.BodyQuaternion.X,
            Sample.BodyQuaternion.Y,
            Sample.BodyQuaternion.Z,


            Sample.EastM,
            Sample.NorthM,
            Sample.UpM,


            Sample.HeadingDeg,
            Sample.CameraTiltDeg,


            Sample.BaroRelativeAltM,

            *AglString,
            *TerrainString,

            Sample.bGroundHit ? 1 : 0
        );


    const bool bWritten =
        FFileHelper::SaveStringToFile(
            CsvRow,
            *VoFramesCsvPath,
            FFileHelper::EEncodingOptions::AutoDetect,
            &IFileManager::Get(),
            FILEWRITE_Append
        );


    if (!bWritten)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to append VO frame %d"
            ),
            FrameId
        );

        return;
    }


    VoFrameCounter++;


    UE_LOG(
        LogTemp,
        Verbose,
        TEXT(
            "Captured nadir VO frame %d"
        ),
        FrameId
    );


    /*
     * If a skyline request is pending, consume it from this exact
     * successful VO frame callback. The simulation cannot advance
     * until this game-thread callback returns, so the skyline views
     * and the nadir image share the same resolved simulation state.
     */
    if (bSkylineCapturePending)
    {
        const bool bSkylineCaptured =
            CaptureSkylineObservation(
                FrameId,
                Sample.SimTimeS
            );


        if (bSkylineCaptured)
        {
            bSkylineCapturePending = false;
        }
        else
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT(
                    "Pending skyline capture failed on VO frame %d; "
                    "request remains queued for the next VO frame"
                ),
                FrameId
            );
        }
    }
}


void UExperimentLoggerComponent::QueueSkylineCapture()
{
    if (!bExperimentRunning)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Cannot queue skyline capture: "
                "no experiment is running"
            )
        );

        return;
    }


    if (bAutomaticNadirCapture)
    {
        if (bSkylineCapturePending)
        {
            UE_LOG(
                LogTemp,
                Verbose,
                TEXT(
                    "Skyline capture request already pending; "
                    "coalescing duplicate request"
                )
            );

            return;
        }


        bSkylineCapturePending = true;


        UE_LOG(
            LogTemp,
            Verbose,
            TEXT(
                "Queued skyline capture for the next "
                "nadir/VO frame"
            )
        );

        return;
    }


    /*
     * No automatic nadir stream is active, so there is no future
     * frame to synchronize against. Capture immediately and mark
     * the VO association explicitly as unavailable.
     */
    if (
        !CaptureSkylineObservation(
            INDEX_NONE,
            GetExperimentSimTimeSeconds()
        )
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Immediate skyline capture failed"
            )
        );
    }
}


bool UExperimentLoggerComponent::CaptureSkylineObservation(
    int32 AssociatedVoFrameId,
    double CaptureSimTimeS)
{
    if (!bExperimentRunning)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Cannot capture skyline: "
                "no experiment is running"
            )
        );

        return false;
    }


    if (
        !NorthCapture ||
        !NorthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot capture skyline: "
                "NorthCapture is not configured"
            )
        );

        return false;
    }


    if (
        !WestCapture ||
        !WestCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot capture skyline: "
                "WestCapture is not configured"
            )
        );

        return false;
    }


    if (
        !NorthDepthCapture ||
        !NorthDepthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot capture skyline: "
                "NorthDepthCapture is not configured"
            )
        );

        return false;
    }


    if (
        !WestDepthCapture ||
        !WestDepthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot capture skyline: "
                "WestDepthCapture is not configured"
            )
        );

        return false;
    }


    if (
        !NadirCapture ||
        !NadirCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot capture skyline: "
                "NadirCapture is not configured"
            )
        );

        return false;
    }


    const int32 ObservationIndex =
        SkylineObservationCounter + 1;


    const FString ObservationId =
        FString::Printf(
            TEXT("sky_%06d"),
            ObservationIndex
        );


    /*
     * Keep the original north filenames unchanged for backwards
     * compatibility. West files are added alongside them.
     */
    const FString NorthImageFileName =
        ObservationId +
        TEXT(".png");


    const FString NorthSkyMaskFileName =
        ObservationId +
        TEXT("_sky.png");


    const FString WestImageFileName =
        ObservationId +
        TEXT("_west.png");


    const FString WestSkyMaskFileName =
        ObservationId +
        TEXT("_west_sky.png");


    const FString NorthSkyMaskFullPath =
        FPaths::Combine(
            SkylineSimDirectory,
            NorthSkyMaskFileName
        );


    const FString WestSkyMaskFullPath =
        FPaths::Combine(
            SkylineSimDirectory,
            WestSkyMaskFileName
        );


    /*
     * Read all poses before rendering either view so the CSV row
     * represents one logical skyline observation instant.
     */
    const FVector NorthLocation =
        NorthCapture->GetComponentLocation();


    const FRotator NorthRotation =
        NorthCapture->GetComponentRotation();


    const FQuat NorthQuaternion =
        NorthCapture
        ->GetComponentQuat()
        .GetNormalized();


    const FVector WestLocation =
        WestCapture->GetComponentLocation();


    const FRotator WestRotation =
        WestCapture->GetComponentRotation();


    const FQuat WestQuaternion =
        WestCapture
        ->GetComponentQuat()
        .GetNormalized();


    const FVector NadirLocation =
        NadirCapture->GetComponentLocation();


    const FRotator NadirRotation =
        NadirCapture->GetComponentRotation();


    const FQuat NadirQuaternion =
        NadirCapture
        ->GetComponentQuat()
        .GetNormalized();


    /*
     * North RGB + depth-derived sky mask.
     */
    NorthCapture->CaptureScene();


    UKismetRenderingLibrary::ExportRenderTarget(
        this,
        NorthCapture->TextureTarget,
        SkylineImagesDirectory,
        NorthImageFileName
    );


    const bool bNorthMaskSaved =
        SaveSkyMask(
            NorthDepthCapture,
            TEXT("north"),
            NorthSkyMaskFullPath
        );


    if (!bNorthMaskSaved)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to save north simulator sky mask for %s"
            ),
            *ObservationId
        );

        return false;
    }


    /*
     * West RGB + depth-derived sky mask.
     */
    WestCapture->CaptureScene();


    UKismetRenderingLibrary::ExportRenderTarget(
        this,
        WestCapture->TextureTarget,
        SkylineImagesDirectory,
        WestImageFileName
    );


    const bool bWestMaskSaved =
        SaveSkyMask(
            WestDepthCapture,
            TEXT("west"),
            WestSkyMaskFullPath
        );


    if (!bWestMaskSaved)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to save west simulator sky mask for %s"
            ),
            *ObservationId
        );

        return false;
    }


    const FString RelativeNorthImagePath =
        FPaths::Combine(
            TEXT("skyline"),
            TEXT("images"),
            NorthImageFileName
        );


    const FString RelativeNorthSkyMaskPath =
        FPaths::Combine(
            TEXT("skyline"),
            TEXT("sim"),
            NorthSkyMaskFileName
        );


    const FString RelativeWestImagePath =
        FPaths::Combine(
            TEXT("skyline"),
            TEXT("images"),
            WestImageFileName
        );


    const FString RelativeWestSkyMaskPath =
        FPaths::Combine(
            TEXT("skyline"),
            TEXT("sim"),
            WestSkyMaskFileName
        );


    const FString CsvRow =
        FString::Printf(

            TEXT("%s,%.9f,")

            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.9f,%.9f,%.9f,%.9f,")

            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.9f,%.9f,%.9f,%.9f,")

            TEXT("%s,%s,")

            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.6f,%.6f,%.6f,")
            TEXT("%.9f,%.9f,%.9f,%.9f,")

            TEXT("%s,%s,")
            TEXT("%d,%d\n"),


            *ObservationId,
            CaptureSimTimeS,


            NorthLocation.X,
            NorthLocation.Y,
            NorthLocation.Z,

            NorthRotation.Yaw,
            NorthRotation.Pitch,
            NorthRotation.Roll,

            NorthQuaternion.W,
            NorthQuaternion.X,
            NorthQuaternion.Y,
            NorthQuaternion.Z,


            NadirLocation.X,
            NadirLocation.Y,
            NadirLocation.Z,

            NadirRotation.Yaw,
            NadirRotation.Pitch,
            NadirRotation.Roll,

            NadirQuaternion.W,
            NadirQuaternion.X,
            NadirQuaternion.Y,
            NadirQuaternion.Z,


            *RelativeNorthImagePath,
            *RelativeNorthSkyMaskPath,


            WestLocation.X,
            WestLocation.Y,
            WestLocation.Z,

            WestRotation.Yaw,
            WestRotation.Pitch,
            WestRotation.Roll,

            WestQuaternion.W,
            WestQuaternion.X,
            WestQuaternion.Y,
            WestQuaternion.Z,


            *RelativeWestImagePath,
            *RelativeWestSkyMaskPath,

            AssociatedVoFrameId,
            AssociatedVoFrameId != INDEX_NONE ? 1 : 0
        );


    const bool bWritten =
        FFileHelper::SaveStringToFile(
            CsvRow,
            *SkylineCsvPath,
            FFileHelper::EEncodingOptions::AutoDetect,
            &IFileManager::Get(),
            FILEWRITE_Append
        );


    if (!bWritten)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to append skyline observation"
            )
        );

        return false;
    }


    SkylineObservationCounter =
        ObservationIndex;


    if (AssociatedVoFrameId != INDEX_NONE)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Captured skyline observation %s (north + west), "
                "synchronized with VO frame %d"
            ),
            *ObservationId,
            AssociatedVoFrameId
        );
    }
    else
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Captured skyline observation %s (north + west), "
                "without synchronized VO frame"
            ),
            *ObservationId
        );
    }


    return true;
}


void UExperimentLoggerComponent::StopExperiment()
{
    if (!bExperimentRunning)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("No experiment is running")
        );

        return;
    }


    if (bSkylineCapturePending)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT(
                "Experiment stopped with one pending skyline "
                "capture request; request discarded"
            )
        );

        bSkylineCapturePending = false;
    }


    bExperimentRunning = false;


    UE_LOG(
        LogTemp,
        Warning,
        TEXT(
            "Experiment stopped: %s"
        ),
        *CurrentRunId
    );


    UE_LOG(
        LogTemp,
        Warning,
        TEXT(
            "VO frames: %d, GT samples: %d, skyline observations: %d"
        ),
        VoFrameCounter,
        GroundTruthSampleCounter,
        SkylineObservationCounter
    );
}


void UExperimentLoggerComponent::TestNorthDepthValues()
{
    if (
        !NorthDepthCapture ||
        !NorthDepthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "North depth capture is not configured"
            )
        );

        return;
    }


    NorthDepthCapture->CaptureScene();


    UTextureRenderTarget2D* RenderTarget =
        NorthDepthCapture->TextureTarget;


    FTextureRenderTargetResource* Resource =
        RenderTarget
        ->GameThread_GetRenderTargetResource();


    if (!Resource)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Could not get depth render target resource"
            )
        );

        return;
    }


    TArray<FLinearColor> Pixels;


    FReadSurfaceDataFlags ReadFlags(
        RCM_MinMax
    );


    ReadFlags.SetLinearToGamma(false);


    const bool bSuccess =
        Resource->ReadLinearColorPixels(
            Pixels,
            ReadFlags,
            FIntRect()
        );


    if (!bSuccess)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to read depth render target"
            )
        );

        return;
    }


    const int32 Width =
        RenderTarget->SizeX;


    const int32 Height =
        RenderTarget->SizeY;


    auto PrintPixel =
        [&](int32 X, int32 Y, const TCHAR* Label)
        {
            const int32 Index =
                Y * Width + X;


            if (!Pixels.IsValidIndex(Index))
            {
                return;
            }


            UE_LOG(
                LogTemp,
                Warning,
                TEXT(
                    "%s depth R = %.9f"
                ),
                Label,
                Pixels[Index].R
            );
        };


    PrintPixel(
        Width / 2,
        Height / 8,
        TEXT("TOP")
    );


    PrintPixel(
        Width / 2,
        Height / 2,
        TEXT("MIDDLE")
    );


    PrintPixel(
        Width / 2,
        Height * 7 / 8,
        TEXT("BOTTOM")
    );
}


bool UExperimentLoggerComponent::SaveSkyMask(
    USceneCaptureComponent2D* DepthCapture,
    const TCHAR* ViewName,
    const FString& OutputPath)
{
    if (
        !DepthCapture ||
        !DepthCapture->TextureTarget
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot save sky mask: "
                "depth capture is not configured"
            )
        );

        return false;
    }


    UTextureRenderTarget2D* RenderTarget =
        DepthCapture->TextureTarget;


    DepthCapture->CaptureScene();


    FTextureRenderTargetResource* Resource =
        RenderTarget
        ->GameThread_GetRenderTargetResource();


    if (!Resource)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot save sky mask: "
                "no render target resource"
            )
        );

        return false;
    }


    TArray<FLinearColor> DepthPixels;


    FReadSurfaceDataFlags ReadFlags(
        RCM_MinMax
    );


    ReadFlags.SetLinearToGamma(false);


    const bool bReadSuccess =
        Resource->ReadLinearColorPixels(
            DepthPixels,
            ReadFlags,
            FIntRect()
        );


    if (!bReadSuccess)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot save sky mask: "
                "failed to read depth pixels"
            )
        );

        return false;
    }


    const int32 Width =
        RenderTarget->SizeX;


    const int32 Height =
        RenderTarget->SizeY;


    if (
        DepthPixels.Num() !=
        Width * Height
        )
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Cannot save sky mask: "
                "expected %d pixels, got %d"
            ),
            Width * Height,
            DepthPixels.Num()
        );

        return false;
    }


    TArray<FColor> MaskPixels;

    MaskPixels.SetNumUninitialized(
        DepthPixels.Num()
    );


    int32 SkyPixelCount = 0;


    for (
        int32 Index = 0;
        Index < DepthPixels.Num();
        ++Index
        )
    {
        const float DepthCm =
            DepthPixels[Index].R;


        const bool bIsSky =
            DepthCm >=
            SkylineSkyDepthThresholdCm;


        MaskPixels[Index] =
            bIsSky
            ? FColor::White
            : FColor::Black;


        if (bIsSky)
        {
            SkyPixelCount++;
        }
    }


    const double SkyFraction =
        static_cast<double>(
            SkyPixelCount
            ) /
        static_cast<double>(
            DepthPixels.Num()
            );


    UE_LOG(
        LogTemp,
        Warning,
        TEXT(
            "%s sky mask: %.2f%% sky"
        ),
        ViewName,
        SkyFraction * 100.0
    );


    const FImageView MaskImage(
        MaskPixels.GetData(),
        Width,
        Height,
        EGammaSpace::Linear
    );


    const bool bSaved =
        FImageUtils::SaveImageByExtension(
            *OutputPath,
            MaskImage,
            100
        );


    if (!bSaved)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "Failed to save sky mask: %s"
            ),
            *OutputPath
        );

        return false;
    }


    return true;
}