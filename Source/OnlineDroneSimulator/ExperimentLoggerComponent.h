#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneCaptureComponent2D.h"

#include "ExperimentLoggerComponent.generated.h"


class UDronePathFollowerComponent;
class UPrimitiveComponent;


/*
 * Computed pinhole calibration for one SceneCaptureComponent2D.
 */
struct FExperimentCameraCalibration
{
    int32 Width = 0;
    int32 Height = 0;

    double HorizontalFovDeg = 0.0;
    double VerticalFovDeg = 0.0;

    double Fx = 0.0;
    double Fy = 0.0;
    double Cx = 0.0;
    double Cy = 0.0;
};


/*
 * One instantaneous VO/ground-truth sample.
 *
 * Camera and body poses are stored in raw UE world coordinates.
 * ENU position is derived from the nadir camera centre.
 */
struct FExperimentVoSample
{
    double SimTimeS = 0.0;

    FVector CameraLocationCm =
        FVector::ZeroVector;

    FQuat CameraQuaternion =
        FQuat::Identity;

    FVector BodyLocationCm =
        FVector::ZeroVector;

    FQuat BodyQuaternion =
        FQuat::Identity;

    double EastM = 0.0;
    double NorthM = 0.0;
    double UpM = 0.0;

    double HeadingDeg = 0.0;
    double CameraTiltDeg = 0.0;

    double BaroRelativeAltM = 0.0;

    bool bGroundHit = false;

    double TrueAglM = 0.0;
    double TerrainElevationM = 0.0;
};


UCLASS(
    ClassGroup = (Custom),
    meta = (BlueprintSpawnableComponent)
)
class ONLINEDRONESIMULATOR_API UExperimentLoggerComponent
    : public UActorComponent
{
    GENERATED_BODY()

public:

    UExperimentLoggerComponent();


protected:

    virtual void BeginPlay() override;


public:

    virtual void TickComponent(
        float DeltaTime,
        ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction
    ) override;


    UFUNCTION(BlueprintCallable, Category = "Experiment Logging")
    void StartExperiment(
        const FString& TimeOfDay,
        float Hour,
        float TimeSpeed,
        const FString& CloudPreset,
        bool bAutomaticSkylineCapture,
        float SkylineCaptureDistanceCm,
        const FString& PathId,
        float CruiseSpeedMps
    );


    /*
     * Requests one skyline observation.
     *
     * If automatic nadir capture is enabled, the request is held
     * until the next nadir/VO frame so the skyline observation can
     * record that exact VO frame_id. Only one request is kept pending.
     *
     * If automatic nadir capture is disabled, the skyline is captured
     * immediately and its vo_frame_id is written as -1.
     */
    UFUNCTION(BlueprintCallable, Category = "Experiment Logging")
    void QueueSkylineCapture();


    /*
     * Captures one VO image and writes one row to vo/frames.csv.
     *
     * This remains Blueprint-callable so the capture policy can
     * easily be changed later.
     */
    UFUNCTION(BlueprintCallable, Category = "Experiment Logging")
    void CaptureNadirObservation();


    UFUNCTION(BlueprintCallable, Category = "Experiment Logging")
    void StopExperiment();


    UFUNCTION(BlueprintCallable, Category = "Experiment Logging")
    void TestNorthDepthValues();


    /*
     * ------------------------------------------------------------
     * VO capture configuration.
     * ------------------------------------------------------------
     */


     /*
      * When enabled, the logger automatically calls
      * CaptureNadirObservation() every N simulation ticks.
      *
      * Ground truth is still written every tick regardless.
      */
    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Experiment Logging|VO"
    )
    bool bAutomaticNadirCapture = true;


    /*
     * We require the UE simulation to run at this fixed rate.
     *
     * Configure the same value in Project Settings.
     */
    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Experiment Logging|VO",
        meta = (ClampMin = "1.0")
    )
    float ExpectedSimulationRateHz = 60.0f;


    /*
     * At 60 Hz:
     *
     * 6 ticks -> 10 Hz images
     * 4 ticks -> 15 Hz
     * 12 ticks -> 5 Hz
     */
    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Experiment Logging|VO",
        meta = (ClampMin = "1")
    )
    int32 NadirCaptureEveryNTicks = 6;


    /*
     * Maximum principal-ray terrain trace distance.
     *
     * 10,000,000 cm = 100 km.
     */
    UPROPERTY(
        EditAnywhere,
        BlueprintReadWrite,
        Category = "Experiment Logging|VO",
        meta = (ClampMin = "100.0")
    )
    float GroundTraceDistanceCm = 10000000.0f;


private:

    bool bExperimentRunning = false;

    /*
     * Sticky single skyline request waiting for the next
     * nadir/VO capture. Repeated queue calls are coalesced.
     */
    bool bSkylineCapturePending = false;


    /*
     * ------------------------------------------------------------
     * Counters.
     * ------------------------------------------------------------
     */

    int32 SkylineObservationCounter = 0;

    /*
     * VO image frame counter.
     *
     * 0-based and contiguous.
     */
    int32 VoFrameCounter = 0;

    /*
     * Full-rate telemetry sample counter.
     */
    int32 GroundTruthSampleCounter = 0;

    int32 TicksSinceLastNadirCapture = 0;


    int64 SimulationTickIndex = 0;


    /*
     * ------------------------------------------------------------
     * Run timing/reference state.
     * ------------------------------------------------------------
     */

    double ExperimentStartWorldTimeS = 0.0;

    /*
     * Body altitude at frame 0.
     *
     * baro_relative_alt_m is measured relative to this value.
     */
    double InitialBodyZCm = 0.0;

    /*
     * Principal-ray AGL at the reference frame.
     */
    double InitialAglM = 0.0;


    /*
     * Small runtime sanity check for the requested fixed timestep.
     */
    int32 TickValidationCount = 0;
    double MaxTickDeltaErrorS = 0.0;
    bool bTickRateValidationDone = false;


    /*
     * ------------------------------------------------------------
     * Run paths.
     * ------------------------------------------------------------
     */

    FString CurrentRunId;
    FString CurrentRunDirectory;


    FString SkylineDirectory;
    FString SkylineImagesDirectory;
    FString SkylineSimDirectory;
    FString SkylineCsvPath;


    FString VoDirectory;
    FString VoImagesDirectory;
    FString VoFramesCsvPath;
    FString VoGroundTruthCsvPath;


    /*
     * ------------------------------------------------------------
     * Components.
     * ------------------------------------------------------------
     */

    UPROPERTY()
    TObjectPtr<USceneCaptureComponent2D> NorthCapture = nullptr;


    UPROPERTY()
    TObjectPtr<USceneCaptureComponent2D> WestCapture = nullptr;


    UPROPERTY()
    TObjectPtr<USceneCaptureComponent2D> NadirCapture = nullptr;


    UPROPERTY()
    TObjectPtr<USceneCaptureComponent2D> NorthDepthCapture = nullptr;


    UPROPERTY()
    TObjectPtr<USceneCaptureComponent2D> WestDepthCapture = nullptr;


    /*
     * Physical drone body / FC reference origin.
     *
     * Expected Component Tag:
     *
     *     DroneBody
     */
    UPROPERTY()
    TObjectPtr<UPrimitiveComponent> BodyComponent = nullptr;


    UPROPERTY()
    TObjectPtr<UDronePathFollowerComponent> PathFollower = nullptr;


    /*
     * ------------------------------------------------------------
     * VO helpers.
     * ------------------------------------------------------------
     */

    double GetExperimentSimTimeSeconds() const;


    bool ComputeCameraCalibration(
        USceneCaptureComponent2D* Capture,
        const TCHAR* CameraName,
        FExperimentCameraCalibration& OutCalibration
    ) const;


    bool TraceNadirSurface(
        double& OutAglM,
        double& OutTerrainElevationM
    ) const;


    bool CollectVoSample(
        FExperimentVoSample& OutSample
    ) const;


    double ComputeNadirHeadingDeg() const;

    double ComputeNadirTiltDeg() const;


    void WriteGroundTruthSample();


    /*
     * ------------------------------------------------------------
     * Skyline helpers.
     * ------------------------------------------------------------
     */

    bool CaptureSkylineObservation(
        int32 AssociatedVoFrameId,
        double CaptureSimTimeS
    );


    bool SaveSkyMask(
        USceneCaptureComponent2D* DepthCapture,
        const TCHAR* ViewName,
        const FString& OutputPath
    );
};









