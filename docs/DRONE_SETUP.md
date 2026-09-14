# Rebuilding the drone pawn

The drone Blueprints used for the thesis are derivative works of a licensed
Fab asset pack and are therefore **not redistributed** (see
[../THIRD_PARTY_ASSETS.md](../THIRD_PARTY_ASSETS.md)). This page documents
everything `UExperimentLoggerComponent` and `UDronePathFollowerComponent`
require, so the pawn can be rebuilt on top of your own copy of the pack.

Everything below is taken directly from
`Source/OnlineDroneSimulator/ExperimentLoggerComponent.cpp`.

---

## How the logger finds things

`UExperimentLoggerComponent::BeginPlay` does **no** hard-coded component
lookups by pointer. It searches its owning actor by **component tag**. Add the
components below to your drone pawn and set the tags exactly as written —
tags are case-sensitive.

### Scene capture components

Add five `SceneCaptureComponent2D` components to the pawn. Each needs a
`TextureTarget` render target assigned; matching render targets are shipped in
`Content/RenderTargets/`.

| Component Tag | Purpose | Suggested render target |
|---|---|---|
| `NadirCapture` | Downward-facing VO camera. Source of `vo/images/` and of the ENU position written to the CSVs. | `Down_TextureRenderTarget2D` |
| `NorthCapture` | North-facing skyline camera. | `North_TextureRenderTarget2D` |
| `WestCapture` | West-facing skyline camera. | `West_TextureRenderTarget2D` |
| `NorthDepthCapture` | Depth view co-located with `NorthCapture`, used to derive the north sky mask. | `RT_NorthDepth` |
| `WestDepthCapture` | Depth view co-located with `WestCapture`, used to derive the west sky mask. | `RT_WestDepth` |

**Depth captures.** `SaveSkyMask` reads the render target as
`FLinearColor` and treats the **red channel as a scene depth in centimetres**.
A pixel is classified as sky when

```
depth_cm >= 99000000.0
```

(`SkylineSkyDepthThresholdCm`, `ExperimentLoggerComponent.cpp:32`). The two
depth captures must therefore be configured to write linear scene depth in cm
into R — not a colour image — and must share the transform and FOV of their
corresponding RGB capture, or the mask will not align with the image.

**Self-occlusion.** The logger calls `HideActorComponents(Owner, true)` on all
five captures at `BeginPlay`, so the UAV (including child actors such as
propellers) is hidden from the navigation cameras automatically. The drone
still renders normally in the game and editor views, and collision is
unaffected. You do not need to configure this yourself.

### Drone body

| Component Tag | Purpose |
|---|---|
| `DroneBody` | The physical body / flight-controller reference origin. Must be a `UPrimitiveComponent`. Supplies `body_ue_*` pose columns and the `baro_relative_alt_m` reference. |

> There is a legacy fallback that matches a component literally named
> `BodyMesh_1` and logs a warning. Do not rely on it — set the `DroneBody`
> tag.

### Path following

Add a `UDronePathFollowerComponent` to the same actor. The logger finds it via
`FindComponentByClass` — no tag needed. It must be initialised from Blueprint
with `Initialize(InPhysicsBody)`, passing the primitive component that the
physics simulation drives.

---

## Logger settings

Exposed on `UExperimentLoggerComponent` (category *Experiment Logging | VO*):

| Property | Default | Meaning |
|---|---|---|
| `bAutomaticNadirCapture` | `true` | Capture a VO frame every N ticks automatically. Ground truth is written every tick regardless of this setting. |
| `ExpectedSimulationRateHz` | `60.0` | Must match the project's fixed frame rate. The component validates the actual tick delta at runtime and warns on drift. |
| `NadirCaptureEveryNTicks` | `6` | At 60 Hz: 6 → 10 Hz imagery, 4 → 15 Hz, 12 → 5 Hz. |
| `GroundTraceDistanceCm` | `10000000.0` | Max principal-ray trace distance for AGL / terrain elevation (100 km). |

`Config/DefaultEngine.ini` already sets `bUseFixedFrameRate=True` and
`FixedFrameRate=60.000000`. If you change one, change the other.

---

## Blueprint entry points

All are `BlueprintCallable` on `UExperimentLoggerComponent`:

| Function | Notes |
|---|---|
| `StartExperiment(TimeOfDay, Hour, TimeSpeed, CloudPreset, bAutomaticSkylineCapture, SkylineCaptureDistanceCm, PathId, CruiseSpeedMps)` | Creates the run directory, writes CSV headers and `settings.json`. The environment arguments are recorded into the manifest; pass the values actually applied by `BP_EnvironmentManager`. |
| `QueueSkylineCapture()` | Requests one skyline observation. With automatic nadir capture enabled the request is held until the next VO frame so the observation can record that exact `vo_frame_id`; otherwise it fires immediately and writes `vo_frame_id = -1`. Repeated calls coalesce into a single pending request. |
| `CaptureNadirObservation()` | Captures one VO image and appends one row to `vo/frames.csv`. Called automatically when `bAutomaticNadirCapture` is set; remains callable so the capture policy can be changed. |
| `StopExperiment()` | Finalises the run. |
| `TestNorthDepthValues()` | Diagnostic helper for verifying the depth capture configuration. |

On `UDronePathFollowerComponent`: `Initialize`, `StartFollowingPath(PathActor, CruiseSpeedMps)`,
`StopFollowingPath`, `ReleaseYawHold`, plus the pure getters `GetIsFollowing`,
`GetActivePath`, `ShouldBlockManualYaw`. The `OnDronePathFinished` delegate
fires when a path completes.
