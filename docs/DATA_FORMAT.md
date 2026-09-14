# Recorded run format

Everything below is read from
`Source/OnlineDroneSimulator/ExperimentLoggerComponent.cpp`. No recorded data
is committed to this repository — see *Research reproducibility* in the README.

## Directory layout

`StartExperiment(...)` creates one directory per run under the project's
`Saved/` directory:

```
Saved/SimulatorRuns/Run_<YYYYmmdd_HHMMSS>/
├── settings.json           run manifest (conventions + configuration)
├── vo/
│   ├── frames.csv          one row per captured nadir image
│   ├── groundtruth.csv     one row per simulation tick (no image)
│   └── images/             nadir VO imagery
└── skyline/
    ├── observations.csv    one row per skyline observation
    ├── images/             north and west skyline imagery
    └── sim/                derived binary sky masks
```

`run_id` is `Run_` followed by the local wall-clock timestamp at
`StartExperiment`.

## Coordinate and rotation conventions

Recorded verbatim into `settings.json`, and worth stating explicitly because
the CSVs mix raw engine coordinates with a derived ENU frame:

* **World frame** — `local_non_georeferenced`. There is no georeference;
  `georeference` is `null` and the vertical datum is UE world `Z = 0`.
* **Engine units** — centimetres, **left**-handed. North is `+X`, east is
  `+Y`, up is `+Z`.
* **ENU frame** — right-handed, derived as
  `east_m = ue_y_cm / 100`, `north_m = ue_x_cm / 100`, `up_m = ue_z_cm / 100`.
  ENU position is taken from the **nadir camera centre**, not the body origin.
* **Quaternions** — `wxyz` order, normalised, expressing a
  component-local → UE-world rotation, expressed in the UE world frame.

## `vo/frames.csv`

One row per nadir image. `frame_id` is 0-based and contiguous.

```
frame_id, sim_time_s, image_path,
cam_ue_x_cm, cam_ue_y_cm, cam_ue_z_cm,
cam_ue_qw, cam_ue_qx, cam_ue_qy, cam_ue_qz,
body_ue_x_cm, body_ue_y_cm, body_ue_z_cm,
body_ue_qw, body_ue_qx, body_ue_qy, body_ue_qz,
east_m, north_m, up_m,
heading_deg, camera_tilt_deg,
baro_relative_alt_m, true_agl_m, terrain_elevation_m, ground_hit
```

* `baro_relative_alt_m` is measured relative to the body altitude at frame 0.
* `true_agl_m` and `terrain_elevation_m` come from a downward principal-ray
  trace limited by `GroundTraceDistanceCm`. `ground_hit` reports whether that
  trace hit geometry — rows with `ground_hit` false have no valid AGL.

## `vo/groundtruth.csv`

The same physical quantities at the **full simulation rate** (every tick,
60 Hz by default), without `image_path`. Use it as dense ground truth between
image frames.

## `skyline/observations.csv`

One row per skyline observation, carrying the full pose of both the north and
west cameras plus the nadir camera at the moment of capture:

```
observation_id, sim_time_s,
north_ue_{x,y,z}_cm, north_ue_{yaw,pitch,roll}_deg, north_ue_quat_{w,x,y,z},
nadir_ue_{x,y,z}_cm, nadir_ue_{yaw,pitch,roll}_deg, nadir_ue_quat_{w,x,y,z},
image_path, sim_sky_mask_path,
west_ue_{x,y,z}_cm, west_ue_{yaw,pitch,roll}_deg, west_ue_quat_{w,x,y,z},
west_image_path, west_sim_sky_mask_path,
vo_frame_id, vo_synchronized
```

`vo_frame_id` links the observation to the VO frame captured on the same tick.
When automatic nadir capture is disabled the observation is written
immediately and `vo_frame_id` is `-1`; check `vo_synchronized` before assuming
the link is valid.

## Sky masks

The files under `skyline/sim/` are binary sky masks derived from the two depth
captures, not rendered imagery. A pixel is sky when its depth is at least
`99000000.0` cm (`SkylineSkyDepthThresholdCm`) — i.e. effectively at the far
plane. See [DRONE_SETUP.md](DRONE_SETUP.md) for the capture configuration this
depends on.

## Camera calibration

`ComputeCameraCalibration` derives a pinhole model per capture from the render
target dimensions and FOV: `Width`, `Height`, `HorizontalFovDeg`,
`VerticalFovDeg`, `Fx`, `Fy`, `Cx`, `Cy`. These are written into
`settings.json` rather than repeated per row, so calibration is fixed for the
duration of a run.

## `settings.json`

The run manifest. Includes `run_id`, `level`, `engine_version` (the actual
`FEngineVersion` at record time), the world-frame and quaternion conventions
above, the environment block (`time_of_day`, `hour`, `time_speed`, `clouds`)
as passed to `StartExperiment`, and the path block (`path_id`,
`cruise_speed_mps`, `start_behavior`, `progress_tracking`, and the follower's
tuning including `look_ahead_distance_m`).

Read `settings.json` before interpreting any CSV — it is the authoritative
record of the conventions a given run was produced under.
