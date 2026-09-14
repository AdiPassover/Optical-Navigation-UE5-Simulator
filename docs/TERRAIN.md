# Terrain heightmaps

## What is included

`Resources/Heightmaps/` contains the two mountain terrain heightmaps generated
for this project with **World Machine Basic**:

| File | Format | Used by |
|---|---|---|
| `MountainWorld_01_Height_1024.png` | 1024 × 1024, 16-bit grayscale PNG | `Content/Levels/mountains` |
| `MountainWorld_02_Height_1024.png` | 1024 × 1024, 16-bit grayscale PNG | `Content/Levels/mountains_less_steep` |

These are the unmodified World Machine height outputs, committed byte-for-byte.
They are original outputs of this project and are covered by the repository's
MIT license.

The `.tmd` World Machine project files are **not** included: they are only
useful inside World Machine, and the PNG height outputs are what Unreal
actually consumes. The heightmaps are sufficient to rebuild the terrain.

## Do I need to import them?

**Usually not.** The two mountain levels in `Content/Levels/` are World
Partition levels whose landscape data is already committed under
`Content/__ExternalActors__/Levels/` and `Content/__ExternalObjects__/Levels/`.
Opening those levels gives you the terrain as used for the experiments.

The heightmaps are included for **provenance** — so the terrain's origin is
documented and auditable — and so the terrain can be regenerated, rescaled or
modified.

## Importing a heightmap into Unreal

If you do want to rebuild the landscape from source:

1. Open the target level.
2. Switch to **Landscape** mode (`Shift+2`).
3. Select the **Manage** tab, then **Import from File**.
4. Set **Heightmap File** to one of the PNGs in `Resources/Heightmaps/`.
5. Unreal will report the file resolution. **1024 × 1024 is not one of
   Unreal's recommended landscape resolutions**, so the import dialog will
   offer the nearest valid section/component layout — accept the suggested
   layout, or set the landscape resolution explicitly before importing.
6. Set the landscape **Scale** to taste. Z scale controls terrain relief; the
   16-bit range maps linearly onto it.
7. Click **Import**.

> The exact scale values used for the thesis levels are stored inside the
> committed landscape actors rather than in any text file, so they are not
> reproduced here. If you need the precise figures, read them off the
> `Landscape` actor's transform in `Content/Levels/mountains` — that is the
> authoritative record.

## A note on the stored reimport path

Unreal records the path a landscape heightmap was last imported from. In the
two committed landscape actors that path pointed at the author's local machine,
so the username segment was redacted before publication:

```
../../../../../../Users/REDACTED_USER/Documents/World Machine Documents/MountainWorld_0N_Height Output_1024.png
```

The replacement is the same byte length as the original, so the asset is
otherwise bit-identical. This string is editor convenience metadata only — it
has no effect on the landscape geometry. If you want *Reimport* to work,
repoint it at your local copy of `Resources/Heightmaps/`.

## Landscape material

Both mountain levels apply a landscape material from the **MAWI Landscape Auto
Material** pack, which is third-party and not redistributed. Without it the
terrain geometry still loads, but renders untextured — which materially
changes the appearance of captured imagery, and therefore the optical
navigation results. See [../THIRD_PARTY_ASSETS.md](../THIRD_PARTY_ASSETS.md).
