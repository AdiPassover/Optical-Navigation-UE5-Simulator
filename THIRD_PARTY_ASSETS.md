# Third-Party Assets

None of the assets listed on this page are included in this repository.

The simulator levels reference them by Content Browser path. To reconstruct a
level as it was used for the thesis experiments, you must acquire each asset
yourself from its original source, under that source's own license, and place
it at the **exact** Content Browser path given below — otherwise Unreal cannot
resolve the references and the level will open with missing actors.

All of these are commercial or separately-licensed assets. Their licenses do
not permit redistribution as part of this repository, so they are excluded
deliberately rather than by oversight.

---

## Required to open the experiment levels

### 1. Quadcopter / drone

| | |
|---|---|
| **Purpose** | The UAV pawn. Carries the camera rig (nadir, north, west and the two depth captures) and the logging components. Referenced by **every** level. |
| **Source** | https://www.fab.com/listings/1a6818f8-2c0e-4ab9-93d9-6ff2d432a734 |
| **Install to** | `Content/Quadcopters/` |
| **Referenced as** | `/Game/Quadcopters/Blueprints/...` |
| **Included here?** | **No.** |

> **Note on modification.** The drone Blueprints used for the thesis are
> *modified* versions of this pack's `BP_Quadcopter_A` / `BP_Quadcopter_B`,
> with the capture rig and `UExperimentLoggerComponent` added. Because those
> Blueprint assets are derivative works of licensed Marketplace content, they
> are **not** redistributed here. See
> [docs/DRONE_SETUP.md](docs/DRONE_SETUP.md) for the component layout and
> tags needed to rebuild them on top of your own copy of the pack.

### 2. Asian village / hills background

| | |
|---|---|
| **Purpose** | Village buildings and surrounding terrain dressing for the `asian_village_hills_background` level. |
| **Source** | https://www.fab.com/listings/cebef7c2-a093-42e0-a3ce-dcd7c06317da |
| **Install to** | `Content/Asian_town/` |
| **Referenced as** | `/Game/Asian_town/...` |
| **Included here?** | **No.** |

### 3. Large flat city

| | |
|---|---|
| **Purpose** | Dense urban environment for the `large_flat_city` level, loaded as a sub-level. |
| **Source** | https://www.fab.com/listings/cfea5a68-2931-40a5-a578-82261247e664 |
| **Install to** | `Content/ChineseCity/` |
| **Referenced as** | `/Game/ChineseCity/Main` (sub-level) |
| **Included here?** | **No.** |

> The level expects the pack to resolve at `/Game/ChineseCity/`. If your Fab
> download installs under a different folder name, either rename the folder to
> `ChineseCity` on import or fix up the sub-level reference inside
> `large_flat_city`.

### 4. Seyeonjeong Pavilion

| | |
|---|---|
| **Purpose** | Pavilion structure and surrounding props for the `hills_with_forest` level, which is the project's default map. |
| **Source** | Obtain from Fab. **URL to be confirmed by the author.** |
| **Install to** | `Content/SeyeonjeongPavilion/` |
| **Referenced as** | `/Game/SeyeonjeongPavilion/...` |
| **Included here?** | **No.** |

### 5. MAWI Landscape Auto Material

| | |
|---|---|
| **Purpose** | Auto-material applied to the landscape in **both** mountain levels (`mountains`, `mountains_less_steep`). Without it the terrain geometry loads but renders untextured, which materially changes the appearance of captured imagery. |
| **Source** | MAWI United. Product documentation: https://www.mawiunited.com/docs/doc_mawi_product_ue_online.html — **Fab store URL to be confirmed by the author.** |
| **Install to** | `Content/MWLandscapeAutoMaterial/` |
| **Referenced as** | `/Game/MWLandscapeAutoMaterial/...` |
| **Included here?** | **No.** |

### 6. Dynamic Mesh Creator

| | |
|---|---|
| **Purpose** | Referenced by the `large_flat_city` level. |
| **Source** | Obtain from Fab. **URL to be confirmed by the author.** |
| **Install to** | `Content/DynamicMeshCreator/` |
| **Referenced as** | `/Game/DynamicMeshCreator/Map/...` |
| **Included here?** | **No.** |

---

## Present in the original project but not required

These packs existed in the author's local project but are **not referenced by
any level in this repository**. They are listed only so the exclusion is on the
record; you do not need them to reproduce the experiments.

| Asset | Local folder | Status |
|---|---|---|
| Downtown West | `Content/Downtown_West/` | Unreferenced. Not included, not required. |
| Gothic Cathedral | `Content/Gothic_Cathedral/` | Unreferenced. Not included, not required. |
| Utopian City | `Content/UtopianCity/` | Unreferenced. Not included, not required. |
| Epic Starter Content | `Content/StarterContent/` | Unreferenced. Not included. Ships with Unreal Engine; add via *Add > Content Feature Pack* if desired. |
| Off World Live Livestreaming Toolkit | `Plugins/OWLLivestreamingToolkit/` | Commercial plugin, disabled in the original `.uproject`. Removed from the plugin list; not required. |

---

## Engine

Unreal Engine 5.4 is required but is not distributed here. It is governed by
the Unreal Engine EULA: https://www.unrealengine.com/eula
