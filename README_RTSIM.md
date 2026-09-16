# RTSIm native SUMO prototype

Experimental RTSIm integration based on SUMO commit [`6bd8e5fcde859fa0016bda327129ed04e16892c6`](https://github.com/eclipse-sumo/sumo/commit/6bd8e5fcde859fa0016bda327129ed04e16892c6). This fork branch provides source for review; validation is ongoing, and it is not an accepted upstream SUMO feature.

The opt-in `rtsim` vehicle device configures individual vehicles with automation presets, drag coefficients, and road-surface inputs for emission calculations. Native Plexe controllers can reuse the Krauss dawdling operation while retaining their own control laws and engine dynamics.

## Configuration

Enable the device with `has.rtsim.device=true`. Select the model/controller explicitly and choose `device.rtsim.automation-level` separately for each vehicle, including vehicles sharing a vType.

| Level | Sigma | Tau (s) |
| --- | ---: | ---: |
| 0 | 0.5 | 1.00 |
| 1 | 0.4 | 0.95 |
| 2 | 0.3 | 0.90 |
| 3 | 0.2 | 0.80 |
| 4 | 0 | 0.70 |
| 5 | 0 | 0.60 |

These are research presets, not SAE capability definitions. Explicit `device.rtsim.sigma`/`tau` override the level pair; omit them on both vehicle and vType when using presets directly. Each parameter resolves from vehicle, then vType, then default.

| `carFollowModel` | Device controller | Tau meaning |
| --- | --- | --- |
| `Krauss` | Omit or `unchanged` | Native Krauss headway |
| `CC` | `ACC` | Plexe ACC time headway |
| `CC` | `PLOEG` | Ploeg CACC headway `h`, plus ACC fallback headway |
| `CC` | `CACC` | ACC fallback headway; PATH cooperative mode uses explicit `device.rtsim.cacc-spacing` in metres |

All managed CC modes require `desired-speed` in m/s. Cooperative followers explicitly name their platoon `leader` and immediate `front` vehicle; both peers must use CC. The device enables ideal native state feeding while the declared peers and actual predecessor match, otherwise using ACC fallback. Sigma state belongs to each vehicle. Sharing the dawdling operation does not make Plexe and Krauss equivalent controllers.

This example uses an upstream passenger-car emission sample to illustrate the API. Define route `trip` and a CC/ACC vehicle named `leader` separately.

```xml
<vType id="automated" carFollowModel="CC" lanesCount="1"
       emissionClass="PHEMlight5/PC_EU4_D_MW"
       accel="1.5" decel="4.5" tauEngine="0.5"/>
<vehicle id="follower" type="automated" route="trip" depart="0">
    <param key="has.rtsim.device" value="true"/>
    <param key="device.rtsim.controller" value="PLOEG"/>
    <param key="device.rtsim.automation-level" value="3"/>
    <param key="device.rtsim.desired-speed" value="20"/>
    <param key="device.rtsim.leader" value="leader"/>
    <param key="device.rtsim.front" value="leader"/>
    <param key="device.rtsim.cd" value="0.6"/>
    <param key="device.rtsim.road-type" value="primary"/>
</vehicle>
```

Plexe sigma defaults to the simulation timestep; an explicit `device.rtsim.sigma-step` must be a positive multiple of that step. Krauss uses its native vType `sigmaStep`. The separate SUMO `ACC`/`CACC` models retain an earlier experimental hook distinct from the Plexe shared-Krauss path.

For fixed drag, use `device.rtsim.cd`. For table lookup, instead supply `model`, `platoon-size`, one-based `position`, `gap`, and `cd-bound`, prefixed with `device.rtsim.`, plus `--device.rtsim.cfd-file`. CSV columns are:

```text
model_id,platoon_size,gap_m,position,cd_lower,cd_upper
```

CFD lookup uses declared size, position, and nominal gap with interpolation inside the supplied curves. These inputs remain fixed during a run; runtime spacing or rank changes do not update them. Surface presets are `primary`, `secondary`, and `cross_country`; direct Fr0 and edge parameters are also supported. Cd/Fr0 feed emission calculations. The device reuses PHEMlight5's existing coefficient-override facility and extends the legacy PHEMlight path.

Tripinfo contains an `<rtsim>` element with configured values and cooperative/fallback counters. Current device operation is microscopic and single-threaded; state saving/loading is not supported. Development evidence comes from headless Windows runs.

## Reviewer source map

- [Device configuration, topology, output](src/microsim/devices/MSDevice_RTSIm.cpp) and [CFD lookup](src/microsim/devices/RTSImCFDTable.cpp).
- [Shared Krauss operation](src/microsim/cfmodels/MSCFModel_Krauss.cpp), [Plexe integration](src/microsim/cfmodels/MSCFModel_CC.cpp), and [vehicle state](src/microsim/cfmodels/CC_VehicleVariables.h).
- [Energy parameter overlays](src/utils/emissions/EnergyParams.cpp), [legacy PHEMlight adapter](src/utils/emissions/HelpersPHEMlight.cpp), and [legacy CEP](src/foreign/PHEMlight/cpp/CEP.cpp).
- [Tripinfo schema](data/xsd/tripinfo_file.xsd) and [regression harnesses](tests/rtsim_local).

Review the MinGW/no-zlib portability changes in [src/CMakeLists.txt](src/CMakeLists.txt) and [OutputDevice_File.cpp](src/utils/iodevices/OutputDevice_File.cpp) separately from the device logic.

## Build and checks

Follow the upstream [build instructions](README.md#build-and-installation), then build this branch's `sumo` and `netconvert` targets. Set `SUMO_HOME` to this checkout and make runtime library dependencies available. From the repository root, replace the executable placeholders below with your built binaries; use `.exe` where appropriate.

```text
cmake --build build --target sumo netconvert --config Release
python tests/rtsim_local/run_integration.py --sumo "path/to/sumo" --netconvert "path/to/netconvert" --phemlight-path "data/emissions/PHEMlight5"
python tests/rtsim_local/run_controllers.py --sumo "path/to/sumo" --netconvert "path/to/netconvert" --phemlight-path "data/emissions/PHEMlight5"
python tests/rtsim_local/run_controllers.py --sumo "path/to/sumo" --netconvert "path/to/netconvert" --phemlight-path "data/emissions/PHEMlight5" --levels-only
```

These checks use upstream's bundled passenger-car sample. `--models CC` narrows controller checks to Plexe. `--levels-only` verifies actual native sigma/headway values for all six levels. Controller tests use bundled TraCI tooling; ordinary device operation does not require an external TraCI controller.

Truck-specific tests require separately supplied `RT_II_D_EU0.veh`, `RT_II_D_EU0.csv`, and `RT_II_D_EU0_FC.csv` files. They are not included in this change. The legacy harness copies them under current SUMO filenames and compares device overrides with edited-copy controls:

```text
python tests/rtsim_local/run_legacy.py --sumo "path/to/sumo" --netconvert "path/to/netconvert" --source "path/to/original-truck-data"
```

Additional scripts provide a matched flat-road controller comparison and Krauss refactor parity; see their `--help` options. Pass explicit binary/data paths because some defaults refer to the development layout. Parity requires a preserved pre-refactor prototype binary accepting `--device.rtsim.probability`; no reference binary is supplied here. Generated outputs stay under the harness directories. Research datasets, manuscripts, result bundles, and compiled executables are not part of this source publication.
