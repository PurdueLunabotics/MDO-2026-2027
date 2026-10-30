# Forward-feed excavation — revision 2

The default excavation analysis is a straight pass with the front drum pair
cutting to one `inlet_depth` below the original surface and the rear pair
cutting to twice that depth. The pairs follow the same tracks. Each removes
one fresh layer of thickness `inlet_depth`; the rear pair does not receive
credit for excavating the front pair's layer again.

This is an uncalibrated geometric and work model requested by the student.
It introduces no fitted soil law, measured data, or simulator run. Existing
capture, cutting-pressure, rolling and internal-lift coefficients
remain assumptions. Predictive soil-force, retention and energy errors are
unknown until compared with independent hardware tests.

## Inputs and operating geometry

`feed_speed` is an adjustable speed cap in m/s; `drum_rpm` remains independently
adjustable. `inlet_depth` is the single depth design variable in metres, with
the former cut-depth start/bounds 0.012 m and 0.003–0.025 m. The saved baseline
carries its former 0.014 m value forward. It is both radial scoop protrusion
and the thickness of each freshly removed layer. The inlet spans the whole
axial width of a drum: `inlet_width=drum_length`. Width is not another inlet
design variable. Future drums with narrower inlets need a separate model.

For outer cutting radius R and inlet depth d:

```text
front_dig_depth = d                 front_dig_y = R - d
rear_dig_depth = 2*d                rear_dig_y = R - 2*d
trench_depth = 2*d                  inlet_width = drum_length
```

The report's legacy side-elevation y coordinate is height above the original
ground, equivalent to global z. The inverse linkage geometry determines the
front positive x station and rear negative x station at these target heights,
including the eccentric actuator pin. Both operating poses must lie within
the available travel-to-maximum-extension branch. `operating_reach_margin`
reports the signed operating reserve; guarded invalid geometry retains a
failed `excavation_domain_margin` rather than being accepted as a prediction.

`extended_x` and `extended_y` retain their maximum-extension capability
meaning. They are not the normal forward-digging pose. The additional
`axle_ground_margin = -extended_y` requires the maximum-extension drum axle
to reach ground level or lower. This is the requested geometric provision for
optional vertical digging and drum-supported recovery. It does not predict
recovery traction, driveability, motor-ground clearance, or structural recovery
loads. Those questions require the actual hardware envelope and test data.

## Fresh soil and the RPM/feed constraint

Let L be one drum's full axial width and n the number of scoops per revolution.
Across four drums, the total fresh volume rate during productive cutting is:

```text
soil_supply_rate = 4*L*d*feed_speed
scoop_throughput = 4*L*d^2*n*drum_rpm/60
feed_per_scoop = 60*feed_speed/(n*drum_rpm)
feed_speed_limit = d*n*drum_rpm/60
feed_margin = d - feed_per_scoop
```

Requiring `feed_margin >= 0` prevents translation from demanding more fresh
volume between successive scoops than the assumed inlet can accept. Forward
advance per scoop is a calculated translation distance, not a second vertical
cut-depth setting.
The report plots the current point and this linear feed-speed/RPM boundary.
This is a necessary volume/kinematic screen under the declared scoop model;
it is not a validated granular plunge threshold. An overfeed case keeps its
negative margin and raw force demand. It is not slowed or clipped into
feasibility. The existing small-radius screen remains `R/4-d >= 0`. The old
half-opening heuristic is retired because protrusion and layer depth are now
the same dimension; keeping that heuristic would contradict the requested geometry.

Storage continues to use four shells and the distinction between stored and
in-situ bulk density:

```text
storage_capacity = 4*pi*(R-d)^2*L*usable_volume_fraction*fill_fraction
payload_mass = storage_capacity*stored_soil_density
fresh_volume = payload_mass/(soil_density*capture_fraction)
feed_distance = fresh_volume/(4*L*d)
```

The required productive lane length therefore grows with storage and falls
with capture efficiency. Stationary endpoint intersection no longer limits
payload. No extra soil is credited for lowering, trench endcaps, collapse or
repeated rotation in a cavity. Invalid shell and operating geometry remain
failures; the equations above describe valid designs only.

## Finite tandem pass and timing

The front pair starts the lane. The rear pair enters the same lane after the
rover advances by the longitudinal station separation. The front pair is
raised when its storage is filled; the rear pair then finishes the same
productive lane. This also covers a productive lane shorter than the station
separation. No pre-excavated entrance is assumed or credited.

```text
station_separation = front_dig_x - rear_dig_x
feed_drive_distance = feed_distance + station_separation
```

The rover stops for the rear pair's entry and the front pair's exit. Their
order is determined by which of productive lane length and station separation
is smaller. With d=feed_distance and b=station_separation, the three rest-to-rest
drive segments have lengths `min(d,b)`, `abs(d-b)` and `min(d,b)`. Each uses the
existing triangular/trapezoidal profile bounded by `feed_speed` and
`acceleration`. The zero-length middle segment is omitted. `feed_time` is their
total duration; `pair_cutting_time` is the first segment duration plus the
middle segment duration when d>b. These times therefore include acceleration
and deceleration instead of assuming an instantaneous speed change.

Drums stop during staged arm positioning. The calculation conservatively
reserves two drum starts per pair, including when a short lane would need
fewer restarts. `dig_time=feed_time+4*drum_spinup_time` charges these starts
sequentially, and both starts' rotational kinetic energy are charged without
regeneration. No internal lifting work is assigned to stopped drum time.

Drive work and lane reservation include the additional distance. The area
screen reserves the pass's enclosing rectangle, including drum envelopes and
the center gap, and cumulative cycle reservations must fit the declared
available excavation area. Fresh-lane relocation remains a separate empty
move between cycles; these inputs do not establish a legal site layout.

Arm loads and work retain the full travel-to-maximum-extension envelope as a
conservative capability allowance. Arm positioning is charged sequentially
for the staged pass. Normal digging uses shallower front/rear target poses;
the timing allowance does not claim that both must actually plunge to the
maximum extension on every cycle. `dig_overlap_time` is zero: the removed
stationary model's lowering/cutting overlap is not subtracted from this pass.
Entry/exit soil transients and an actual actuator/controller trajectory remain
unvalidated.

## Loads, drive work and electrical accounting

The pressure screen uses the raw advance per scoop at the commanded speed cap:

```text
cut_force = cut_pressure*(4*L)*feed_per_scoop
```

The full inlet width is engaged; the former 0.15 partial-width multiplier is
absent. This increases cutting-force and traction demands relative to revision 1.
During acceleration/deceleration the pressure-law cutting force scales with
instantaneous feed. Drum cutting work integrates this term using productive
distance, while internal lifting follows actual rotation time. The wheel-drive
tool-work term conservatively applies peak cut force over that distance.

The drum work model retains cutting torque, half-full mean internal lifting
torque, conservative ring inertia and nonregenerative startup. Per-pair drum
torque and power capability are checked with the selected shared motor
provider. Fr remains a diagnostic; there is no newly invented RPM-dependent
capture law.

Feeding also requires wheel drive. The drive screen includes rolling
resistance, the full horizontal cut reaction without cancellation credit,
wheel-speed-dependent motor
capability and available traction after vertical tool reaction unloads the
wheels. Full load is used for drive force acceptance; traction uses empty
rover weight after the vertical reaction as a conservative normal-load reserve.
Rolling work uses half the final payload on average, consistent with the two
staged equal fills. Every drive segment receives a full-load translational
startup allowance without regenerative credit. `feed_motor_margin`,
`feed_traction_margin` and `feed_power_margin`
remain separate signed constraints.

`feed_drive_energy` and `feed_drive_peak_power` are electrical J and peak W.
They are included once in `dig_energy` and the simultaneous `dig_power`
envelope. Each motor's output-shaft work and power pass through its shared
`Gearmotor` efficiency once. Mission continues to sum electrical phase energy;
peak power multiplied by duration is not phase energy. Wheel/drum mass
boundaries, Earth gravity, route, score and competition dimension/mass limits
are unchanged.

## Evidence, domain and reproduction

Implementation: `rover/alternatives/moving_excavation.py`, selected through
`ModelSet.excavation`. The old stationary `Excavation` implementation remains
an explicitly selected reference/compatibility model and is absent from the
default excavation analysis. Schema 7 identifies the unified inlet geometry;
schema 6 introduced the moving pass and its operating, feed and reach diagnostics. Historical
reports must be regenerated before comparing scores or feasibility.

Analytical checks should cover same-track soil conservation, independent
storage/density accounting, the feed/RPM boundary, raw overfeed rejection,
ground-level axle reach, invalid geometry, one-time motor losses and smooth
dependencies. These checks establish numerical behavior, not measured error.
See [verification notes](VERIFICATION.md) for runs actually completed.

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --set feed_speed=0.005 --set drum_rpm=20 --output results/moving_excavation_baseline.json
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/my_change_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/my_change_optimization.json
```

## Input migration

Use `inlet_depth` in new JSON files and `--set` overrides. Moving designs have
no independent `cut_depth`, `scoop_opening`, `scoop_depth_fraction` or
`cut_engagement` input. Historical moving input dictionaries map `cut_depth`
to `inlet_depth`; the other three settings are retired. Retired values and the
migration policy are recorded under `model_metadata.excavation_operation`, and
the CLI prints a migration notice. Mixed `cut_depth`/`inlet_depth` inputs are
rejected. This is a model change, not a geometry-preserving translation: old
35 mm operating layers become the chosen inlet depth, and full width replaces
the former partial-width force factor. Old reports require reevaluation.

`cut_depth` survives only as a derived output equal to `inlet_depth` for the
existing linkage breakout-work port. The legacy `cut_depth_margin` output now
reports the radial `R/4-inlet_depth` reserve. Explicitly selected stationary
reference factories retain their original independent inputs and equations
for historical regression comparisons.

The user-specified depths are not a motor-body collision model. Before using
predictions to choose hardware, measure scoop intake, drum/motor envelopes,
cutting and vertical reactions, retention, feed traction and current/temperature
over representative soil preparations, including overfeed and recovery cases.
No independent validation dataset or quantitative prediction error is available.
