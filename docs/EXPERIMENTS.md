# Data collection and higher-fidelity pathways

For an example of how to preserve field measurements without silently fitting
them to a different drum, see [UVA 2026 observations](UVA_2026_FIELD_OBSERVATIONS.md).

## Shared requirements

Record tool/version, git revision, CAD hash, units, geometry, gravity, material
batch, particle-size distribution, moisture, bulk density, compaction procedure,
initial terrain, operating sequence, sensor zero/calibration, sampling rate and
repeat ID. Save failures as well as successes. Keep raw data immutable. Process
with a checked-in script and include units in CSV column names. Use the same
loading/energy conventions as MODEL_DEVELOPMENT.md.

Recommended manifest fields: schema_version, dataset_id, data_kind, author, date,
source_uri, source_sha256, tool_version, model_revision, geometry_hash, material,
gravity_m_s2, conditions, varied_inputs, valid_domain, output_definitions,
uncertainty, train_run_ids, validation_run_ids, validation_metrics, limitations.
Never label synthetic fixtures as experiments or DEM results.

## Simple bench tests: highest-value first

**Drums:** vary RPM, scoop protrusion, penetration trajectory and fill state.
Measure torque (or current with a calibrated torque conversion), electrical
power, angular speed, retained mass, spilled mass, excavation geometry and elapsed
time. Weigh after a standard lift/transport/dump sequence. Empty/free-spinning,
filled/free-spinning and cutting trials separate bearing/drive losses, internal
recirculation and soil cutting. Record startup transients and jams. Characterize
digging and dumping separately; dumping RPM need not equal digging RPM.

**Escape/mobility:** use the actual wheel/suspension geometry. Test full, empty
and one-pair-loaded states. Sweep step height on a rigid fixture first, then repeat
with compacted and disturbed soil. Log individual wheel torque/speed, body pose,
wheel sinkage/slip, battery energy and whether both axles escape. Define success
before testing (complete exit without assistance within a time/thermal limit).
Do not convert one successful run into a universal height limit.

**Structure:** weigh members/joints separately. Measure deflection under known
loads at drum pivots and wheel mounts. Validate boundary conditions and joint
compliance before raising model fidelity. Calibrate mass/stiffness independently
of soil models; carry actual joint/fastener/weld allowables into later FEA.

**Uneven-load stability and linkage:** weigh all four drums separately at empty
and retained-full states. With the rover restrained against a fall, sweep the
front and rear arms independently, including extended deposition poses and one
full drum or one full pair against empty drums. Record each wheel's normal force,
body pitch/roll and the arm/actuator pivot positions. Measure actuator force
against arm angle for several loads, especially near the weakest leverage. Use
these measurements to validate a CG/support model before testing moving dumps;
repeat on slopes and disturbed soil only with a safe test fixture.

Example raw drum columns:
`run_id,time_s,drum_angle_rad,drum_speed_rpm,shaft_torque_Nm,voltage_V,current_A,
shaft_x_m,shaft_z_m,force_x_N,force_z_N,retained_mass_kg`.
Compute electrical energy by integrating V*I, mechanical energy by integrating
torque*angular_speed. They are different outputs; never apply drive efficiency twice.

## Altair EDEM: excavation/internal transport

1. Match CAD and the physical soil preparation. Calibrate particle contacts against
   independent bulk tests (density, angle of repose, shear/penetration); matching
   angle of repose alone does not establish correct excavation resistance.
2. Prescribe the SAME stationary plunge, RPM ramp, lift and reverse-dump sequence
   used by this model. A translating-drum test needs a distinct operating model.
3. Record force/torque by axis, retained mass versus time, spills, cavity geometry
   and motor-work integral. Compute endpoint accessible volume separately from
   storage capacity. Include filled and nearly empty states.
4. Study particle-size/coarse-graining, time-step and boundary sensitivity. Repeat
   with independent particle beds. Record all software/contact-model settings.
5. Validate against held-out physical tests. Export a compact deterministic response
   with a validity classifier/envelope. Run EDEM offline, not once per FD call.

The default cut_pressure and capture/lift factors are not
calibration targets. Replace them using evidence; fitting EDEM to those guesses
would merely reproduce the assumptions with more computation.

## Project Chrono: wheels/terrain/escape

1. Build the four-wheel side-rocker suspension, drum payload positions, wheel
   geometry and actual motor/gearbox maps. Check rigid-ground mass properties,
   step equilibrium and energy conservation before adding deformable terrain.
2. Select and document an appropriate terrain/contact model. Calibrate wheel pull,
   sinkage and slip independently. Soil parameters do not transfer automatically
   between EDEM, SCM-type terrain, other contact solvers and bench materials.
3. Simulate both front and rear step stages, slopes, asymmetric loading and the
   specified fixed-time route. Log per-wheel normal force, torque, slip, sinkage,
   path error, electrical energy, peak power and escape success/time.
4. Check time-step, terrain resolution, contact stiffness and controller sensitivity.
   Validate held-out trajectories, not only a single tuned climb.
5. Replace TrenchTraversal or Mobility through ModelSet using a validated response.
   Preserve route timing constraints;
   a failed 45-second trajectory cannot be reported as a feasible slower traversal.

## Before training a surrogate

Define independent design and operating inputs. Include fill state, geometry,
soil preparation and gravity where they vary. Use space-filling experiments plus
extra samples near failure boundaries, and store feasible/infeasible outcomes.
Split by physical run/bed. Report errors by region, not just one global R-squared.
Use conservative uncertainty bounds for feasibility and test extrapolation rejection.
Do not smooth across failure regimes without evidence.
Optimize the surrogate only inside validated coverage, then re-evaluate candidate
designs with the original simulator and selected physical tests.
