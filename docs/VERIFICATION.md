# Current simulator verification

## Simplified motor energy model, 2026-10-14

The student approved a lower-fidelity motor model before reviewing any new
30-degree climbing screen. This entry supersedes the electrical-model details
in the historical entries below.

- Removed ShaftPhase, ShaftDuty, IntegratedMotor, electrical quadrature,
  equivalent-DC/PWM equations and the drive_motion module. No stator-current
  time histories, RMS currents or required-voltage report outputs remain.
- Existing disciplines retain mechanical loads and pass output-shaft work [J]
  and peak power [W] to Gearmotor.evaluate(work, peak). Electrical estimates
  divide by one provisional combined motor+gear efficiency: drive 0.72,
  drums 0.70. These existing config inputs are now active in the X44 profile.
  They are uncalibrated assumptions, not manufacturer efficiency ratings.
- Preserved the standard 12 V torque-speed envelope, optional current torque
  caps, fixed 100:1 drive gearing, mass inventory, startup torque/kinetic energy,
  route, requirements and all margin signs. Gear mechanical efficiency affects
  torque capability and is not divided into energy again. Zero current limit
  still means undecided; continuous operation and climbing remain unverified.
- Required ports remain; retired optional diagnostics and intentional X44
  energy/peak/battery/score changes are documented in MOTOR_INTERFACE.md.
  Metadata records energy model revision 2 and the actual combined efficiencies.
  The source manifest is revision 2, with unchanged raw CSV and checksum.
- The six saved cases retain all generic output/margin expectations and all X44
  mechanical/mass/envelope expectations. Historical X44 electrical values are
  deliberately not treated as new expected answers. Independent tests check
  work and kinetic-energy limits, gear conversion, no double loss, invalid
  inputs, current-limit failure, energy sums and shared-provider composition.
  The actual excavation component's zero-resistance limit equals 1/2 I omega^2
  divided by efficiency, including both drum pairs once.
- Full suite: 109 tests in 199.074 s, 107 passed and two failed. The generic plate
  cold-start optimization remains unreliable (this run failed final feasibility).
  The X44 restart passes solver success/final feasibility but fails its existing
  score-improvement assertion. Neither assertion was weakened or removed.
  Final focused motor rerun: 13 tests in 6.705 s, 12 passed; the same optimizer-score
  failure remains. All six interface tests pass. The newly separated HTML
  report test and independent component startup-energy test pass.
- Required reference baseline completed. Required reference derivative CLI
  passed derivative comparison, but default SLSQP still exited unsuccessfully;
  its final iterate stays failed in reference_optimization.json. Failed margins
  include width -0.01486 m, battery peak-power reserve -6.292 W, actuator force
  -38.982 N, drum torque -0.2549 N*m and structural deflection -0.001404 m.
  This task does not claim to fix default optimization.
- The X44 supported-start derivative CLI passes derivatives and finishes feasible.
  Its score decreases from 2288.16331382745 to 2282.78957861425 (about 0.235%).
  Starting/final cycle energy is 3.19534/3.18250 Wh and peak electrical power is
  576.159/573.825 W. A feasible solver-success status is not evidence of score
  improvement or a global optimum. Both iterates remain in the report.
- CLI JSON/HTML/N2 and test logs are under ignored results/motor_simple/.
  No raw runs are staged. No commit, push or merge was performed. The new
  30-degree traction screen remains deferred for the student's motor review.

## Shared motor interface cleanup, 2026-10-14

The student requested this refactor for review before any 30-degree traction
work. No physical requirement, geometry bound, soil/motor coefficient, safety
factor or optimizer tolerance was changed. No new climbing screen was added.

- Removed all five X44-specific discipline/mass component copies. Existing
  Excavation, Mobility and SiteRelocation instances now consume a shared
  motor-provider/Gearmotor interface. Existing DrumMass and GeometryMass use
  a separate assembly-inventory option. X44 source verification and its
  unchanged performance equations remain in `alternatives/x44_motor.py`.
- Mechanical shaft duties are produced once: excavation owns soil/fill/spin
  demands; `drive_motion.py` owns shared wheel-shaft acceleration/cruise/brake
  demands. `motors.py` owns gear conversion and electrical evaluation. The
  generic reference retains its original constant-efficiency work approximation;
  X44 retains its original equivalent-DC phase integration. No reference
  computation is run and overwritten by an X44 component anymore.
- The hardware selector now configures the student's chosen components rather
  than silently replacing them. A synthetic LowerLift excavation subclass
  survives both motor selections, and an intentionally different electrical
  backend reaches excavation, route and relocation without changing their loads.
  Custom full replacements without the required option are rejected clearly.
  Empirical example subclasses call the parent's option initialization.
- Required output names/native units, global config names, mass boundaries,
  margin signs and report fields are retained. Component input lists and
  dependencies now reflect the selected provider; unused generic motor inputs
  are no longer consumer ports in the X44 profile. Legacy global config/report
  fields remain explicitly inactive. Motor/provider option provenance is saved.
- Captured six synthetic software regression cases before edits: generic
  reference, generic plate frame, default X44 plate, saved feasible X44 plate,
  explicit 15/30 A limits and another generic speed/ratio/geometry case.
  `tests/fixtures/motor_interface_before.json` contains selected expected values,
  all enforced non-bound margins and old source hashes. It is not calibration.
  Fresh evaluations match all saved outputs with maximum normalized error
  3.73e-16; largest energy-value difference is 2.28e-13 (J/Wh as reported).
  All six feasibility results are unchanged. Full comparisons are saved under
  ignored `results/motor_refactor/comparison.json`.
- Six interface tests pass in 1.614 s: before/after comparison, unchanged
  component classes, shared-backend replacement, student-hook composition,
  reference dependency omissions, and invalid/empty mechanical-duty domains.
  All ten existing motor tests pass after API migration (7.277 s), retaining
  manufacturer-curve, work/conservation, domain, mass/energy, dependency,
  derivative, report and supported-start optimizer assertions.
- The full suite ran 107 tests in 145.993 s: **106 passed, one failed**.
  The failed test is the same unchanged generic plate cold-start SLSQP
  regression already failing before this refactor. The newly added empty-duty
  domain test was run afterward with the six-test interface group. No existing
  assertion, constraint or tolerance was weakened. The full suite is not green.
- Required reference baseline CLI completed. Required default reference
  derivative/optimization CLI passed derivatives but failed SLSQP after 66
  derivative evaluations/407 driver evaluations in 92.908 s. The final
  deflection margin is -9.1293e-5 m. It remains marked failed/infeasible in
  `results/motor_refactor/reference_optimization.json`; this refactor does not
  claim to repair cold-start optimization.
- The explicit X44 supported-start CLI passed derivatives and converged
  feasible in 157 evaluations, saved in `results/motor_refactor/optimization.json`.
  Estimated score is 2006.8319450481367 versus 2006.8319450482652 before:
  difference 1.29e-10 points. This reproduces the previous candidate, not a
  better/global optimum or hardware validation. N2/interface source checks and
  unchanged mass/mission energy accounting pass. `git diff --check` passes.

Review the file map/API in MOTOR_INTERFACE.md. Current limits, thermal/PWM,
gearbox efficiency and hardware inventory uncertainties remain as documented in
X44_MOTORS.md. Slope/rock constraints, travel height and wheel sizing were not
reworked. The requested 30-degree traction screen awaits the student's review.
All changes remain uncommitted; no push or merge was performed.

## Expanded wheelbase and standard X44 hardware, 2026-10-14

The student increased the exploratory wheelbase upper bound from 0.50 to 0.65 m
and selected standard (trapezoidal, not FOC) X44 motors at 12 V. Drive gearing
is fixed at 100:1; drum gearing remains exploratory. Current limits are explicitly
undecided. No slope traction/climbing requirement is claimed verified.

- Added the optional five-factory `--motors x44-standard` integration profile,
  official 6 kB CSV and versioned manifest. Manufacturer endpoint equations
  reproduce the held-out interior curve to the test tolerances (torque 1e-8 N m,
  current/power/efficiency-percent 1e-6, voltage 1e-7 V). The separate rounded
  41 A/393 W specification is reproduced within 1%; it is not an independent
  hardware validation experiment. PWM remains an explicitly unvalidated
  equivalent-DC model. No motor peak is treated as a continuous rating.
- Replaced generic scalable motor/gear masses with published inventory: motor
  0.34019428 kg, selected drive gearbox 0.697 kg including the listed shaft.
  Drum gearbox uses a provisional 0.753 kg three-stage budget, pending stage,
  adapter, shaft and weighed assembly selection. Gear efficiency remains
  provisional. Four-wheel/two-drum-motor boundaries and mission energy balance
  are preserved, with no regeneration credit.
- Ten focused X44 tests pass, including source checksum/reproduction,
  physical work/power limits, invalid speed/current/gearbox domains, PWM supply
  versus stator current, mass/energy conservation, all optional dependency
  omission checks, interface sources, generic-input inactivity, total
  derivatives, HTML reporting and a feasible-start optimizer improvement.
- The full suite at the first eight-test motor revision ran **100 tests in
  459.412 s: 99 passed, one failed**. The failure is the unchanged
  `test_new_profile_optimizer_retains_success_feasibility_and_score_improvement`
  cold-start SLSQP test for the generic-motor plate profile after widening the
  wheelbase bound. The remaining two motor tests were subsequently added and
  all ten motor tests passed in 19.031 s. **The full suite is not all green.**
  No existing assertion, physical constraint or tolerance was removed/loosened.
- The wheelbase-only derivative-checked warm run converged feasible at
  `results/wheelbase_expanded/optimization.json`, 1877.85 points and wheelbase
  0.500025 m. This local solve does not prove that more length has no value.
- The first derivative-checked X44 SLSQP solve is retained at
  `results/x44_standard/optimization.json` as **failed_or_infeasible**. It stopped
  after 102 derivative evaluations/1027 driver evaluations in 472.95 s. Its
  score 2217.48 is not an acceptable design: aluminum stress exceeds its
  allowable by 1.822 MPa, and the minimum rock wheel reaction is -3.678 N.
- A separate feasible start reduced each drum to 0.300 m length, keeping the
  other failed-iterate design values. This is a synthetic numerical start,
  not calibration, stored in `tests/fixtures/x44_standard_feasible_start.json`.
  Explicit supported COBYLA, scaled radius 0.01, all design variables free,
  maxiter 1500 converged in 157 evaluations/12.82 s. Score improves from
  2000.57 to 2006.83; every constraint passes. The separate result is
  `results/x44_standard/restart.json`; its predecessor remains failed.
  A fresh CLI run after final source/report edits also passed the derivative
  check and converged to the same feasible design at
  `results/x44_standard/verified.json` (HTML opened through the app; opening
  request was queued, so actual visibility was not confirmed).
  Final wheelbase 0.648409 m, drum length 0.3009995 m, diameter 0.359 m,
  drum speed 119.972 rpm, exploratory ratio 39.5166, dry mass 36.8671 kg and
  payload 48.4250 kg. Match the drum ratio to available cartridges and rerun.
- That design requests drive peak/RMS stator current 7.753/5.143 A per motor,
  drum peak 23.348 A, drive/drum voltage 9.527/8.254 V on the modeled level
  mission. These do not verify 30-degree climbing, current settings or thermal
  operation. The new HTML card prominently marks continuous operation
  unverified. Generic reference motor coefficients are labeled inactive.
- Fresh reanalysis reproduced feasibility and exactly balanced cycle energy
  to displayed floating-point precision (0.0 J residual). An 801-height audit
  (3204 raised-wheel poses) reproduced the 41-height sweep minima: hull reserve
  0.153118 m and minimum wheel reaction 2.747785 N, both at 0.40 m. Reaction
  balance residual is 2.27e-13 N. That tiny wheel load is not a robust hardware
  allowance. Dense sampling is still not an actual obstacle crossing proof.
- Baseline CLI runs for generic reference and X44 plate profiles completed and
  saved their actual failed constraints; baseline mode does not assert
  feasibility. Both relevant optimization starts passed total-derivative checks.
  A separate generic plate cold-start scaling probe retaining the former
  0.12 m numerical wheelbase span reached its 60-iteration limit and remains
  failed in `results/wheelbase_scale_probe.json`: torque margin -0.07783 N m,
  buckling margin -14.49 N. This numerical experiment was not adopted as a fix.

At the old stability optimum, adding 10 mm per drum at otherwise fixed inputs
increased payload 32.9935 to 34.4847 kg and estimated score 1877.61 to 1939.56,
but failed sphere clearance (-5.034 mm after required clearance), wheel
reaction (-0.221 N), and aluminum stress (3.353 MPa above allowable). Reducing
diameter 10 mm reduced score to 1803.38. These are same-design perturbations,
not reoptimized designs or proof of a global preference. Width is constrained
by coupled interference/load checks, not just inspection mass or storage.

Read X44_MOTORS.md for the equations, source domain, changed instantaneous-power
margin interpretation and missing PWM/thermal/gearbox evidence. Generated
`results/` records are ignored local artifacts; no commit/push/merge was made.

## Loaded 30-degree slope and one-wheel articulation, 2026-10-14

The student requested slope/rock stability next and explicitly kept the
original one-wheel-raised rock case. Slope and rock are separate; no combined
incline/rock, wheel-pair raises, traction law or selected-motor model was added.
Equations, force balance, new ports and domains: TRAVERSAL_STABILITY.md.

- Added `ModelSet.traversal_stability` with three enforced margins: planar
  `slope_tip_margin` [m], sampled `rock_sweep_tip_margin` [m] and ideal
  differential `rock_sweep_wheel_load_margin` [N]. `slope_angle=pi/6` is fixed,
  not optimized. Reference and plate profiles retain their respective rocker
  mass placement. Original endpoint obstacle, drum-wheel and uneven-dump
  constraints retain their meanings.
- **92 local .venv tests passed in 40.376 s**, including 11 independent slope/
  articulation tests. These cover known moment limits and flat axle reactions,
  every slope heading, global/differential force and moment balance, positive
  hull with negative wheel reaction, mirrored raises, mass placement, dense
  sampling, unreachable/singular/invalid domains, smooth derivatives and real
  OpenMDAO sources. Existing mass, reference drum, energy, N2 and all omitted-
  dependency audits pass. Optimizer tests retain success, final feasibility
  and genuine score-improvement assertions.
- Exact prior plate design re-evaluated at unchanged inputs in
  results/stability/previous_design.json: score remains **2479.2303**, but new
  slope reserve is **-40.1456 mm** and minimum platform reaction is
  **-474.403 N** at the 0.40 m pose. The original endpoint hull still passes
  narrowly. A negative wheel normal disproves equilibrium of that prescribed
  four-contact pose; it is not proof that a full rollover must follow.
- Required plate CLI baseline is saved in results/stability/baseline.json.
  It still fails reach at the original 0.40 m wheelbase, plus slope support.
  Invalid-height loads/CG are explicitly guarded diagnostics, not predictions
  for a real 40 cm platform pose.
- Required plate CLI derivative check and default SLSQP optimization pass in
  results/stability/optimization.json: **1877.6091 points**, dry mass **32.6544
  kg**, payload **32.9935 kg**, loaded CG height **299.840 mm**, travel drum
  centers **389.351 mm**. Wheelbase **0.500 m**, track **0.649286 m**, pivot
  offset **20 mm**, drum length **0.221245 m**. All remaining modeled constraints
  pass; 22 iterations, 46 function calls / 47 recorded driver evaluations.
  Aluminum stress envelope **119.898 MPa**, deflection **2.14268 mm**, plate
  strip buckling reserve **762.751 N**. No structural limit was relaxed.
- Final slope pitch/roll reserves are **76.8875 / 151.531 mm** at 30 degrees.
  Limiting GLOBAL static support angle is **39.8206 degrees**. One-wheel hull
  reserve is **108.760 mm**; minimum normal is **0.499643 N**, both worst at
  0.40 m. The reaction condition controls and is near the existing 0.5 N
  numerical reserve. That reserve is not a hardware uncertainty allowance.
- Independent 801-height-per-wheel evaluation (3204 poses) in
  results/stability/dense_audit.json reproduces both minima exactly at stored
  precision. Maximum normalized reaction-balance residual is **1.71e-13 N**.
  This is a finer numerical grid, not continuous-height or physical validation.
- Required reference baseline/derivative-checked cold-start runs were completed.
  The reference SLSQP final remains **failed_or_infeasible**, 1730.7701 points,
  deflection reserve **-0.174709 mm**, in
  results/stability/reference_optimization.json. Other reference starts also
  stopped infeasible or stagnated despite optimizer success; their ignored
  reports retain failures. Default-radius COBYLA requested an invalid physics
  trial and raised an error rather than silently substituting valid geometry.
- A new synthetic feasible reference regression fixture uses the already
  supported COBYLA solver with a **0.02 initial scaled trust radius**, keeping
  the existing tolerance and all physical constraints. Full geometry variables
  remain free; no failure assertion was removed. The smaller steps avoid the
  observed invalid trial. CLI `--cobyla-radius` and model metadata expose that
  setting; SLSQP/default settings are unchanged. Exact CI-style run in
  results/stability/reference_regression_cobyla.json passes derivatives,
  succeeds and ends feasible, score **529.2242 -> 921.2846**. CI and regression
  read tests/fixtures/traversal_stability_feasible_start.json; earlier fixtures
  are retained. This is numerical regression evidence, not calibration or a
  claim of a global optimum.
- Mission energy residual run_energy-cycles*cycle_energy is **0.0 Wh** for the
  plate result at stored precision. No mass group is counted twice. HTML/SVG
  show slope reserves, support hull and wheel-reaction curves; final frame,
  travel and stability SVGs were rendered and visually inspected. N2 files
  include the actual additional CG/mass connections.

These tests do not certify rock traversal or slope climbing. Slope wheel-load
allocation, slip, traction/holding torque, real approach/contact profiles,
post-unloading motion, joint strength/stops, impacts, payload motion and
cross-slope frame loads remain unverified. The selected Kraken/MAXPlanetary
hardware is still not represented by the generic motor model. Material and
mass/CG uncertainty are especially significant near a 0.5 N contact load.
No experimental calibration or simulator validation was introduced. Historical
sections below describe their earlier revisions and constraint sets.

## Requested screen removals and plate revision 2, 2026-10-14

The student explicitly removed the open-frame filled-chassis collision screen
and the uncalibrated Froude caps, and requested a plate connectivity that allows
the suspension hub and lower mounts at the same elevation. No slope, traction,
motor-hardware, ground-clearance requirement or material calibration was added.

- Plate connectivity is now four radial plus three perimeter strips. The lower
  boundary runs B+-S-B-, with three signed triangular openings; no direct
  B+-B- strip or hub/lower-mount vertical ordering is used. Distance between real
  mounting pads, cutout opening, section/buckling/stress, axle packaging and the
  5 mm deflection envelope remain enforced. Flipped triangles and additional
  mechanisms stay failed. Mass, self-weight, CG, yaw inertia, axial stiffness
  and strip buckling all use the same new seven-member inventory.
- `chassis_clearance_margin` is diagnostic only for plate-frame. It remains an
  active constraint for the reference filled-envelope profile. Both profiles
  exclude `dig_froude_margin`/`dump_froude_margin` from their actual constraints
  and feasibility labels. Existing ports/units and legacy diagnostic references
  remain, with explicit JSON metadata and HTML notices. No extra high-speed
  capture/lift loss is assumed; empirical alternatives retain their own domains.
- Full local .venv suite: **81 tests passed in 54.157 s**. New tests include
  independently hand-solved forces for the coplanar lower boundary, moving the
  hub through the lower-mount elevation, and actual driver/report constraint
  membership. Existing equilibrium/elastic-work, failure-domain, mass/mission
  energy, N2 dependency/source, derivative and optimization assertions pass.
- `results/plate_revision2/baseline.json`: baseline remains infeasible for the
  40 cm obstacle at 0.40 m wheelbase; reach -0.0066872 m, guarded tip -0.0230532 m,
  drum-wheel margin -0.0067872 m. Baseline structural screens pass. Guarded
  obstacle values do not establish the requested-height pose.
- `results/plate_revision2/optimization.json`: required CLI total-derivative
  step-size check passed; SLSQP converged, final remaining modeled constraints
  all pass. **2479.2303 points**, dry mass **39.2634 kg**, payload **53.8938 kg**,
  structural mass **3.9112 kg**, 122 recorded driver evaluations. Rocker offset
  **39.506 mm**, lower mount height **146.538 mm**, upper mount **358.017 mm**.
  Travel drum center height **593.908 mm**, loaded CG height **469.429 mm**.
  Excavation **100.128 RPM**, dumping **60 RPM** (existing upper bound), Fr
  **2.01239** as diagnostic only. These speeds follow constant capture/lift and
  generic motor/gear models, not measured retention or the newly selected motors.
  Deflection envelope **2.47879 mm**, plate buckling reserve **1436.105 N**;
  nodal force-balance residual **2.86e-13 N**.
- The previous four-cutout plate result is retained in results/plate_frame/.
  Jointly changing topology and the requested screens increased the local score
  from 2282.8164 and lowered travel centers from 704.761 mm. This is not an
  isolated causal comparison or global-optimum claim.
  At the exact prior design inputs, the new member inventory reduces structure
  mass from 4.08356 to 4.04241 kg; the re-evaluated design remains feasible and
  scores 2284.3875. Saved as results/plate_revision2/same_design.json.
- Reference CLI baseline passes its existing non-obstacle screens and remains
  obstacle-infeasible. Required reference cold-start derivative check passes,
  but final optimization remains **failed_or_infeasible**: 2434.0603 points,
  turn motor margin -0.05167 N, stress margin -22.0569 MPa and deflection margin
  -1.61818 mm. Saved at results/plate_revision2/reference_optimization.json;
  its failure was not converted to success.
- Removing the caps also made the old reference CI start encounter a line-search
  failure. Numerical fixture revision 2 changes ONLY initial dig/dump RPM to
  25/20, preserving all success, feasibility and improvement assertions. This
  is a numerical starting point, not hardware data. The exact existing CI CLI
  command from that fixture passes derivative checks, succeeds and finishes
  feasible at **2439.6207 points**; saved in
  results/plate_revision2/reference_ci_optimization.json. No remaining physical
  constraint, optimizer tolerance or assertion was loosened to recover it.

Reports show the open member schematic and distinguish inactive diagnostics.
After the last plot-label adjustment, all three report tests passed in 1.296 s;
the frame and travel SVGs were rendered and visually inspected.
Actual frame/equipment interference, RPM-dependent retention/dumping, 30-degree
slope support/traction/structural loads, CF material allowables and selected
motor/current/thermal behavior remain unverified. No physical data or simulator
validation was introduced. The following sections retain historical verification
for their stated revisions; their earlier enforced Froude caps and plate topology
do not describe revision 2.

## Optional plate-frame profile, 2026-10-14

Implemented `--structure plate-frame` after the student's frame/load discussion.
This is a preliminary analytical model, not plate FEA or experimental validation.
The reference configuration/factories remain available. See PLATE_FRAME.md for
the exact model, material assumptions, scope and additive/profile-specific ports.

- Local .venv full suite: **78 tests passed in 50.405 s**, including 14 new frame
  tests. Independent actuator force/moment balance, truss nodal equilibrium and
  elastic-work equality, box-section subtraction, mass/energy conservation,
  geometry/packing/mechanism failure cases, width/span trends, N2 sources,
  smooth total derivatives and a complete frame optimizer regression all pass.
- `main.py --structure plate-frame --baseline --output
  results/plate_frame/baseline.json`: the new structural screens pass. The rover
  remains infeasible at the original 0.40 m wheelbase: requested obstacle reach
  margin -0.006687198 m, posed tip margin -0.022280099 m and clearance margin
  -0.006787198 m. No obstacle requirement was relaxed.
- `main.py --structure plate-frame --check-derivatives --output
  results/plate_frame/optimization.json`: derivative step-size check passed;
  SLSQP converged with all modeled constraints passing. Score 341.8832 ->
  **2282.8164**, dry mass 38.2183 kg, payload 51.2393 kg.
  Driver recorded 52 model evaluations and 22 derivative evaluations.
  Final aluminum stress envelope 119.308 MPa, deflection envelope 3.6724 mm,
  CF effective bending stress 121.528 MPa. Plate-strip buckling reserve
  1223.213 N; rocker Euler reserve 5594.519 N; arm Euler reserve 10.0064 N.
  Plate force-balance residual 5.68e-13 N. The 5 mm limit, metal allowable,
  strength factor and structural load multiplier were not loosened.
- Fixed user dimensions remain plate thickness 6.35 mm and CF ID 25.4 mm.
  Several final section widths/walls/spans meet their exploratory lower bounds;
  actual connector dimensions, local wall behavior and CF product properties
  can change the result. The model does not certify the selected sections.
- The reference drum, route, score and mission energy boundaries remain intact;
  no rocker/drum/wheel mass is counted twice. Frame CG and sloping-rocker obstacle
  CG are profile-specific rather than using the old frame rectangle lump.
- Reference CLI baseline and derivative-checked default optimization were also
  run: their scores/failures reproduce the earlier cold-start reference result.
  Reference optimization remains **failed_or_infeasible** (2294.9805 points,
  failed stress/deflection and five other constraints), not an accepted design.
  Its saved report is results/plate_frame/reference_optimization.json.

CF properties and the four-radial/four-perimeter plate topology are provisional.
Two-force strips omit plate-junction bending and whole-plate buckling. Tube/axle
flexure screens are not assembled-frame FEA; connections, local box/CF wall
buckling and recovery/cutting/obstacle structural reactions require separate
verification. No hardware or simulator calibration was performed.

Per-output N2 files now use `<output_stem>_n2.html`, preventing reports of different
profiles in one directory from linking to whichever diagram was written last.

## Obstacle drum-wheel clearance integration

Checked 2026-10-14 after completing the student's obstacle drum–wheel clearance
component and its integration. This is an analytical software/envelope check,
not simulator agreement, hardware calibration or experimental traversal validation.

- Local .venv full suite: **64 tests passed**. The 13 clearance tests include
  zero-pose and hand-calculated pitch/roll limits, rigid-distance conservation,
  cross-pair minimum selection, sphere touching/overlap signs, an explicit case
  of separated cylinders with overlapping enclosing spheres, finite/valid input
  domains, unreachable/boundary rejection, smooth derivatives, actual linkage
  sources and active/reportable constraints. Existing drum/mass, interface and
  N2 dependency checks also pass.
- The active ModelSet.obstacle_clearance factory compares 64 pairs at four
  prescribed single-wheel-raised travel poses. The required buffer reuses
  mechanism_clearance=0.015 m. Wheel/drum envelopes are rigid outer cylinders
  enclosed in spheres; a negative sphere bound is inconclusive about actual
  cylinder collision and is conservatively rejected. Default physical inputs,
  old factories, constraints, solver tolerance, route and score are unchanged.
- `results/obstacle_clearance_baseline.json`: original baseline remains
  infeasible. Reach margin -0.0066872 m, guarded tip margin -0.0274500 m and new
  clearance margin -0.0067872 m. Validity is zero; its gap bound is a guarded
  diagnostic at 0.3933124 m, not a prediction at the requested 0.40 m height.
- With only wheelbase changed to 0.46 m, baseline reach/support/clearance pass;
  conservative gap bound 0.0888998 m and clearance margin 0.0738998 m.
- Required default-start derivative check passed, but SLSQP failed with a line
  search exit in `results/obstacle_clearance_optimization.json`. The final iterate
  is retained as failed/infeasible (motor, battery, linkage/actuator, drum torque
  and structure failures). A run from wheelbase 0.46 m likewise passed derivatives
  but failed battery power, actuator force, drum torque and stress; see
  `results/obstacle_clearance_reachable_optimization.json`. Neither is accepted.
- The original obstacle fixture has a new margin of -0.1598726 m. It is retained
  unchanged as a historical pre-clearance starting vector. A new explicit
  numerical fixture, tests/fixtures/obstacle_clearance_feasible_start.json, is
  verified feasible and uses additional initial motor/battery/structural reserves,
  interior cut/RPM settings and shifted pivots. The historical reference wheel
  coefficient 8 kg/m² is explicit. These are synthetic numerical inputs, not
  measured hardware. CI and the optimizer regression now use this fixture with
  the same derivative, optimizer-success, final-feasibility and objective-
  improvement checks; the test additionally checks starting feasibility.
- Derivative-checked optimization from that interior start converged feasible:
  `results/obstacle_clearance_interior/optimization.json`, 2173.4109 → 2177.8442
  modeled points. Final reach reserve 0.0571379 m, gravity tip reserve 0.0019441 m,
  conservative gap bound 0.1077071 m and clearance margin 0.0927071 m. Energy
  accounting residual run_energy-cycles*cycle_energy is 0.0 Wh at stored precision.
  This locally converged result is not a global-optimum or hardware-performance
  claim. The full regression suite reproduces a successful feasible improvement.

JSON and HTML include the additive clearance fields, validity/evaluated height
and model evidence. HTML distinguishes guarded diagnostics and level-ground
schematics from the four posed checks. The alternatives package description now
reflects its two active obstacle factories. No fitting dataset exists or is
implied. Other hardware interference, intermediate climbing poses, flexible
wheel/frame deformation, terrain dynamics and the underspecified 0.50 m ditch
remain outside the screen. The default-start solver failure remains unresolved.

Historical verification records follow.

Checked 2026-10-11 after activating the requested 0.40 m raised-wheel gravity
screen. The user confirmed centered rocker pivots, an average-angle differential
and an estimated +0.10 m offset above the wheel-center line. This offset is
editable. The 0.50 m deep ditch is circle-like but lacks width/radius, and remains
explicitly unverified in documentation, report metadata and the HTML report.
See [OBSTACLE_TRAVERSAL.md](OBSTACLE_TRAVERSAL.md) for equations/domain.

- Local .venv full suite: all 51 tests passed. Thirteen obstacle checks cover
  independent rectangle/triangle limits and moment balance, rigid rocker/shaft
  geometry and platform heights, mirrored cases, mounted/suspension mass
  conservation, zero-height limits, high-CG failure, below-axis first-branch
  maximum, invalid domains, smooth valid-pose derivatives and N2 connections.
- Original baseline: `results/obstacle_traversal/baseline.json` is now infeasible.
  Its maximum wheel-height reach is 0.39331 m, 0.00669 m below the new requirement.
  The flagged guarded tip value is not a prediction at 0.40 m. All pre-existing
  baseline outputs are unchanged exactly; old constraints/bounds are unchanged.
- Changing only starting wheelbase to 0.46 m gives a feasible baseline, saved at
  `results/obstacle_traversal/reachable_start.json`, with 0.04722 m reach reserve
  and 0.06795 m tip reserve.
- Default-start derivative check passed, but SLSQP failed. The final iterate
  `results/obstacle_traversal/optimization.json` remains infeasible with width,
  drive/load/torque and structural failures. A solve from wheelbase 0.46 m also
  failed: `reachable_optimization.json`, frame deflection about -5e-5 m. A
  stiffer initial tube (0.065 m diameter) failed stress and deflection:
  `stiff_start_optimization.json`. None is an accepted design.
- The previous reference optimum already passes the added screens. A restart
  from it with starting frame diameter increased by 0.001 m passed its derivative
  check and converged feasible: `results/obstacle_traversal/reference_restart.json`,
  2060.4052 modeled points, 0.07221 m reach reserve and 0.10741 m tip reserve.
  Its fully loaded travel pose has about 29.3 degree pitch and 15.6 degree roll.
  The numerical test fixture `tests/fixtures/obstacle_feasible_start.json`
  records this full starting vector and provenance. The optimizer regression
  now uses that feasible start, retaining its derivative, success, feasibility
  and objective-improvement assertions. Default config values remain unchanged
  apart from adding the two requested geometry/requirement inputs.

Energy accounting matches cycles*cycle_energy to 2e-15 Wh in the restart. The
reference drum and existing port/dependency tests pass. Rocker mass is a subset
of structure_mass; new mounted-body ports exclude it and the wheel/drive masses.
These are analytical/software checks without hardware calibration. The optimizer
results are local, and the cold-start failures remain unresolved. Passing the
added necessary static screens does not verify joint load equilibrium/stops,
climbing path, torque/traction, collisions, soil or dynamic traversal.

Checked 2026-10-10 after adding a selectable actuator pin along the arm.
All 38 tests passed using the local .venv. New checks cover pin/drum circle
closure, numerical virtual work, unchanged gravity work at matched drum poses,
invalid fractions/triangles, arm-length conservation in guarded trials,
derivatives at an interior pin position, N2 dependencies, and SVG pin endpoints.
The default fraction is fixed at 1 in ASSUMPTIONS; moving it into DESIGN enables
optimization without changing its physical meaning.

- Reference baseline: `results/arm_attachment/baseline/design.json`, feasible.
  Pre-change versus post-change reference outputs and constraint margins differ
  by at most 1.2e-16 in their native units (floating-point roundoff).
- Default derivative-checked optimization:
  `results/arm_attachment/reference_optimization/design.json`, converged feasible
  at 2060.8794 modeled points, with the reference drum and tip attachment.
- Matched-pose interior baseline:
  `results/arm_attachment/interior_baseline_matched/design.json`, feasible at
  fraction 0.95, with actuator travel/start/dig lengths of
  0.352987/0.424022/0.653223 m and a 0.570 m B-to-pin distance.
- Optimizing the fraction as an additional design variable from the original
  start failed SLSQP's line search; the saved final iterate in
  `results/arm_attachment/final_optimization/design.json` remains infeasible,
  with approximately -8.2e-5 m frame-deflection margin. An additional start at
  fraction 0.9 also failed. Those failures are not accepted designs.
- A separate restart from the failed adjustable-pin design, increasing only
  the starting frame diameter by 0.001 m, passed its derivative check and
  converged feasible at `results/arm_attachment/warm_restart/design.json`:
  2057.5359 points and fraction 0.9121. This demonstrates local start sensitivity,
  not a global optimum or experimentally validated attachment position.

These are analytical/software checks, with no new calibration data. Actuator
body/arm interference and pin-bracket strength/mass remain outside the model;
the elementary structural envelopes and equivalent linkage mass distribution
are still approximations. Constraint limits, energy accounting and test
acceptance thresholds were not relaxed.

Checked 2026-10-02 for the collaboration handoff: 32 tests passed, baseline
feasible, and derivative-checked SLSQP converged feasible. Separating wheel and
drum mass into replaceable components changed no baseline or optimized design
values or reported physics outputs at the reference inputs. Combined wheel,
drum, frame and linkage replacements were exercised in one test. The workflow
under `.github/workflows/` is ready for a standalone repository, but has not
run on GitHub while this folder remains nested in the all-years repository.

Checked 2026-10-02 after making track width a design variable and narrowing
TipStability to one full pair versus one empty pair at full extension. All 31
tests passed. The baseline was feasible. The derivative-checked default SLSQP
solve converged feasible at `results/extended_tip_optimization.json`: 2060.9
modeled construction points, 0.574 m track width, 0.487 m wheelbase and 0.0523 m
static tip margin for that prescribed state. A solve started from the prior
width-variable optimum converged feasible at 2043.3 points, confirming that
these are local optima. The 2060.9-point design would also pass the former
all-pattern tip screen with about 0.0314 m margin; its score gain is therefore
from finding a different local design, not evidence that relaxing the tip screen
was necessary. Other half-loaded arm positions need a software interlock;
the present model does not verify the interlock or dynamic dumping stability.

## Earlier verification under broader tip screens

Checked 2026-09-30. These are software and numerical checks, not experimental
validation. The reference physics coefficients are uncalibrated.

- `python -m unittest discover -s tests -v`: all 30 tests passed. They include
  independent circle closure for offset actuator pivots, a 16-pattern uneven-load
  tip-envelope check with a failure case, reachable-soil and wheel/trench checks,
  dependency audit, energy accounting and a complete optimization.
- `python main.py --baseline --output results/asymmetric_tip/baseline.json`:
  feasible, with a 0.108 m static uneven-load tip margin.
- `python main.py --check-derivatives --output
  results/asymmetric_tip/optimized.json`: derivative step-size check passed.
  SLSQP converged with all modeled constraints passing in 27 derivative
  evaluations and 77 driver/model evaluations. The result has 2016.0
  construction points, 42.41 kg total retained payload and a 0.0133 m static
  tip margin. The worst pattern is both front drums full and both rear drums
  empty (mask 3), with front/rear arm positions allowed to differ.
- A second SLSQP solve started at wheelbase 0.380 m and drum diameter 0.359 m,
  saved at `results/wheelbase_probe/optimization.json`. Its derivative check
  passed and it converged feasible in 16 derivative evaluations and 40 driver
  evaluations. The final design has 2041.8 points, 0.453 m wheelbase,
  0.321 m drum diameter and 0.0320 m static tip margin. This higher-scoring
  result confirms start-point sensitivity; it does not establish a global optimum.

The static tip result is only a level-ground CG/support-polygon screen. It does
not validate a moving dump, dump reaction, slopes, wheel unloading or changing
terrain support. The mission still deposits at the raised travel pose; the
stability screen also checks extended positions. The wheel-diameter screens
remain placeholders for trench traversal and travel obstacle clearance. They do
not assess slip, sinkage, contact dynamics or whether the rover can exit a
disturbed-soil pit. The reported SLSQP result is local, not a certified global
optimum.

Generated `results/` files are ignored local artifacts; earlier reports use
different equations and are not directly comparable.
