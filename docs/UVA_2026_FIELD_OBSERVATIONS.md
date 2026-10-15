# UVA 2026 bucket-drum observations

These are comparison observations, **not calibration data for the reference
drum**. The 2026 UVA drum was described by [UVA Engineering](https://engineering.virginia.edu/news-events/news/uva-engineerings-mars-team-wins-nasa-2026-lunabotics-championship)
as a curved helix. This repository currently models a full-width drum with a
constant capture and internal-lift factor. Geometry, loading sequence and soil
preparation may differ.

## Provenance and raw observations

- User-provided estimates from competition video/photo: one drum mass about
  1.7 kg, outer diameter about 0.320 m, axial width assumed 0.320 m, and radial
  scoop protrusion about 0.034 m. The exact mass boundary (shell, shaft,
  fittings or motor) is unconfirmed. The photo suggests approximately half-full
  drums, but does not measure internal volume or retained mass. Reported
  full-speed-to-stop time is approximately 0.5 s with uncertainty of several
  tenths of a second; spin-up was not measured.
- User-counted excavation rotations/time: 24/14 s and 18/12 s, giving 102.9 and
  90.0 average rpm. User-counted deposition rotations/time: 9/7 s, 7/5 s,
  7/4 s and 8/4 s, giving 77.1, 84.0, 105.0 and 120.0 average rpm. These
  averages include whatever acceleration, deceleration or pauses occurred in the
  counted intervals; they are not steady-state tachometer readings.
- `2025-2026 Competition Team Notes (Responses).xlsx`, `Form Responses 1`
  row 43: Y43=122,248 cm³, Z43=112 s total excavation, AB43=8 cycles;
  AC43=125,587 cm³, AD43=117 s total excavation, AF43=5 cycles. The
  `2026_Lunabotics_Scores_UCF (3).xlsx` `Summary` sheet D16 and F16 gives
  122,247.7784 and 125,586.6332 cm³ of scored berm construction. The notes
  have two UVA responses: J43 says 25 L per drum; J44 says 24 L in practice and
  48 L theoretical, without a clear per-drum/system convention. The user
  confirmed that the 2026 rover had four collection drums, all collecting in
  each single-dig, single-delivery cycle.

## Derived quantities and interpretation

- The two scored runs delivered 15.28 and 25.12 L of berm volume per counted
  cycle. With four collecting drums and one dig/delivery per cycle, the delivered
  equivalents are **3.82 and 6.28 L per drum**, respectively. Run 2 averaged
  23.4 s of recorded excavation per cycle; a 12 s interval counted from video
  was one observation, not evidence of an extra dig.
- A solid 0.320 m diameter by 0.320 m wide cylinder encloses 25.74 L. This is
  an outer geometric reference, not usable internal storage. Scored berm volume
  is not necessarily identical to the volume held immediately after digging:
  delivery loss, compaction and counting conventions intervene.
- For a **comparison only**, inserting UVA's estimated 0.034 m scoop protrusion
  into the reference storage geometry gives 15.96 L per drum inside the assumed
  shell. Using the reference usable-volume factor 0.90, stored-soil density
  1,250 kg/m³, berm density 1,500 kg/m³ and delivery efficiency 0.90, the two
  delivered volumes correspond to `fill_fraction` about **0.35–0.58** *if*
  storage, rather than reachable soil, is the limiting factor. The current
  reference 0.40 would correspond to 4.31 L delivered per drum under those
  same assumptions. These are model-equivalent values, not measured internal
  fill or a fit to UVA's helical drum.
- At 0.320 m diameter and Earth gravity, the observed excavation averages imply
  drum Froude numbers about 1.45–1.89; deposition averages imply about
  1.06–2.58. The reference model's provisional `max_dig_froude=0.30` and
  `max_dump_froude=0.20` correspond to about 41 and 33 rpm at this diameter.
  These historical reference levels are now diagnostics only; neither is an
  enforced RPM cap or a validated operating-domain boundary. The observations
  do not give a speed-dependent retained-mass or torque curve for our drum.
- UVA's estimated 0.034 m protrusion exceeds the current design bound of
  0.025 m. The reference scoop-opening assumption also limits its admissible
  bite to about 0.0175 m. This indicates different scoop geometry, not a
  measured error in either design.

## What would make this usable for model calibration

Confirm whether the rover was stationary, the real scoop opening/intake
geometry, start/end shaft height or trench depth, and the fill-fraction
denominator. For a retention response, measure soil mass in a drum immediately
after excavation and after the travel/dump sequence, at known RPMs and soil
conditions. For drive power or an RPM-loss response, add synchronized shaft
torque or calibrated motor current/voltage. The scored berm volumes and video
timing alone support comparison bounds, not a unique capture, lift or torque
coefficient.
