# Optional attachment beyond the drum shaft (revision 1)

`actuator_attachment_fraction = f` measures B-to-pin distance divided by
B-to-drum-shaft distance `arm_length = L`. The default maximum remains 1.
Select `--attachment-max 1.5 --optimize-attachment` with the plate-frame box-arm
profile to explore pins beyond the drum shaft. This selects coordinated
geometry, gravity-load, beam and mass-properties factories through ModelSet.
It does not alter the drum, route, soil or score model. Values above the
selected bound are rejected. The supported maximum is 1.5.

## Assumptions and equations

The same uniform aluminum box tube continues from B through the drum shaft C
to pin D. Physical length is `ell = max(1,f)*L`. Two arms add a total
`2*arm_linear_mass*(ell-L)` kg to linkage_mass; GeometryMass sums that once.
Diagnostics `arm_physical_length` (m, per arm) and `arm_extension_mass` (kg,
both tails combined) are present only with the extended model selected.
All existing port meanings, per-pair loading and margin signs are preserved.

The uniform tube gravity moment about B is `mu*g*ell^2*cos(theta)/2`.
Thus linkage force and lifting work use the equivalent weight
`mu*g*ell^2/(2*L)` at C. Structural mount equilibrium instead uses actual
full tube weight at ell/2. The beam screen includes the drum-pair point load
at L, actuator load at ell and full-length uniform weight; it finds moment
and deflection extrema on both segments. Axial and bending stress maxima
are conservatively combined. Euler screening uses the whole physical length
with the existing conservative effective-length assumption.

Added tail mass is placed at its actual midpoint `(C+D)/2`, mirrored front
and rear. Local rod yaw inertia and parallel-axis terms are included in all
three loading states and the sprung-mass calculation. The original core
linkage mass-placement approximation remains in use; this is not a complete
CAD inertia model. Extra mass affects mobility, stability and mission scores.

Stowed and deployed extents include the tail and a conservative box-section
circumradius. The travel-to-dig tail arc is screened at analytical coordinate
extrema for deployed envelope and pit floor. Transport clearance includes
the stowed tail. Original drum/wheel and obstacle checks remain active.

## Evidence and limits

No fitted relationship, calibration dataset or physical validation is added.
Independent horizontal overhang equilibrium/beam limits, gravity work,
mass/yaw-inertia accounting, pin closure, invalid inputs, reference equivalence
for f <= 1 and smooth derivatives are in `tests/test_extended_attachment.py`.
Analytical agreement is numerical verification, not experimental validation.

Brackets, shaft connections, joint stiffness, actuator housing interference,
arm/wheel collisions, ground contact during obstacle crossing and the actual
soil interaction of a tail extending beyond the digging drum remain unmodeled.
The pit-floor screen does not prove the tail passes through excavated soil.
An extended design needs CAD and digging tests before hardware selection.

Reproduce the multistart search using `python -m studies.linkage_multistart
--attachment-max 1.5 --reference PATH_TO_PREVIOUS_ANALYSIS.json --output
results/attachment_extension_study`. It preserves wheel dimensions, frees
actuator lengths, sets climb angles to zero and retains the explicitly
requested turn-traction exclusion. Each report continues to show excluded
physical failures. The manifest records seeds, versions and source hashes.
