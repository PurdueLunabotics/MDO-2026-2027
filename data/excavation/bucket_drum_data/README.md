# Bucket drum published points - v1

This is a source-labelled literature catalogue, not a force model or calibration.
No interpolation, regression, synthetic response points or simulator runs were added.
The rover code, saved baseline and existing IPEx surface are unchanged.

Open `index.html` for graphs and a searchable point table, or use `points.csv`.
Each plotted point has a stable `point_id` which joins to a source and page/figure.
`manifest.json` records source/image hashes and coordinate calibrations.

## What is included

- NASA (2022): 28 recoverable Figure 13 force marker centres, 12 paired Figure
  14/15 performance conditions, and 3 Table 3 average collected masses.
- Kafi (2026), arXiv v1: all 9 Table II conditions, with the values as printed.
- Giel (2025), arXiv v1: 14 Figure 4 trial markers, 4 Table II trials and 2 author
  summary/estimate records. The summaries are not additional experiments.
- Galati (2026) and Li (2024) are indexed as pending numerical extraction.
  Accessible abstracts/previews do not supply their response tables. No values
  are inferred from accuracy metrics, another paper or assumed model equations.

## Units and conventions

CSV names carry units. SI values are used where the source unit is established.
`specific_energy_reported_value`/`specific_energy_reported_unit` also preserve
the printed native energy units. Blank cells mean unavailable/unreported, not zero.
Kafi's feed is explicitly 0 m/s because its methods describe fixed-position tests.
NASA's force is horizontal force per whole drum width (N/m = printed mN/mm).
Drum size and width cannot be assigned to the Figure 13 markers, so no absolute
force is calculated. Shaft torque is never converted into horizontal force.
NASA/Kafi outcomes are per single tool; Giel outcomes are for its two-drum tool.

## Reading quality and source conflicts

NASA Figure 13: this is an incomplete visible subset. Merged or hidden marks do
not recover individual run identities or repeated coincident observations. The
paper reports 48 trials; the catalogue does not reconstruct those trials. Hollow
points have distinct visible cores within overlapping groups. Never use these
28 locations to estimate the complete sample mean, scatter or uncertainty.
The earlier eight trend-line readings are not included in this dataset.

NASA depth labels: Figure 13-15 labels include 12.7, 16 and 25 mm. Tables 1-2
instead imply 10.56, 13.8 and 19.12 mm for the 40%-scoop-height cuts. Figure labels
are retained and flagged; they are not silently corrected.

NASA energy: Figure 15 prints Wh/kg, while the text describes mechanical energy
from torque and RPM. The printed unit is not confirmed and is not converted
or pooled with other energy data. Its apparent magnitude merits author checking.

Kafi: Table II is the transcription authority. Its caption gives 48/24/16 s for
four revolutions at 5/10/15 RPM, whereas the methods give 48/24/12 s. The CSV
duration is calculated as 4*60/RPM and labelled with a source conflict. Reported
rates differ from reported mass/duration. K-A15 gives E=56.59 J and m=1.424 kg,
so E/m=39.74 J/kg, versus 80.59 J/kg printed. Mean torque times four revolutions
also conflicts materially with all three A energies. Both printed and arithmetic
check values are preserved; none is silently substituted. Table I's diameter is
labelled inner diameter and should not be treated as cutting-tip diameter.

Giel: the native Figure 4 marker centres supply mass/time only. Derived per-trial
rates are mass/time, not samples from the paper's regression. The 27/72 mm/s
feeds are configuration averages, not exact trial feeds. Other configurations'
feeds, depths and normal loads remain unknown. Tool-only electrical summary
energy and the full-system batch estimate have different accounting boundaries.

Digitization resolution (coordinate reading, not experimental uncertainty):
NASA force about 1 N/m; NASA rate about 0.001 kg/s; NASA printed energy about
1 printed unit; Giel mass about 0.03 kg and time about 0.2 s. No statistical
error bars are invented. Published condition means are not raw replicate data.

## Interpretation

Only NASA currently supplies recovered horizontal-force response points.
Kafi can help inspect RPM/torque trends within its own design/soil/test method.
Giel adds low-RPM travel, collection and energy context. These sources do not
form one exchangeable force surface. Galati/Li full text and author data remain
the next gaps. An experimental validation claim would require independent data.

## Source links

- NASA22: [ISRU Pilot Excavator: Bucket drum scaling experimental results](https://ntrs.nasa.gov/citations/20210025846)
- GAL26: [Physics-Based Torque Prediction Model for Excavating Drums on Granular Soil](https://doi.org/10.1002/rob.70146)
- LI24: [Performance analysis of a bucket drum for lunar regolith collection in lunar base construction](https://doi.org/10.1016/j.autcon.2024.105617)
- KAF26: [Designing an Efficient Excavator Bucket for Lunar ISRU: A Comparative Study with Vision-Based Fill and Displacement Analysis](https://arxiv.org/html/2609.25724v1)
- GIE25: [Design and Development of a Modular Bucket Drum Excavator for Lunar ISRU](https://arxiv.org/html/2511.00492v1)
