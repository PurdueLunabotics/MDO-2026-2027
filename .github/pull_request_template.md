## Model question and resulting behavior

<!-- State the physical question and what changes at the same rover design. -->

## Evidence and domain

<!-- Link the model card, data manifest/CAD or test evidence, validity range,
     and held-out validation. Label synthetic data and uncalibrated assumptions. -->

## Interface and constraints

<!-- Name the ModelSet slot(s), any changed ports/units, and affected constraints.
     Explain any shared contract change before removing or redefining a port. -->

## Verification

- [ ] Reference and replacement compared at the same design
- [ ] Relevant physical and invalid-domain tests added
- [ ] `python -m unittest discover -s tests -v`
- [ ] `python main.py --baseline --output results/my_change_baseline.json`
- [ ] `python main.py --check-derivatives --output results/my_change_optimization.json`
- [ ] Remaining uncertainty and failed cases stated
