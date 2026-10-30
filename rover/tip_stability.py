"""Level-ground static tip screen at the prescribed asymmetric dump state.

One front/rear drum pair is full and the other is empty. Both pairs are at
their fully extended linkage positions. The reverse loading is symmetric.
Other half-loaded arm positions require a software interlock; this component
does not claim to validate that controller or dynamic dumping.
"""
from .components import scalar
from .discipline import Discipline, names


def extended_pair_tip_margin(dry_mass, payload_mass, wheelbase, track_width,
                             extended_x):
    """Distance from the asymmetric-load CG to the closest support edge [m].

    The full pair holds half of the total four-drum payload. Equal dry drum and
    linkage masses at mirrored extended positions cancel longitudinally, and
    paired drums cancel laterally. Reversing which pair is full changes only the
    sign of the longitudinal moment.
    """
    full_pair_payload = payload_mass / 2
    total_mass = dry_mass + full_pair_payload
    cg_x = full_pair_payload * extended_x / total_mass
    return min(wheelbase / 2 - abs(cg_x), track_width / 2)


class TipStability(Discipline):
    CONFIG_INPUTS = tuple(sorted(names("wheelbase track_width")))
    COUPLED_INPUTS = {
        "dry_mass": "kg", "payload_mass": "kg", "extended_x": "m",
    }
    # Retain the existing diagnostic output contract. Mask 3 denotes the
    # front pair full; the equivalent rear-pair-full case is mask 12.
    OUTPUT_UNITS = {"tip_stability_margin": "m", "tip_worst_mask": None}
    DEPENDENCIES = {
        "tip_stability_margin": set(CONFIG_INPUTS) | set(COUPLED_INPUTS),
        "tip_worst_mask": set(),
    }

    def compute(self, inputs, outputs):
        v = lambda name: scalar(inputs, name)
        outputs["tip_stability_margin"] = extended_pair_tip_margin(
            v("dry_mass"), v("payload_mass"), v("wheelbase"),
            v("track_width"), v("extended_x"))
        outputs["tip_worst_mask"] = 3


class DepositionTipStability(Discipline):
    """Static asymmetric support screen at the commanded deposition pose.

    Half-loaded transitions to other arm poses require an interlock. Height
    enters the dynamic turn screen, not this level-ground gravity projection.
    """
    CONFIG_INPUTS = ("wheelbase", "track_width")
    COUPLED_INPUTS = {"deposition_half_cg_x": "m"}
    OUTPUT_UNITS = TipStability.OUTPUT_UNITS
    DEPENDENCIES = {"tip_stability_margin": set(CONFIG_INPUTS) | set(COUPLED_INPUTS),
                    "tip_worst_mask": set()}

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        outputs["tip_stability_margin"] = min(v("wheelbase")/2-abs(v("deposition_half_cg_x")),
                                               v("track_width")/2)
        outputs["tip_worst_mask"] = 3
