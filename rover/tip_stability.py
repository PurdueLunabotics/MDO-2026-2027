"""Level-ground support screens for asymmetric drum loading."""
from .components import scalar
from .discipline import Discipline, names
from .mobility import rolling_resistance_force, wheel_contact_area


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
    """Legacy gravity-only screen with both drum pairs fully extended."""
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


def deposition_tip_margin(v):
    """Signed support reserve [m] with peak drive reaction on each rocker.

    Use the largest wheel torque from the point turn and the two straight moves.
    Opposing drive reactions in the turn unload opposite wheels. Forward and
    reverse also cover both pitch signs. This screen assumes half the rover
    weight on each side. Mobility separately checks CG load transfer.
    """
    mass = v("dry_mass") + v("payload_mass") / 2
    weight = mass * v("gravity")
    half_track = v("track_width") / 2
    wheelbase = v("wheelbase")
    radius = v("wheel_diameter") / 2
    area = wheel_contact_area(v("wheel_diameter"), v("wheel_width"),
                              v("wheel_deflection_fraction"))
    rolling_force = rolling_resistance_force(weight, area, v("rocker_load_factor"),
                                             v("rolling_base"), v("rolling_pressure_scale"))
    scrub_moment = v("scrub_coefficient") * weight * wheelbase / 4
    acceleration = v("acceleration") * v("turn_accel_fraction")
    yaw_acceleration = acceleration / half_track
    yaw_moment = rolling_force * half_track + scrub_moment + v("deposition_half_yaw_inertia") * yaw_acceleration
    turn_force = yaw_moment / half_track
    straight_force = rolling_force + mass * acceleration
    wheel_torque = max(turn_force, straight_force) * radius / 4  # N m per wheel shaft

    # Worst-side pitch moment is two shaft reactions acting with the CG offset.
    side_weight = weight / 2
    pitch_reserve = wheelbase / 2 - abs(v("deposition_half_cg_x")) - 2 * wheel_torque / side_weight
    return min(pitch_reserve, half_track)


class DepositionTipStability(Discipline):
    """Support screen for the half-loaded maneuver at the commanded deposition pose."""

    CONFIG_INPUTS = tuple(sorted(names(
        "wheelbase track_width wheel_diameter wheel_width wheel_deflection_fraction "
        "rocker_load_factor gravity rolling_base rolling_pressure_scale scrub_coefficient "
        "acceleration turn_accel_fraction")))
    COUPLED_INPUTS = {
        "dry_mass": "kg", "payload_mass": "kg", "deposition_half_cg_x": "m",
        "deposition_half_yaw_inertia": "kg*m**2",
    }
    OUTPUT_UNITS = TipStability.OUTPUT_UNITS
    DEPENDENCIES = {"tip_stability_margin": set(CONFIG_INPUTS) | set(COUPLED_INPUTS),
                    "tip_worst_mask": set()}

    def compute(self, inputs, outputs):
        outputs["tip_stability_margin"] = deposition_tip_margin(lambda name: scalar(inputs, name))
        outputs["tip_worst_mask"] = 3
