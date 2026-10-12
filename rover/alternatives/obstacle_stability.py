"""Gravity-only support calculation for future obstacle-pose models.

Ideal centered side rockers, an average-angle differential, and point contacts
on horizontal platforms. See docs/OBSTACLE_TRAVERSAL.md for assumptions/domain.
"""
import math
from ..components import scalar
from ..discipline import Discipline


def projected_support_margin(contacts, center_of_mass):
    """Minimum signed distance to the projected support hull's edges [m].

    Coordinates are world x/y/z, with z up and gravity vertical. Contacts must
    support upward vertical forces (e.g. horizontal platforms), with no other
    applied forces or moments. Noncoplanar contact heights are allowed. Positive
    means the projected CG is inside the hull; zero is marginal; negative means
    global static equilibrium is impossible under these assumptions.

    For an articulated rover this is a necessary global equilibrium screen:
    internal joint equilibrium, individual wheel loads, friction and pose
    reachability require separate checks. Contact and CG heights cannot change
    this result at fixed world x/y coordinates. Suspension geometry must first
    determine their x/y coordinates and the posed CG from the component masses.
    The negative value is an edge-normal reserve, not Euclidean hull distance.
    """
    points = [tuple(float(v) for v in point) for point in contacts]
    cg = tuple(float(v) for v in center_of_mass)
    if len(cg) != 3 or not all(math.isfinite(v) for v in cg):
        raise ValueError("Center of mass must have three finite world coordinates.")
    if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError("Contacts must have three finite world coordinates each.")
    xy = sorted({p[:2] for p in points})
    if len(xy) < 3:
        raise ValueError("At least three distinct projected support points are required.")

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

    def half_hull(sequence):
        result = []
        for p in sequence:
            while len(result) >= 2 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
        return result

    # Monotone chain gives counterclockwise boundary; removes redundant contacts.
    hull = half_hull(xy)[:-1] + half_hull(reversed(xy))[:-1]
    if len(hull) < 3:
        raise ValueError("Projected support polygon has zero area.")
    return min(cross(a, b, cg) / math.hypot(b[0]-a[0], b[1]-a[1])
               for a, b in zip(hull, hull[1:] + hull[:1]))


def raised_wheel_pose(wheelbase, track, radius, pivot_offset, height,
                      raised=0):
    """Rigid rocker geometry; order FL, FR, RL, RR; returns pose diagnostics.

    Pivot is centered longitudinally, offset above axle line by pivot_offset.
    Shaft length is track and its roll is solved simultaneously with rocker
    pitch. Average-angle differential sets chassis pitch. Point contacts are
    vertically below centers; wheel camber/contact-patch geometry is omitted.

    Outside reach, evaluate a clearly flagged guarded pose just inside reach
    for finite optimizer arithmetic. reach_margin always uses requested height.
    Its failure rejects the design; the guarded tip value is not a prediction.
    """
    if (not all(math.isfinite(v) for v in
                (wheelbase, track, radius, pivot_offset, height))
            or wheelbase <= 0 or track <= 0 or radius < 0 or height < 0
            or abs(pivot_offset) >= wheelbase/2 or raised not in range(4)):
        raise ValueError("Invalid rocker geometry, height, or raised-wheel index.")
    # Above-axis pivots reach maximum height at theta=pi/2. Below-axis
    # pivots reach a maximum earlier; invert the first, increasing branch.
    def height_at(theta):
        q = wheelbase/2*math.sin(theta)+pivot_offset*(math.cos(theta)-1)
        return wheelbase*math.sin(theta)*track/math.hypot(track,q)

    limit = math.pi/2
    if pivot_offset < 0:
        lo,hi = 0.0,limit
        for _ in range(50):
            t = (lo+hi)/2
            q = wheelbase/2*math.sin(t)+pivot_offset*(math.cos(t)-1)
            derivative_sign = track*track*math.cos(t)+pivot_offset*q*(1-math.cos(t))
            if derivative_sign > 0:
                lo = t
            else:
                hi = t
        limit = (lo+hi)/2
    maximum = height_at(limit)
    margin = maximum-height
    used = height if margin > 0 else maximum*(1-1e-6)
    if used == 0:
        roll = theta = 0.0
    elif pivot_offset < 0:
        lo,hi = 0.0,limit
        for _ in range(50):
            t = (lo+hi)/2
            if height_at(t) > used:
                hi = t
            else:
                lo = t
        theta = (lo+hi)/2
        q = wheelbase/2*math.sin(theta)+pivot_offset*(math.cos(theta)-1)
        roll = math.atan2(q,track)
    else:
        ratio = used/wheelbase
        lo, hi = 0.0, math.acos(ratio)
        # T sin(phi) = h/2 + d cos(phi)(cos(theta)-1)
        # cos(phi) sin(theta) = h/L; solve on forward-contact branch.
        for _ in range(50):
            mid = (lo+hi)/2
            c = math.cos(mid)
            f = track*math.sin(mid)-used/2-pivot_offset*(
                math.sqrt(max(0.0, c*c-ratio*ratio))-c)
            if f > 0:
                hi = mid
            else:
                lo = mid
        roll = (lo+hi)/2
        theta = math.asin(min(1.0, ratio/math.cos(roll)))
    sx, sy = ((1,1), (1,-1), (-1,1), (-1,-1))[raised]
    origin_z = radius+used/4+pivot_offset*math.cos(roll)*(1+math.cos(theta))/2
    roll *= sy
    theta *= sx
    pitch = theta/2
    centers = []
    for wx, wy in ((1,1), (1,-1), (-1,1), (-1,-1)):
        t = theta if wy == sy else 0.0
        c, s = math.cos(t), math.sin(t)
        cr, sr = math.cos(roll), math.sin(roll)
        centers.append((wx*wheelbase*c/2+pivot_offset*s,
                        wy*track*cr/2-wx*wheelbase*sr*s/2+pivot_offset*sr*c,
                        origin_z+wy*track*sr/2+wx*wheelbase*cr*s/2-pivot_offset*cr*c))
    return {"centers": centers, "contacts": [(x,y,z-radius) for x,y,z in centers],
            "shaft_origin": (0.0,0.0,origin_z), "pitch": pitch, "roll": roll,
            "reach_margin": margin, "valid": margin > 0, "used_height": used}


def posed_loaded_cg(pose, radius, pivot_offset, sprung_mass, sprung_x,
                    sprung_z, unsprung_mass):
    """Conserve mass; transform mounted hardware, retain wheel/rocker centers.

    Sprung CG coordinates refer to original level ground. Equal wheel/drive
    masses and two uniform rockers have aggregate CG at mean wheel centers.
    """
    if (not all(math.isfinite(v) for v in (radius,pivot_offset,sprung_mass,
                                         sprung_x,sprung_z,unsprung_mass))
            or sprung_mass <= 0 or unsprung_mass < 0):
        raise ValueError("Finite positive mounted mass and nonnegative suspension mass required.")
    a, r = pose["pitch"], pose["roll"]
    z = sprung_z-radius-pivot_offset
    xp = sprung_x*math.cos(a)-z*math.sin(a)
    zp = sprung_x*math.sin(a)+z*math.cos(a)
    body = (xp, -math.sin(r)*zp, pose["shaft_origin"][2]+math.cos(r)*zp)
    suspension = tuple(sum(p[i] for p in pose["centers"])/4 for i in range(3))
    total = sprung_mass+unsprung_mass
    return tuple((sprung_mass*b+unsprung_mass*u)/total
                 for b,u in zip(body,suspension))


class ObstacleStability(Discipline):
    CONFIG_INPUTS = ("wheelbase", "track_width", "wheel_diameter",
                     "rocker_pivot_offset", "obstacle_height")
    COUPLED_INPUTS = {"loaded_sprung_mass": "kg", "loaded_sprung_cg_x": "m",
                      "loaded_sprung_cg_z": "m", "wheel_mass": "kg",
                      "drive_mass": "kg", "rocker_mass": "kg"}
    OUTPUT_UNITS = {"obstacle_pose_margin": "m", "obstacle_tip_margin": "m",
                    "obstacle_pose_valid": None, "obstacle_evaluated_height": "m",
                    "obstacle_worst_wheel": None, "obstacle_chassis_pitch": "rad",
                    "obstacle_chassis_roll": "rad", "obstacle_loaded_cg_x": "m",
                    "obstacle_loaded_cg_y": "m", "obstacle_loaded_cg_z": "m"}
    pose_inputs = {"wheelbase", "track_width", "rocker_pivot_offset", "obstacle_height"}
    all_inputs = set(CONFIG_INPUTS) | set(COUPLED_INPUTS)
    DEPENDENCIES = dict.fromkeys(OUTPUT_UNITS, all_inputs)
    for n in ("obstacle_pose_margin", "obstacle_pose_valid", "obstacle_evaluated_height"):
        DEPENDENCIES[n] = pose_inputs
    model_evidence = {"status": "analytical, uncalibrated gravity support screen",
                      "loading": "all four drums loaded; fixed travel arm position",
                      "suspension": "centered pivots; average-angle differential",
                      "pivot_offset": "user estimate +0.10 m; editable, not frozen",
                      "guard": "negative pose margin rejects unreachable poses; tip diagnostics then use guarded height",
                      "ditch": "0.50 m deep, circle-like; width/radius unknown; unverified",
                      "omissions": "joint/load equilibrium, wheel camber, stops, traction, climb path, collision, terrain and dynamics"}

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        cases = []
        for wheel in range(4):
            pose = raised_wheel_pose(v("wheelbase"),v("track_width"),v("wheel_diameter")/2,
                                     v("rocker_pivot_offset"),v("obstacle_height"),wheel)
            cg = posed_loaded_cg(pose,v("wheel_diameter")/2,v("rocker_pivot_offset"),
                                 v("loaded_sprung_mass"),v("loaded_sprung_cg_x"),v("loaded_sprung_cg_z"),
                                 v("wheel_mass")+v("drive_mass")+v("rocker_mass"))
            cases.append((projected_support_margin(pose["contacts"],cg),wheel,pose,cg))
        margin, wheel, pose, cg = min(cases, key=lambda case: case[0])
        outputs["obstacle_tip_margin"] = margin
        outputs["obstacle_pose_margin"] = pose["reach_margin"]
        outputs["obstacle_pose_valid"] = float(pose["valid"])
        outputs["obstacle_evaluated_height"] = pose["used_height"]
        outputs["obstacle_worst_wheel"] = wheel
        outputs["obstacle_chassis_pitch"], outputs["obstacle_chassis_roll"] = pose["pitch"],pose["roll"]
        for axis,value in zip("xyz",cg):
            outputs["obstacle_loaded_cg_"+axis] = value
