"""Axle-relative actuator mount: side along arm, top along its upward normal.

All lengths are metres. Uniform tube continues to the pin axial station when
side > 0; the rigid normal bracket has unknown mass/strength, omitted explicitly.
No calibration or CAD collision validation is implied. See ARM_OFFSETS.md.
"""
import math
import numpy as np


def pin_radius(length, side, top):
    if (not all(math.isfinite(x) for x in (length, side, top))
            or length <= 0 or length+side <= 0):
        raise ValueError("Finite offsets and a pin axial station beyond B are required")
    return math.hypot(length+side, top)


def physical_length(length, side):
    pin_radius(length, side, 0.)
    return length+max(side, 0.)


def attachment_point(bottom_x, bottom_z, drum_x, drum_z, side=0., top=0.):
    """D=C+side*u+top*(-u_z,u_x), vectorized for a sweep of drum positions."""
    dx, dz = drum_x-bottom_x, drum_z-bottom_z
    length = np.hypot(dx, dz)
    if (np.any(~np.isfinite(length)) or np.any(length <= 0)
            or not all(math.isfinite(x) for x in (side, top))
            or np.any(length+side <= 0)):
        raise ValueError("Invalid arm or actuator offsets")
    return drum_x+(side*dx-top*dz)/length, drum_z+(side*dz+top*dx)/length


def offset_mount_loads(bottom, upper, drum, pair_weight, arm_weight, side, top):
    """Frame loads and force ON arm at pin; exact rigid-body gravity balance."""
    b, a, c = (np.asarray(p, dtype=float) for p in (bottom, upper, drum))
    if (any(p.shape != (2,) or not np.isfinite(p).all() for p in (a,b,c))
            or not all(math.isfinite(x) and x >= 0 for x in (pair_weight, arm_weight))):
        raise ValueError("Invalid arm geometry or weights")
    r = c-b
    length = float(np.linalg.norm(r))
    ell = physical_length(length, side)
    if top == 0. and side <= 0.:
        from .plate_frame import arm_mount_loads
        return arm_mount_loads(b,a,c,pair_weight,arm_weight,1+side/length)
    d = np.asarray(attachment_point(*b,*c,side,top))
    direction = a-d
    distance = np.linalg.norm(direction)
    if distance <= 1e-9: raise ValueError("Coincident actuator joints")
    u = direction/distance
    lever = (d[0]-b[0])*u[1]-(d[1]-b[1])*u[0]
    if abs(lever) <= 1e-9: raise ValueError("Actuator toggle cannot balance gravity")
    actuator = u*((pair_weight+arm_weight*ell/(2*length))*r[0]/lever)
    return -actuator, actuator+np.array([0.,-pair_weight-arm_weight]), actuator


def offset_beam_screen(length, side, top, direction, actuator, pair_weight,
                       arm_weight, area, inertia, depth, modulus):
    """Tube bending, relative deflection and compression including eccentric force.

    Transfer off-axis force to its tube station with couple -top*F_parallel.
    Evaluate both sides of this moment jump, zero-shear moment extrema and all
    real zero-slope displacement extrema. Bracket itself is not sized here.
    """
    ell = physical_length(length, side)
    if (not all(math.isfinite(x) and x > 0 for x in (area,inertia,depth,modulus))
            or not all(math.isfinite(x) and x >= 0 for x in (pair_weight,arm_weight))):
        raise ValueError("Invalid beam section or weights")
    e = np.asarray(direction,dtype=float)/length
    actuator = np.asarray(actuator,dtype=float)
    if e.shape != (2,) or actuator.shape != (2,) or not np.isfinite(e).all() or not np.isfinite(actuator).all():
        raise ValueError("Invalid beam directions/forces")
    n = np.array([-e[1],e[0]])
    fc, fa = -pair_weight*e[0], float(actuator@n)
    axial_pin = float(actuator@e)
    couple = -top*axial_pin
    q = -arm_weight*e[0]/ell
    station = length+side
    points = [(length,fc),(station,fa)]
    boundaries = sorted(set((0.,length,station,ell)))
    stations = list(boundaries)
    for lo,hi in zip(boundaries,boundaries[1:]):
        active = sum(force for x,force in points if x>(lo+hi)/2)
        if abs(q)>1e-12:
            root = ell+active/q
            if lo<root<hi: stations.append(root)
    moments, axial = [], []
    for s in stations:
        for left in (True,False):
            active = lambda x: x>s or (x==s and left)
            moments.append(sum(force*max(x-s,0.) for x,force in points)
                           +q*(ell-s)**2/2+(couple if active(station) else 0.))
            axial.append(-arm_weight*(1-s/ell)*e[1]
                         +(-pair_weight*e[1] if active(length) else 0.)
                         +(axial_pin if active(station) else 0.))
    def displacement(s):
        point = sum(force*(s*s*(3*x-s) if s<=x else x*x*(3*s-x))/6 for x,force in points)
        moment = couple*(s*s if s<=station else station*(2*s-station))/2
        return (point+moment+q*s*s*(6*ell*ell-4*ell*s+s*s)/24)/(modulus*inertia)
    extrema = list(boundaries)
    for lo,hi in zip(boundaries,boundaries[1:]):
        mid = (lo+hi)/2
        coeff = np.array([0.,q*ell*ell/2,-q*ell/2,q/6])
        for x,force in points:
            if mid<x: coeff[1]+=force*x; coeff[2]-=force/2
            else: coeff[0]+=force*x*x/2
        if mid<station: coeff[1]+=couple
        else: coeff[0]+=couple*station
        if np.any(coeff):
            for root in np.polynomial.polynomial.polyroots(np.trim_zeros(coeff,'b')):
                if abs(root.imag)<1e-9 and lo<root.real<hi: extrema.append(float(root.real))
    moment = max(abs(x) for x in moments)
    return (max(abs(x) for x in axial)/area+moment*depth/(2*inertia),
            max(abs(displacement(s)) for s in extrema), moment, max(0.,-min(axial)))


class OffsetArmMechanics:
    """Explicit pure-helper strategy for the selected frame component."""
    physical_length = staticmethod(physical_length)
    mount_loads = staticmethod(offset_mount_loads)
    beam_screen = staticmethod(offset_beam_screen)
