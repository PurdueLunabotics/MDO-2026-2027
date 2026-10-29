"""Opt-in uniform box-arm continuation beyond the drum shaft, f > 1.

The B-to-drum length and all existing ports retain their meanings. A physical
tail is added through the pin; it has the same tube section/material as the arm.
No bracket/CAD or soil-contact validation is claimed. See EXTENDED_ATTACHMENT.md.
"""
import math
import numpy as np

from ..components import scalar
from ..linkage import LinkageGeometry, LinkageLoads, sweep_poses, attachment_point
from .plate_frame import PlateMassProperties, arm_mount_loads, arm_beam_screen
from .aluminum_box_arm import BoxArmPlateStructureLoads

EXTRA_OUTPUTS = {"arm_physical_length": "m", "arm_extension_mass": "kg"}


def physical_length(length, fraction):
    if not all(math.isfinite(x) and x > 0 for x in (length, fraction)):
        raise ValueError("Arm length and attachment fraction must be finite and positive")
    return length*max(1., fraction)


def extended_mount_loads(bottom, top, drum, pair_weight, arm_weight, fraction):
    if fraction <= 1:
        return arm_mount_loads(bottom, top, drum, pair_weight, arm_weight, fraction)
    b,a,c = (np.asarray(p,dtype=float) for p in (bottom,top,drum))
    if any(p.shape != (2,) or not np.isfinite(p).all() for p in (a,b,c)) or not all(math.isfinite(x) for x in (pair_weight,arm_weight,fraction)) or min(pair_weight,arm_weight)<0:
        raise ValueError("Invalid extended-arm loads or geometry")
    r=c-b; direction=a-(b+fraction*r); distance=np.linalg.norm(direction)
    if distance <= 1e-9 or np.linalg.norm(r) <= 1e-9:
        raise ValueError("Coincident extended-arm joints")
    u=direction/distance;lever=fraction*(r[0]*u[1]-r[1]*u[0])
    if abs(lever)<=1e-9:
        raise ValueError("Actuator toggle cannot support the extended arm")
    actuator=u*((pair_weight+arm_weight*fraction/2)*r[0]/lever)
    return -actuator,actuator+np.array([0.,-pair_weight-arm_weight]),actuator


def extended_beam_screen(length,fraction,direction,actuator,pair_weight,arm_weight,area,inertia,depth,modulus):
    if fraction <= 1:
        return arm_beam_screen(length,fraction,direction,actuator,pair_weight,arm_weight,area,inertia,depth,modulus)
    ell=physical_length(length,fraction)
    if not all(math.isfinite(x) and x>0 for x in (area,inertia,depth,modulus)) or min(pair_weight,arm_weight)<0:
        raise ValueError("Invalid extended-arm beam section or loading")
    e=np.asarray(direction,dtype=float)/length;n=np.array([-e[1],e[0]])
    if not np.isfinite(e).all() or not np.isfinite(actuator).all():
        raise ValueError("Nonfinite extended-arm beam forces")
    fc=float(np.array([0.,-pair_weight])@n);fa=float(np.asarray(actuator)@n)
    q=float(np.array([0.,-arm_weight/ell])@n)
    points=[(length,fc),(ell,fa)]
    boundaries=[0.,length,ell];stations=list(boundaries)
    for lo,hi in zip(boundaries,boundaries[1:]):
        active=sum(force for x,force in points if x>(lo+hi)/2)
        if abs(q)>1e-12:
            root=ell+active/q
            if lo<root<hi:stations.append(root)
    moments=[sum(force*max(x-s,0.) for x,force in points)+q*(ell-s)**2/2 for s in stations]
    axial=[]
    for s in boundaries:
        for side in (-1,1):
            axial.append(-arm_weight*(1-s/ell)*e[1]
                         +(-pair_weight*e[1] if s<length or (s==length and side<0) else 0.)
                         +(float(np.asarray(actuator)@e) if s<ell or side<0 else 0.))
    # Conservative combined envelope: maxima may be at different stations.
    stress=max(abs(x) for x in axial)/area+max(abs(x) for x in moments)*depth/(2*inertia)
    def displacement(s):
        point=sum(force*(s*s*(3*x-s) if s<=x else x*x*(3*s-x))/6 for x,force in points)
        return (point+q*s*s*(6*ell*ell-4*ell*s+s*s)/24)/(modulus*inertia)
    extrema=list(boundaries)
    for lo,hi in zip(boundaries,boundaries[1:]):
        coeff=np.array([0.,q*ell*ell/2,-q*ell/2,q/6])
        for x,force in points:
            if (lo+hi)/2<x:
                coeff[1]+=force*x;coeff[2]-=force/2
            else:coeff[0]+=force*x*x/2
        if np.any(coeff):
            for root in np.polynomial.polynomial.polyroots(np.trim_zeros(coeff,'b')):
                if abs(root.imag)<1e-9 and lo<root.real<hi:extrema.append(float(root.real))
    return stress,max(abs(displacement(s)) for s in extrema),max(abs(x) for x in moments),max(0.,-min(axial))


class ExtendedArmMechanics:
    physical_length=staticmethod(physical_length)
    mount_loads=staticmethod(extended_mount_loads)
    beam_screen=staticmethod(extended_beam_screen)


class ExtendedGeometryMixin:
    """Compatibility class: axial continuation is now included in base geometry."""
    pass


class ExtendedLoadsMixin:
    """Compatibility class: base loads include actual physical arm gravity."""
    pass


class ExtendedLinkageGeometry(LinkageGeometry):
    pass


class ExtendedLinkageLoads(LinkageLoads):
    pass


class ExtendedPlateStructureLoads(BoxArmPlateStructureLoads):
    pass


class ExtendedPlateMassProperties(PlateMassProperties):
    pass
