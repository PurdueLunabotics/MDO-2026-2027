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


class ExtendedLinkageGeometry(LinkageGeometry):
    OUTPUT_UNITS={**LinkageGeometry.OUTPUT_UNITS,**EXTRA_OUTPUTS}
    DEPENDENCIES={}

    def setup(self):
        super().setup()
        self.inputs_from_config("arm_box_width","arm_box_height")
        names=set(self._var_rel_names['input'])
        self.DEPENDENCIES={n:set(names) for n in self.OUTPUT_UNITS}

    def compute(self,inputs,outputs):
        super().compute(inputs,outputs)
        v=lambda n:scalar(inputs,n)
        out=lambda n:float(np.asarray(outputs[n]).item())
        ell=physical_length(v('arm_length'),v('actuator_attachment_fraction'))
        extra=2*v('arm_linear_mass')*(ell-v('arm_length'))
        outputs['arm_physical_length']=ell;outputs['arm_extension_mass']=extra
        if v('actuator_attachment_fraction')<=1:return
        outputs['linkage_mass']+=extra
        poses,angles=sweep_poses({n:scalar(inputs,n) for n in inputs})
        bx,bz=v('pivot_x'),v('bottom_pivot_height');f=v('actuator_attachment_fraction')
        radius=math.hypot(v('arm_box_width'),v('arm_box_height'))/2
        sx,sz=attachment_point(bx,bz,out('stowed_x'),out('stowed_y'),f)
        critical=[float(angles[0]),float(angles[-1])]
        lo,hi=sorted(critical)
        for angle in (k*math.pi/2 for k in range(-4,5)):
            if lo<=angle<=hi:critical.append(angle)
        tips=[(bx+ell*math.cos(a),bz+ell*math.sin(a)) for a in critical]
        outputs['stowed_length']=max(out('stowed_length'),2*(abs(sx)+radius))
        outputs['stowed_height']=max(out('stowed_height'),sz+radius)
        outputs['deployed_length']=max(out('deployed_length'),2*(max(abs(x) for x,z in tips)+radius))
        outputs['deployed_height']=max(out('deployed_height'),max(z for x,z in tips)+radius)
        outputs['deployed_height_margin']=2.5-out('deployed_height')
        outputs['transport_clearance_margin']=min(out('transport_clearance_margin'),sz-radius-v('transport_clearance'))
        outputs['pit_depth_margin']=min(out('pit_depth_margin'),min(z for x,z in tips)-radius+v('pit_depth'))


class ExtendedLinkageLoads(LinkageLoads):
    DEPENDENCIES={}

    def setup(self):
        super().setup();self.inputs_from_config('actuator_attachment_fraction')
        self.DEPENDENCIES={n:set(self._var_rel_names['input']) for n in self.OUTPUT_UNITS}

    def compute(self,inputs,outputs):
        modified={n:inputs[n].copy() for n in inputs}
        scale=max(1.,scalar(inputs,'actuator_attachment_fraction'))
        modified['arm_linear_mass']*=scale*scale
        super().compute(modified,outputs)


class ExtendedPlateStructureLoads(BoxArmPlateStructureLoads):
    ARM_MECHANICS=ExtendedArmMechanics
    model_evidence={**BoxArmPlateStructureLoads.model_evidence,'attachment_extension_revision':1,
        'arm_extension':'uniform continuation of the same box tube beyond drum shaft; gravity-only loads; entire-length bending/deflection and conservative Euler screen; no bracket or soil-contact validation'}


class ExtendedPlateMassProperties(PlateMassProperties):
    CONFIG_INPUTS=tuple(sorted(set(PlateMassProperties.CONFIG_INPUTS)|{'arm_length'}))
    COUPLED_INPUTS={**PlateMassProperties.COUPLED_INPUTS,'arm_linear_mass':'kg/m'}
    DEPENDENCIES=dict.fromkeys(PlateMassProperties.OUTPUT_UNITS,set(CONFIG_INPUTS)|set(COUPLED_INPUTS))
    model_evidence={**PlateMassProperties.model_evidence,'attachment_extension_revision':1,
        'placement':'Reference core linkage approximation plus actual uniform overhang centroids and yaw inertias; two tails added exactly once'}

    def compute(self,inputs,outputs):
        v=lambda n:scalar(inputs,n)
        extra_length=physical_length(v('arm_length'),v('actuator_attachment_fraction'))-v('arm_length')
        if extra_length==0:
            return super().compute(inputs,outputs)
        each=v('arm_linear_mass')*extra_length;extra=2*each
        modified={n:inputs[n].copy() for n in inputs};modified['linkage_mass']-=extra
        super().compute(modified,outputs)
        ratio=extra_length/v('arm_length')
        x=v('travel_x')+ratio*(v('travel_x')-v('pivot_x'))/2
        z=v('travel_y')+ratio*(v('travel_y')-v('bottom_pivot_height'))/2
        local=each*(ratio*(v('travel_x')-v('pivot_x')))**2/12
        core=sum(v(n) for n in ('wheel_mass','drive_mass','drum_mass','battery_mass','structure_mass','fixed_mass','linkage_mass'))-extra
        out=lambda n:float(np.asarray(outputs[n]).item())
        for state,soil in (('empty',0.),('loaded',v('payload_mass')),('half',v('payload_mass')/2)):
            mass=core+soil
            outputs[state+'_cg_x']=out(state+'_cg_x')*mass/(mass+extra)
            outputs[state+'_cg_z']=(out(state+'_cg_z')*mass+extra*z)/(mass+extra)
            outputs[state+'_yaw_inertia']+=2*(local+each*x*x)
        mass=out('loaded_sprung_mass')
        outputs['loaded_sprung_cg_x']=out('loaded_sprung_cg_x')*mass/(mass+extra)
        outputs['loaded_sprung_cg_z']=(out('loaded_sprung_cg_z')*mass+extra*z)/(mass+extra)
        outputs['loaded_sprung_mass']=mass+extra
