import cadquery as cq
from cadquery import exporters
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
import os, math
OUT='/mnt/data/bipedal-walking-robot-cad'
os.makedirs(OUT,exist_ok=True)
# Reconstructed illustrative dimensions in mm; not measured from the source video.
# Coordinate system: X left/right, Y front/back, Z vertical. Ground plane Z=0.
assy=cq.Assembly(name='Linkage_Biped_Reconstructed_Assumed_Dimensions')

def color(rgb): return cq.Color(*rgb)
metal=color((0.72,0.74,0.77)); dark=color((0.28,0.30,0.33)); platec=color((0.82,0.72,0.51)); footc=color((0.62,0.64,0.67)); jointc=color((0.45,0.47,0.50)); motorc=color((0.22,0.24,0.27))

def add(name, shape, c=metal):
    assy.add(shape, name=name, color=c)

def box_center(name, dx,dy,dz, x,y,z, c=metal):
    sh=cq.Workplane('XY').box(dx,dy,dz).translate((x,y,z))
    add(name,sh,c)
    return sh

def cylinder_between(name,p1,p2,r,c=metal):
    a=cq.Vector(*p1); b=cq.Vector(*p2); d=b-a; length=d.Length
    sh=cq.Solid.makeCylinder(r,length,a,d.normalized())
    add(name,sh,c)
    return sh

def pivot(name,x,y,z,r=7,th=12,c=jointc):
    sh=cq.Solid.makeCylinder(r,th,cq.Vector(x,y-th/2,z),cq.Vector(0,1,0))
    add(name,sh,c)

def link_plate(name,p1,p2,y, width=18, thick=5, hole_r=3.2, c=metal):
    # Flat capsule-shaped link plate lying in X-Z plane, extruded along Y.
    x1,z1=p1; x2,z2=p2
    dx=x2-x1; dz=z2-z1; L=math.hypot(dx,dz); ang=math.degrees(math.atan2(dz,dx))
    # rectangular beam plus round ends; cylinders axis Y
    mid=((x1+x2)/2,y,(z1+z2)/2)
    sh=cq.Workplane('XY').box(L,width,thick).val() # length local X, width local Y, thickness local Z
    # rotate local X into X-Z direction, and thickness axis towards Y; easier use polygon extruded in Y
    ux,uz=dx/L,dz/L; nx,nz=-uz,ux; w=width/2
    pts=[(x1+nx*w,z1+nz*w),(x2+nx*w,z2+nz*w),(x2-nx*w,z2-nz*w),(x1-nx*w,z1-nz*w)]
    # XZ plane normal points -Y; extrude and shift to desired center y
    wp=cq.Workplane('XZ').polyline(pts).close().extrude(thick).translate((0,y+thick/2,0))
    s=wp.val()
    # end rounds along Y, overlapping rectangle
    for i,(x,z) in enumerate([(x1,z1),(x2,z2)]):
        disk=cq.Solid.makeCylinder(width/2,thick,cq.Vector(x,y-thick/2,z),cq.Vector(0,1,0))
        s=s.fuse(disk)
    # pivot holes through Y
    for x,z in [(x1,z1),(x2,z2)]:
        cut=cq.Solid.makeCylinder(hole_r,thick+2,cq.Vector(x,y-thick/2-1,z),cq.Vector(0,1,0))
        s=s.cut(cut)
    add(name,s,c)
    return s

# Upper mounting plate: estimated 250 x 110 x 8 mm, with mounting holes.
plate=cq.Workplane('XY').box(250,110,8).translate((0,0,405))
for x in [-100,-60,-20,20,60,100]:
    for y in [-35,35]:
        plate=plate.cut(cq.Solid.makeCylinder(2.5,10,cq.Vector(x,y,400),cq.Vector(0,0,1)))
add('Upper_mounting_plate_ASSUMED_250x110x8',plate,platec)
# Mount brackets and six servo envelope blocks (illustrative only; exact servo shape unknown)
for side,x in [('L',-62),('R',62)]:
    for j,yy in enumerate([-32,32]):
        box_center(f'{side}_upper_bracket_{j+1}',34,16,20,x,yy,390,dark)
    # Three compact servo envelope blocks per leg; positions are assumptions
    box_center(f'{side}_hip_servo_envelope',36,26,28,x,0,371,motorc)
    # vertical leg bars, two parallel depth rails
    for k,yy in enumerate([-24,24]):
        cylinder_between(f'{side}_outer_leg_rail_{k+1}',(x,yy,365),(x+(-4 if side=='L' else 4),yy,150),4.2,dark)
    # linkage pivots and flat links in sagittal plane, duplicated at front/back offsets
    sx=-1 if side=='L' else 1
    hip=(x,320); knee=(x+sx*13,225); ankle=(x+sx*4,118); mid=(x+sx*28,260); lower=(x-sx*18,180)
    for j,yy in enumerate([-24,24]):
        link_plate(f'{side}_upper_link_{j+1}',hip,mid,yy,16,5,3.0,metal)
        link_plate(f'{side}_diagonal_link_{j+1}',mid,knee,yy,15,5,3.0,metal)
        link_plate(f'{side}_lower_link_{j+1}',knee,lower,yy,15,5,3.0,metal)
        link_plate(f'{side}_ankle_link_{j+1}',lower,ankle,yy,14,5,3.0,metal)
    # joint boss discs at key pivots
    for j,yy in enumerate([-24,24]):
        for nm,(xx,zz) in [('hip',hip),('mid',mid),('knee',knee),('lower',lower),('ankle',ankle)]:
            pivot(f'{side}_{nm}_pivot_{j+1}',xx,yy,zz,5.5,8,jointc)
    # foot sole plate 100x60x5, with heel/toe plate support
    footx=x
    foot=cq.Workplane('XY').box(105,62,5).translate((footx,0,5))
    for xx in [footx-35,footx,footx+35]:
        for yy in [-18,18]: foot=foot.cut(cq.Solid.makeCylinder(2.4,7,cq.Vector(xx,yy,1.5),cq.Vector(0,0,1)))
    add(f'{side}_foot_plate_ASSUMED_105x62x5',foot,footc)
    # toe/heel triangular gussets
    for j,yy in enumerate([-22,22]):
        gus=cq.Workplane('XZ').polyline([(footx-35,7),(footx+22,7),(footx+4,45)]).close().extrude(4).translate((0,yy+2,0))
        add(f'{side}_foot_gusset_{j+1}',gus,footc)
    # ankle upright bracket and lower servo envelope
    box_center(f'{side}_ankle_servo_envelope',30,24,24,footx,0,48,motorc)
    # rear support vertical connecting ankle to foot
    cylinder_between(f'{side}_ankle_support', (footx,0,70),(footx,0,115),4.5,dark)

# Central electronics / mounting stand-offs, visible under top plate
for i,x in enumerate([-80,80]):
    for j,y in enumerate([-32,32]):
        cylinder_between(f'plate_standoff_{i}_{j}',(x,y,360),(x,y,398),3.5,dark)
# Small generic electronics enclosure on top plate to indicate payload space
box_center('Electronics_enclosure_placeholder_ASSUMED',68,48,22,0,0,421,dark)
# Export complete assembly STEP and a neutral STL preview
step_path=os.path.join(OUT,'biped_robot_reconstructed_full_assembly.step')
assy.save(step_path)
# STL assembly export is useful for quick preview but not a replacement for STEP
try:
    exporters.export(cq.Compound.makeCompound([obj.obj.val() if hasattr(obj.obj,'val') else obj.obj for obj in assy.objects.values()]),os.path.join(OUT,'biped_robot_reconstructed_preview.stl'))
except Exception as e:
    print('STL export skipped:',e)
# Script copy for editable parametric regeneration
import shutil
shutil.copy(__file__,os.path.join(OUT,'generate_biped_cad.py'))
readme='''# Reconstructed linkage-based biped robot CAD\n\n**Important:** this is a visual reconstruction based on the uploaded video, not the original CAD and not a dimensionally verified replica. All dimensions are assumed in millimetres. Servo envelopes are placeholders; the exact linkage topology, joint coordinates, hole sizes, plate thicknesses, clearances and hardware are unknown.\n\n## Files\n- `biped_robot_reconstructed_full_assembly.step` — neutral STEP assembly for import into SolidWorks, Fusion 360, FreeCAD and other CAD tools.\n- `biped_robot_reconstructed_preview.stl` — mesh preview (if exported).\n- `generate_biped_cad.py` — CadQuery source used to generate the model.\n\n## Assumed envelope and dimensions\n- Top plate: 250 × 110 × 8 mm.\n- Overall height: approximately 430 mm including top plate/electronics placeholder.\n- Foot plates: 105 × 62 × 5 mm each.\n- Leg centres: approximately 124 mm apart.\n- Flat linkage bars: 14–18 mm nominal width, 5 mm thickness.\n- Cylindrical rails/pivots and servo blocks are illustrative envelopes, not validated component models.\n\n## Before manufacturing\nMeasure the actual robot or obtain its original drawings. Confirm centre-to-centre link lengths, servo model and spline geometry, pivot offsets, fasteners, material, thickness, interference, ground clearance and full motion envelope. Run a kinematic/interference check and structural analysis before fabrication.\n'''
open(os.path.join(OUT,'README_DIMENSIONS_AND_LIMITATIONS.md'),'w').write(readme)
print('Wrote',step_path,os.path.getsize(step_path))
