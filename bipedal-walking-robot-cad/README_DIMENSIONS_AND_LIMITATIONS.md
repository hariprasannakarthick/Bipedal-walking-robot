# Reconstructed linkage-based biped robot CAD

**Important:** this is a visual reconstruction based on the uploaded video, not the original CAD and not a dimensionally verified replica. All dimensions are assumed in millimetres. Servo envelopes are placeholders; the exact linkage topology, joint coordinates, hole sizes, plate thicknesses, clearances and hardware are unknown.

## Files
- `biped_robot_reconstructed_full_assembly.step` — neutral STEP assembly for import into SolidWorks, Fusion 360, FreeCAD and other CAD tools.
- `biped_robot_reconstructed_preview.stl` — mesh preview (if exported).
- `generate_biped_cad.py` — CadQuery source used to generate the model.

## Assumed envelope and dimensions
- Top plate: 250 × 110 × 8 mm.
- Overall height: approximately 430 mm including top plate/electronics placeholder.
- Foot plates: 105 × 62 × 5 mm each.
- Leg centres: approximately 124 mm apart.
- Flat linkage bars: 14–18 mm nominal width, 5 mm thickness.
- Cylindrical rails/pivots and servo blocks are illustrative envelopes, not validated component models.

## Before manufacturing
Measure the actual robot or obtain its original drawings. Confirm centre-to-centre link lengths, servo model and spline geometry, pivot offsets, fasteners, material, thickness, interference, ground clearance and full motion envelope. Run a kinematic/interference check and structural analysis before fabrication.
