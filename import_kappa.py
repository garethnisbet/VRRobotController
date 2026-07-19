"""
Import a kappa diffractometer from the current Blender scene into the Three.js viewer.

Extracts bone transforms, rotation axes, mesh parenting, and joint limits
from a Blender armature, then exports a config JSON and skin-free GLB.

Usage from Blender Script Editor:
    exec(open('/path/to/RobotVisualisation/import_kappa.py').read())

To override defaults, set variables before exec():
    ARMATURE_NAME = 'MyArmature'
    CONFIG_FILE = 'my_diffractometer_config.json'
    GLB_FILE = 'my_diffractometer_scene.glb'
    DEVICE_NAME = 'My Diffractometer'
    project_dir = '/explicit/path/to/RobotVisualisation'  # only if auto-detect fails

Prerequisites:
  - Bones for the three kappa geometry joints MUST be named exactly:
      theta, kappa, phi
    The viewer uses these names to activate virtual chi/theta/phi control.
  - All other bones can use any names.
  - Set IK limits on pose bones (Properties > Bone > Inverse Kinematics) for
    accurate joint limits; otherwise named defaults are used.
"""

import bpy
import json
import os
import re
import math
import mathutils

# ── Overridable settings ─────────────────────────────────────────────────────

try:
    project_dir = PROJECT_DIR
except NameError:
    project_dir = None
    try:
        # Blender's Run Script sets __file__ to '<blend file>/<text name>',
        # which is not a real filesystem path — only trust __file__ if it
        # actually exists on disk.
        if os.path.isfile(__file__):
            project_dir = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        pass  # exec() doesn't set __file__
    if project_dir is None:
        # Find this script via the Text Editor (filepath may be '//'-relative)
        _match = next(
            (bpy.path.abspath(t.filepath) for t in bpy.data.texts
             if t.filepath and os.path.basename(t.filepath) == 'import_kappa.py'),
            None
        )
        if _match and os.path.isfile(_match):
            project_dir = os.path.dirname(os.path.abspath(_match))
    if project_dir is None:
        # The documented usage is exec(open('/path/import_kappa.py').read())
        # from a text block — recover the path from that exec line.
        _rx = re.compile(r"""open\(\s*['"]([^'"]+import_kappa\.py)['"]""")
        for _t in bpy.data.texts:
            for _line in _t.lines:
                _m = _rx.search(_line.body)
                if _m and os.path.isfile(bpy.path.abspath(_m.group(1))):
                    project_dir = os.path.dirname(
                        os.path.abspath(bpy.path.abspath(_m.group(1))))
                    break
            if project_dir:
                break
    if project_dir is None:
        raise RuntimeError(
            "Cannot determine project directory. "
            "Open import_kappa.py in Blender's Text Editor before running, "
            "or set PROJECT_DIR = '/path/to/RobotVisualisation' before exec()."
        )

try:
    armature_name = ARMATURE_NAME
except NameError:
    armature_name = None  # auto-detect

try:
    config_file = CONFIG_FILE
except NameError:
    config_file = None  # derived from device name

try:
    glb_file = GLB_FILE
except NameError:
    glb_file = None  # derived from config name

try:
    device_name = DEVICE_NAME
except NameError:
    device_name = None  # derived from armature name

# Virtual axes are synthetic joints with no corresponding Blender bone.
# Define them before exec() as a list of dicts, e.g.:
#   VIRTUAL_AXES = [
#       {'name': 'virtual_axis_1', 'parent': 'kdelta.002',
#        'restPos': [0, 0, -0.362], 'restQuat': [0.01605, -0.68861, 0.72476, 0.01681]},
#       {'name': 'virtual_axis_2', 'parent': 'virtual_axis_1',
#        'restPos': [0, 0, -0.045], 'restQuat': [0.74273, 0.66879, 0.00018, -0.03281]},
#   ]
# Parents are resolved by name and may reference earlier entries in this list.
try:
    virtual_axes = VIRTUAL_AXES
except NameError:
    virtual_axes = []

# Explicit mesh→bone overrides, used when proximity gives wrong results, e.g.:
#   MESH_BONE_MAP = {'Base': 'root_bone', 'UpperBase': 'root_bone'}
try:
    mesh_bone_map_override = MESH_BONE_MAP
except NameError:
    mesh_bone_map_override = {}

# ── Coordinate conversion ────────────────────────────────────────────────────
# Blender Z-up → glTF/Three.js Y-up: X→X, Y→-Z, Z→Y

C = mathutils.Matrix((
    (1, 0, 0, 0),
    (0, 0, 1, 0),
    (0, -1, 0, 0),
    (0, 0, 0, 1),
))
C_inv = C.inverted()

# Pure geometric axis mapping: C @ Blender_axis
#   Blender X → Three.js  X  = [1, 0, 0]
#   Blender Y → Three.js -Z  = [0, 0,-1]
#   Blender Z → Three.js +Y  = [0, 1, 0]
# If a joint rotates in the wrong direction, negate its axis in the config.
AXIS_MAP = {
    'X': [ 1,  0,  0],
    'Y': [ 0,  0, -1],
    'Z': [ 0,  1,  0],
}

# Axis overrides: force a specific Three.js axis for named joints (case-insensitive).
# Used when the pure geometric AXIS_MAP result has the wrong sign for the physical
# convention.  Example: kappa bones typically use Blender Y (→ AXIS_MAP gives [0,0,-1])
# but the diffractometer API convention requires rotation about +Z ([0,0,1]).
# Keys are lowercase bone names; values are [x, y, z] Three.js unit vectors.
JOINT_AXIS_OVERRIDES = {
    'theta': [0, 0, 1],
    'kappa': [0, 0, 1],
}

# Default joint limits by exact bone name (degrees), applied when no IK limits are set.
# Joints not listed here default to [-180, 180].
JOINT_LIMITS = {
    'theta':     [-200,  180],
    'kappa':     [-150,  150],
    'phi':       [-360,  360],
    'mu':        [ -10,  220],
    'gamma':     [ -10,  120],
    'delta':     [ -10,  150],
    'two_theta': [ -10,  170],
}

# Demo pose values for named joints (case-insensitive); all others default to 0.
# Covers i16/i19-family joint names (both lowercase config names and capitalised Blender names).
DEMO_ANGLES = {
    'gamma':             30,
    'delta':             45,
    'stokes':            90,
    'detector_end':      15,
    'merlin_rot':        10,
    'crystal_rot':       45,
    'mu':                20,
    'theta':             30,
    'kappa':             50,
    'phi':              -60,
}


def sanitize_gltf_name(name):
    """Match Three.js GLTFLoader name sanitization: spaces→underscores, dots removed."""
    return name.replace(' ', '_').replace('.', '')


def compact_json(obj, indent=2):
    """JSON with compact single-line arrays for short numeric lists."""
    raw = json.dumps(obj, indent=indent)

    def collapse_array(m):
        inner = m.group(0)
        collapsed = re.sub(r'\s+', ' ', inner).strip()
        if len(collapsed) < 80 and '{' not in collapsed:
            return collapsed
        return inner

    return re.sub(r'\[[\s\d.,eE+\-"]+?\]', collapse_array, raw)


# ── Step 1: Find the armature ────────────────────────────────────────────────

if armature_name:
    arm_obj = bpy.data.objects[armature_name]
else:
    arm_obj = None
    for obj in bpy.data.objects:
        if obj.type == 'ARMATURE':
            arm_obj = obj
            break
    if arm_obj is None:
        raise RuntimeError("No armature found in scene. Set ARMATURE_NAME explicitly.")
    armature_name = arm_obj.name

arm = arm_obj.data

if device_name is None:
    device_name = armature_name

if config_file is None:
    slug = re.sub(r'[^a-zA-Z0-9]+', '_', device_name).strip('_').lower()
    config_file = f'{slug}_config.json'

if glb_file is None:
    slug = re.sub(r'[^a-zA-Z0-9]+', '_', device_name).strip('_').lower()
    glb_file = f'{slug}_scene.glb'

print(f"\n{'='*60}")
print(f"Kappa Diffractometer Importer")
print(f"{'='*60}")
print(f"  Armature:  {armature_name}")
print(f"  Device:    {device_name}")
print(f"  Config:    {config_file}")
print(f"  GLB:       {glb_file}")
print(f"{'='*60}\n")


# ── Step 2: Extract bone transforms ──────────────────────────────────────────

print("Step 2: Extracting bone transforms...")

bones_ordered = []
bone_index = {}

for bone in arm.bones:
    bones_ordered.append(bone)
    bone_index[bone.name] = len(bones_ordered) - 1

bone_data = []
for bone in bones_ordered:
    if bone.parent:
        local_mat = C @ bone.parent.matrix_local.inverted() @ bone.matrix_local @ C_inv
    else:
        local_mat = C @ bone.matrix_local @ C_inv

    pos = local_mat.to_translation()
    quat = local_mat.to_quaternion()

    bone_data.append({
        'blender_name': bone.name,
        'parent_name': bone.parent.name if bone.parent else None,
        'pos': [round(pos.x, 5), round(pos.y, 5), round(pos.z, 5)],
        'quat': [round(quat.w, 5), round(quat.x, 5), round(quat.y, 5), round(quat.z, 5)],
    })

print(f"  Found {len(bone_data)} bones")


# ── Step 3: Determine rotation axes and fixed joints ─────────────────────────

print("Step 3: Determining rotation axes...")

pose = arm_obj.pose

for pb in pose.bones:
    bd = bone_data[bone_index[pb.name]]
    lock_w = pb.lock_rotation_w
    lock_xyz = list(pb.lock_rotation)

    free = []
    if not lock_w:
        free.append('W')
    if not lock_xyz[0]:
        free.append('X')
    if not lock_xyz[1]:
        free.append('Y')
    if not lock_xyz[2]:
        free.append('Z')

    # Bones named virtual_axis_* are reference frames — always fixed
    if pb.name.startswith('virtual_axis'):
        bd['fixed'] = True
        bd['axis'] = [0, 1, 0]
        status = "FIXED (virtual axis)"
    elif len(free) == 0 or len(free) == 4:
        bd['fixed'] = True
        bd['axis'] = [0, 1, 0]
        status = "FIXED (all locked)" if len(free) == 0 else "FIXED (all free)"
    else:
        free_axes = [f for f in free if f != 'W']
        if len(free_axes) == 1:
            bd['fixed'] = False
            _override = {k.lower(): v for k, v in JOINT_AXIS_OVERRIDES.items()}.get(pb.name.lower())
            bd['axis'] = _override if _override is not None else AXIS_MAP[free_axes[0]]
            status = f"axis={free_axes[0]} → {bd['axis']}" + (" (override)" if _override else "")
        else:
            bd['fixed'] = True
            bd['axis'] = [0, 1, 0]
            status = f"FIXED (multiple free: {free_axes}, review manually)"

    print(f"  {pb.name}: free={free}  → {status}")


# ── Step 3b: Auto-generate virtual axes from kappa geometry ──────────────────
# Runs only when theta/kappa/phi are all present and VIRTUAL_AXES was not set.

if not virtual_axes:
    _theta_bd = next((bd for bd in bone_data if bd['blender_name'] == 'theta'), None)
    _phi_bd   = next((bd for bd in bone_data if bd['blender_name'] == 'phi'),   None)
    _kappa_bd = next((bd for bd in bone_data if bd['blender_name'] == 'kappa'), None)

    if _theta_bd and _phi_bd and _kappa_bd:
        print("\nStep 3b: Auto-generating virtual axes from kappa geometry...")

        _phi_bone = arm.bones['phi']

        # virtual_axis_1 — child of phi's parent (the last fixed link before phi).
        # This joint follows the full theta→kappa chain but NOT phi spin, so it
        # represents the virtual chi frame at the sample position.
        # Its local transform is identical to phi's (same pos/quat relative to that parent).
        _phi_parent_name = _phi_bd['parent_name']
        virtual_axes.append({
            'name':     'virtual_axis_1',
            'parent':   _phi_parent_name,
            'restPos':  list(_phi_bd['pos']),
            'restQuat': list(_phi_bd['quat']),
        })
        print(f"  virtual_axis_1: parent='{_phi_parent_name}' (phi's parent), pos={_phi_bd['pos']}")

        # virtual_axis_2 — child of virtual_axis_1, offset by phi bone length along -Z.
        # Provides a second reference point so the axis direction is visible in the scene.
        _phi_len = round(_phi_bone.length, 5)
        virtual_axes.append({
            'name':     'virtual_axis_2',
            'parent':   'virtual_axis_1',
            'restPos':  [0, 0, -_phi_len],
            'restQuat': [1, 0, 0, 0],
        })
        print(f"  virtual_axis_2: parent='virtual_axis_1', offset=[0,0,{-_phi_len}]")


# ── Step 4: Determine mesh-to-bone parenting ─────────────────────────────────

print("\nStep 4: Mapping meshes to bones...")


def find_bone_ancestor(obj):
    """Walk up the parent chain to find the bone this mesh ultimately belongs to."""
    current = obj
    while current.parent:
        if current.parent == arm_obj and current.parent_bone:
            return current.parent_bone
        current = current.parent
    return None


def find_nearest_bone(obj):
    """Return the bone whose head is nearest to obj's world-space bounding-box centre."""
    corners = [obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box]
    centre = sum(corners, mathutils.Vector((0, 0, 0))) / 8
    best_name, best_dist = None, float('inf')
    for bone in arm.bones:
        d = (centre - arm_obj.matrix_world @ bone.head_local).length
        if d < best_dist:
            best_dist, best_name = d, bone.name
    return best_name, best_dist


mesh_to_bone = {}
unparented_meshes = []

for obj in bpy.data.objects:
    if obj.type != 'MESH':
        continue
    if obj.hide_render:
        continue

    if obj.name in mesh_bone_map_override:
        bone_name = mesh_bone_map_override[obj.name]
        mesh_to_bone[obj.name] = bone_name
        print(f"  {obj.name} → bone: {bone_name} (override)")
        continue

    bone_name = find_bone_ancestor(obj)
    if bone_name:
        mesh_to_bone[obj.name] = bone_name
        print(f"  {obj.name} → bone: {bone_name}")
    else:
        unparented_meshes.append(obj.name)
        print(f"  {obj.name} → (no bone parent, will be static)")


# ── Step 5: Extract joint limits ─────────────────────────────────────────────

print("\nStep 5: Extracting joint limits...")

for pb in pose.bones:
    bd = bone_data[bone_index[pb.name]]

    if bd['fixed']:
        bd['limits'] = [0, 0]
        continue

    free_axes = [f for f in ['W', 'X', 'Y', 'Z']
                 if not (f == 'W' and pb.lock_rotation_w)
                 and not (f == 'X' and pb.lock_rotation[0])
                 and not (f == 'Y' and pb.lock_rotation[1])
                 and not (f == 'Z' and pb.lock_rotation[2])]

    rot_axis = [f for f in free_axes if f != 'W']
    if not rot_axis:
        bd['limits'] = [0, 0]
        continue

    axis_letter = rot_axis[0]
    has_limit = False

    if axis_letter == 'X' and pb.use_ik_limit_x:
        bd['limits'] = [round(math.degrees(pb.ik_min_x)), round(math.degrees(pb.ik_max_x))]
        has_limit = True
    elif axis_letter == 'Y' and pb.use_ik_limit_y:
        bd['limits'] = [round(math.degrees(pb.ik_min_y)), round(math.degrees(pb.ik_max_y))]
        has_limit = True
    elif axis_letter == 'Z' and pb.use_ik_limit_z:
        bd['limits'] = [round(math.degrees(pb.ik_min_z)), round(math.degrees(pb.ik_max_z))]
        has_limit = True

    if has_limit:
        print(f"  {pb.name}: {bd['limits']} deg  (from IK limits)")
    else:
        bd['limits'] = JOINT_LIMITS.get(pb.name, [-180, 180])
        src = "named default" if pb.name in JOINT_LIMITS else "default"
        print(f"  {pb.name}: {bd['limits']} deg  ({src})")


# ── Step 6: Export GLB (skin-free) ───────────────────────────────────────────

print("\nStep 6: Exporting GLB...")

glb_path = os.path.join(project_dir, glb_file)

# Hide render-hidden meshes so they don't appear in the export.
visibility_backup = []
for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.hide_render and not obj.hide_viewport:
        visibility_backup.append(obj)
        obj.hide_viewport = True

# Hide ALL armature objects.  Armature bones are exported as GLTF nodes, and if
# any bone shares a name with a mesh object (common in kappa setups where the
# bone and its mesh are both called 'phi', 'theta', etc.) the bone node ends up
# in Three.js's allNodes map, overwriting the mesh node.  That leaves the mesh
# unattached in the viewer.  Hiding armatures before export prevents this.
#
# The GLTF exporter calls bpy.ops.object.mode_set at startup; that operator
# polls the active object and fails if it is hidden.  Clear the active object
# first so there is nothing hidden for the exporter to trip over.
active_object_backup = bpy.context.view_layer.objects.active
bpy.context.view_layer.objects.active = None

armature_visibility_backup = []
for obj in bpy.data.objects:
    if obj.type == 'ARMATURE' and not obj.hide_viewport:
        armature_visibility_backup.append(obj)
        obj.hide_viewport = True

parenting_backup = []
for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.parent == arm_obj and obj.parent_bone:
        parenting_backup.append({
            'obj': obj,
            'parent_bone': obj.parent_bone,
            'parent_type': obj.parent_type,
            'matrix_world': obj.matrix_world.copy()
        })
        mat = obj.matrix_world.copy()
        obj.parent = None
        obj.matrix_world = mat

try:
    bpy.ops.export_scene.gltf(
        filepath=glb_path,
        export_format='GLB',
        export_apply=True,
        use_visible=True,
        export_yup=True,
        export_animations=False,
        export_skins=False,
    )
finally:
    # Always restore — even if export fails — so the Blender scene is not left broken.
    # Two-pass restore: set parent relationships first, then update the depsgraph so
    # Blender computes bone matrices before we assign matrix_world.  Without the
    # update() call Blender resolves matrix_parent_inverse against the armature origin
    # (parent_type='OBJECT') and the bone connection is silently lost.
    for info in parenting_backup:
        obj = info['obj']
        obj.parent = arm_obj
        obj.parent_bone = info['parent_bone']
        obj.parent_type = info['parent_type']
    bpy.context.view_layer.update()
    for info in parenting_backup:
        info['obj'].matrix_world = info['matrix_world']
    for obj in visibility_backup:
        obj.hide_viewport = False
    for obj in armature_visibility_backup:
        obj.hide_viewport = False
    bpy.context.view_layer.objects.active = active_object_backup

print(f"  Exported to {glb_path}")
print(f"  Unparented/restored {len(parenting_backup)} meshes")
print(f"  Hidden/restored {len(visibility_backup)} render-hidden meshes")
print(f"  Hidden/restored {len(armature_visibility_backup)} armature(s)")


# ── Step 7: Generate config JSON ─────────────────────────────────────────────

print("\nStep 7: Generating config JSON...")

joints = []
for bd in bone_data:
    parent_idx = bone_index[bd['parent_name']] if bd['parent_name'] else -1

    joint = {
        'name': bd['blender_name'],
        'bone': bd['blender_name'],
        'restPos': bd['pos'],
        'restQuat': bd['quat'],
        'axis': bd['axis'],
        'limits': bd['limits'],
        'parent': parent_idx,
    }
    if bd['fixed']:
        joint['fixed'] = True

    joints.append(joint)

# Inject virtual axes (synthetic joints with no Blender bone)
if virtual_axes:
    print(f"\n  Injecting {len(virtual_axes)} virtual axis joint(s)...")
    name_to_idx = {j['name']: i for i, j in enumerate(joints)}
    for va in virtual_axes:
        parent_name = va.get('parent')
        if parent_name is None:
            parent_idx = -1
        elif parent_name in name_to_idx:
            parent_idx = name_to_idx[parent_name]
        else:
            print(f"  WARNING: parent '{parent_name}' not found for '{va['name']}' — using root")
            parent_idx = -1
        new_idx = len(joints)
        joints.append({
            'name': va['name'],
            'restPos': va['restPos'],
            'restQuat': va['restQuat'],
            'axis': [0, 1, 0],
            'limits': [0, 0],
            'parent': parent_idx,
            'fixed': True,
        })
        name_to_idx[va['name']] = new_idx
        print(f"  {va['name']}: idx={new_idx}, parent={parent_idx} ({parent_name})")

# Links: mesh name → joint index
links = []
for mesh_name, bone_name in sorted(mesh_to_bone.items()):
    joint_idx = bone_index.get(bone_name)
    if joint_idx is None:
        print(f"  WARNING: bone '{bone_name}' not found in armature for mesh '{mesh_name}'")
        continue
    links.append({
        'name': sanitize_gltf_name(mesh_name),
        'label': mesh_name.replace('_', ' '),
        'joint': joint_idx,
    })

# Demo pose: kappa-specific values for theta/kappa/phi, 0 elsewhere
_demo_lower = {k.lower(): v for k, v in DEMO_ANGLES.items()}
demo_pose = [_demo_lower.get(j['name'].lower(), 0) for j in joints]

# Validate kappa geometry joints
required = {'theta', 'kappa', 'phi'}
present = {j['name'] for j in joints}
missing = required - present
if missing:
    print(f"\n  WARNING: Missing kappa geometry joints: {sorted(missing)}")
    print(f"  The viewer will NOT activate virtual angle mode.")
    print(f"  Rename the corresponding bones in Blender to: theta, kappa, phi")
else:
    kappa_movable = [j['name'] for j in joints if j['name'] in required and not j.get('fixed')]
    print(f"  Kappa geometry joints found (movable): {kappa_movable}")

config = {
    'name': device_name,
    'armature': armature_name,
    'model': glb_file,
    'joints': joints,
    'links': links,
    'demoPose': demo_pose,
}

config_path = os.path.join(project_dir, config_file)
with open(config_path, 'w') as f:
    f.write(compact_json(config))
    f.write('\n')

movable_count = sum(1 for j in joints if not j.get('fixed'))
fixed_count = len(joints) - movable_count
print(f"  Wrote {config_path}")
print(f"  {movable_count} movable joints, {fixed_count} fixed joints")
print(f"  {len(links)} links")


# ── Step 8: Register in panel.js ─────────────────────────────────────────────

print("\nStep 8: Checking panel.js registration...")

panel_path = os.path.join(project_dir, 'js', 'panel.js')
with open(panel_path, 'r') as f:
    panel_src = f.read()

if config_file in panel_src:
    print(f"  '{config_file}' already registered in panel.js")
else:
    pattern = r"(export\s+const\s+configFiles\s*=\s*\[.*?)(])"
    match = re.search(pattern, panel_src, re.DOTALL)
    if match:
        before_bracket = match.group(1).rstrip()
        if not before_bracket.endswith(','):
            before_bracket += ','
        new_src = (panel_src[:match.start()] + before_bracket +
                   f"\n    '{config_file}'\n" + match.group(2) + panel_src[match.end():])
        with open(panel_path, 'w') as f:
            f.write(new_src)
        print(f"  Added '{config_file}' to configFiles in panel.js")
    else:
        print(f"  WARNING: Could not find configFiles array in panel.js — add '{config_file}' manually")


# ── Summary ──────────────────────────────────────────────────────────────────

movable_joints = [j for j in joints if not j.get('fixed')]
print(f"\n{'='*60}")
print(f"Import complete!")
print(f"{'='*60}")
print(f"  Config:  {config_file}")
print(f"  GLB:     {glb_file}")
print(f"  Joints:  {movable_count} movable, {fixed_count} fixed")
print(f"  Links:   {len(links)} meshes")

print(f"\nMovable joints:")
for j in movable_joints:
    print(f"  {j['name']}: axis={j['axis']}, limits={j['limits']}")

virtual_axes = [j for j in joints if j['name'].startswith('virtual_axis')]
if virtual_axes:
    print(f"\nVirtual axes ({len(virtual_axes)}):")
    for j in virtual_axes:
        print(f"  {j['name']}: parent={j['parent']}, restQuat={j['restQuat']}")

print(f"\nTo test, open:")
print(f"  http://localhost:8000/threejs_scene.html?config={config_file}")
print(f"\nReview checklist:")
print(f"  1. All meshes load and appear correctly")
print(f"  2. FK sliders move the correct joints")
print(f"  3. Each joint rotates about the correct axis")
print(f"  4. Labels toggle shows correct mesh names")
print(f"  5. Virtual Angles panel appears (theta/kappa/phi control)")
print(f"  6. Chi slider moves kappa while compensating theta and phi")
print(f"  7. Kappa alpha angle is correct (check browser console)")
print(f"  8. Kappa sign toggle switches between +/- kappa solutions")
print(f"\nIf rotation directions are wrong, negate the axis in the config.")
print(f"If Virtual Angles panel is missing, check bone names: theta, kappa, phi")
print(f"If meshes are missing, check GLTFLoader name sanitization in browser console.")
print(f"{'='*60}")
