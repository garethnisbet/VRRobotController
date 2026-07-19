"""
Import a hexapod (Stewart platform) from the current Blender scene into the Three.js viewer.

Extracts pivot positions, mesh parenting, and platform limits from a Blender
armature with Damped Track leg constraints, then exports a config JSON and GLB.

The script auto-detects the hexapod structure:
  - Finds bones with Damped Track constraints → leg pairs (lower/upper)
  - Finds the control bone that upper legs follow (via Child Of) → platform
  - Reads Limit Location / Limit Rotation constraints → platform limits

Usage from Blender Script Editor:
    exec(open('/FastDrive/Dropbox/ClaudeCodeProjects/RobotVisualisation/import_hexapod.py').read())

To override defaults, set variables before exec():
    ARMATURE_NAME = 'MyArmature'
    CONTROL_BONE  = 'ControlHandle'
    CONFIG_FILE   = 'hexapod_config.json'
    GLB_FILE      = 'hexapod_scene.glb'
    DEVICE_NAME   = 'Hexapod'
    BASE_MESH     = 'BasePlate'
    PLATFORM_MESH = 'TopPlate'
"""

import bpy
import json
import os
import re
import math
import mathutils

# ── Overridable settings ─────────────────────────────────────────────────────

PROJECT_DIR = '/FastDrive/Dropbox/ClaudeCodeProjects/RobotVisualisation'

try:    armature_name = ARMATURE_NAME
except NameError: armature_name = None

try:    control_bone_name = CONTROL_BONE
except NameError: control_bone_name = None

try:    config_file = CONFIG_FILE
except NameError: config_file = None

try:    glb_file = GLB_FILE
except NameError: glb_file = None

try:    device_name = DEVICE_NAME
except NameError: device_name = None

try:    base_mesh_name = BASE_MESH
except NameError: base_mesh_name = None

try:    platform_mesh_name = PLATFORM_MESH
except NameError: platform_mesh_name = None

DEFAULT_LIMITS = {
    'x':  [-30, 30],
    'y':  [-30, 30],
    'z':  [-20, 20],
    'rx': [-11, 11],
    'ry': [-11, 11],
    'rz': [-20, 20],
}

DEFAULT_DEMO_POSE = [0, 0, 5, 5, 0, 10]


# ── Helpers ──────────────────────────────────────────────────────────────────

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


def blender_to_threejs(vec):
    """Blender Z-up world position → Three.js Y-up: (x,y,z) → (x, z, -y)."""
    return [round(vec.x, 6), round(vec.z, 6), round(-vec.y, 6)]


# ── Step 1: Find the armature ────────────────────────────────────────────────

if armature_name:
    arm_obj = bpy.data.objects.get(armature_name)
    if arm_obj is None:
        available = [o.name for o in bpy.data.objects if o.type == 'ARMATURE']
        raise RuntimeError(
            f"Armature '{armature_name}' not found.\n"
            f"Available armatures: {available}"
        )
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
    device_name = 'Hexapod'

if config_file is None:
    slug = re.sub(r'[^a-zA-Z0-9]+', '_', device_name).strip('_').lower()
    config_file = f'{slug}_config.json'

if glb_file is None:
    slug = re.sub(r'[^a-zA-Z0-9]+', '_', device_name).strip('_').lower()
    glb_file = f'{slug}_scene.glb'

print(f"\n{'='*60}")
print(f"Hexapod Importer")
print(f"{'='*60}")
print(f"  Armature:  {armature_name}")
print(f"  Device:    {device_name}")
print(f"  Config:    {config_file}")
print(f"  GLB:       {glb_file}")
print(f"{'='*60}\n")


# ── Step 2: Identify hexapod structure ───────────────────────────────────────
# Look for Damped Track constraints → leg pairs (lower ↔ upper)
# Look for Child Of constraints → upper legs following control bone

print("Step 2: Identifying hexapod structure...")

pose = arm_obj.pose

damped_track_pairs = {}   # bone_name → target_bone_name
child_of_targets = {}     # bone_name → (target_armature_obj, target_bone_name)

for pb in pose.bones:
    for con in pb.constraints:
        if con.type == 'DAMPED_TRACK' and con.subtarget:
            target_obj = con.target
            if target_obj and target_obj.type == 'ARMATURE':
                damped_track_pairs[pb.name] = con.subtarget
        if con.type == 'CHILD_OF' and con.subtarget:
            target_obj = con.target
            if target_obj and target_obj.type == 'ARMATURE':
                child_of_targets[pb.name] = (target_obj, con.subtarget)

print(f"  Damped Track constraints: {len(damped_track_pairs)}")
print(f"  Child Of constraints:     {len(child_of_targets)}")

# Find the control bone: the one that most upper-leg bones follow
control_arm_obj = None
if control_bone_name is None:
    target_counts = {}
    target_arm_objs = {}
    for (target_arm, target_bone) in child_of_targets.values():
        target_counts[target_bone] = target_counts.get(target_bone, 0) + 1
        target_arm_objs[target_bone] = target_arm
    if target_counts:
        control_bone_name = max(target_counts, key=target_counts.get)
        control_arm_obj = target_arm_objs[control_bone_name]
    else:
        raise RuntimeError(
            "No Child Of constraints found — cannot identify control bone.\n"
            "Set CONTROL_BONE explicitly."
        )
else:
    for (target_arm, target_bone) in child_of_targets.values():
        if target_bone == control_bone_name:
            control_arm_obj = target_arm
            break

# Control bone may be in the same armature or a different one
if control_arm_obj is None:
    control_arm_obj = arm_obj
control_arm_data = control_arm_obj.data

print(f"  Control bone: {control_bone_name} (in '{control_arm_obj.name}')")

# Bones directly connected to the control bone via Child Of
child_of_bones = set()
for bone_name, (target_arm, target_bone) in child_of_targets.items():
    if target_bone == control_bone_name:
        child_of_bones.add(bone_name)

# Build the set of platform-connected bones: child_of_bones + any armature
# descendants of those bones (the upper leg bones may be children of helper bones)
platform_connected = set(child_of_bones)
for bone in arm.bones:
    parent = bone.parent
    while parent:
        if parent.name in platform_connected:
            platform_connected.add(bone.name)
            break
        parent = parent.parent

print(f"  Platform-connected bones: {sorted(platform_connected)}")

# Build leg pairs from bidirectional Damped Track connections
seen_pairs = set()
dt_pairs = []
for src, tgt in damped_track_pairs.items():
    pair = frozenset((src, tgt))
    if pair not in seen_pairs:
        seen_pairs.add(pair)
        dt_pairs.append((src, tgt))

legs = []
for a, b in sorted(dt_pairs):
    a_platform = a in platform_connected
    b_platform = b in platform_connected
    if a_platform and not b_platform:
        legs.append((b, a))
    elif b_platform and not a_platform:
        legs.append((a, b))
    else:
        # Neither bone in platform set — use rest position distance to control bone
        control_head = control_arm_data.bones[control_bone_name].head_local
        dist_a = (arm.bones[a].head_local - control_head).length
        dist_b = (arm.bones[b].head_local - control_head).length
        if dist_a < dist_b:
            legs.append((b, a))
        else:
            legs.append((a, b))

legs.sort(key=lambda pair: pair[0])

if not legs:
    raise RuntimeError(
        "No leg pairs found. Check that the armature has Damped Track constraints linking leg bones."
    )

for i, (lower_name, upper_name) in enumerate(legs):
    print(f"  Leg {i+1}: lower={lower_name}  upper={upper_name}")

print(f"  Total legs: {len(legs)}")


# ── Step 3: Extract pivot positions ──────────────────────────────────────────
# Use rest-pose bone head positions (armature local) → world → Three.js

print("\nStep 3: Extracting pivot positions...")

arm_world = arm_obj.matrix_world
control_arm_world = control_arm_obj.matrix_world

control_bone_rest = control_arm_data.bones[control_bone_name]
control_head_world = control_arm_world @ control_bone_rest.head_local
platform_rest_pos = blender_to_threejs(control_head_world)

print(f"  Platform rest position (Three.js): {platform_rest_pos}")

leg_data = []
for lower_name, upper_name in legs:
    lower_bone = arm.bones[lower_name]
    upper_bone = arm.bones[upper_name]

    lower_head_world = arm_world @ lower_bone.head_local
    upper_head_world = arm_world @ upper_bone.head_local

    base_pivot = blender_to_threejs(lower_head_world)

    # Platform pivot is relative to the control bone head
    rel = upper_head_world - control_head_world
    platform_pivot_local = blender_to_threejs(rel)

    print(f"  {lower_name}: basePivot          = {base_pivot}")
    print(f"  {upper_name}: platformPivotLocal  = {platform_pivot_local}")

    leg_data.append({
        'lower_bone': lower_name,
        'upper_bone': upper_name,
        'basePivot': base_pivot,
        'platformPivotLocal': platform_pivot_local,
    })


# ── Step 4: Map meshes to bones ─────────────────────────────────────────────

print("\nStep 4: Mapping meshes to bones...")


def find_bone_ancestor(obj):
    """Walk up the parent chain to find the bone this mesh belongs to."""
    current = obj
    while current:
        if current.parent and current.parent.type == 'ARMATURE' and current.parent_bone:
            return current.parent_bone
        if current.parent and current.parent.type == 'MESH':
            current = current.parent
        else:
            break
    return None


mesh_to_bone = {}
unparented_meshes = []

for obj in bpy.data.objects:
    if obj.type != 'MESH' or obj.hide_render:
        continue
    bone_name = find_bone_ancestor(obj)
    if bone_name:
        mesh_to_bone[obj.name] = bone_name
        print(f"  {obj.name} → bone: {bone_name}")
    else:
        unparented_meshes.append(obj.name)
        print(f"  {obj.name} → (unparented)")

# Assign meshes to legs
for ld in leg_data:
    lower_meshes = [m for m, b in mesh_to_bone.items() if b == ld['lower_bone']]
    upper_meshes = [m for m, b in mesh_to_bone.items() if b == ld['upper_bone']]
    ld['lowerMesh'] = sanitize_gltf_name(lower_meshes[0]) if lower_meshes else None
    ld['upperMesh'] = sanitize_gltf_name(upper_meshes[0]) if upper_meshes else None
    if not lower_meshes:
        print(f"  WARNING: No mesh for lower bone '{ld['lower_bone']}'")
    if not upper_meshes:
        print(f"  WARNING: No mesh for upper bone '{ld['upper_bone']}'")

# Identify platform (top plate) mesh
if platform_mesh_name is None:
    platform_meshes = [m for m, b in mesh_to_bone.items() if b == control_bone_name]
    if platform_meshes:
        platform_mesh_name = sanitize_gltf_name(platform_meshes[0])

if platform_mesh_name:
    print(f"  Platform mesh: {platform_mesh_name}")
else:
    print(f"  WARNING: No mesh parented to control bone '{control_bone_name}'")
    print(f"           Set PLATFORM_MESH explicitly.")

# Identify base mesh (largest unparented mesh)
if base_mesh_name is None and unparented_meshes:
    best, best_vol = None, 0
    for name in unparented_meshes:
        obj = bpy.data.objects[name]
        d = obj.dimensions
        vol = d.x * d.y * d.z
        if vol > best_vol:
            best_vol = vol
            best = name
    if best:
        base_mesh_name = sanitize_gltf_name(best)

if base_mesh_name:
    print(f"  Base mesh: {base_mesh_name}")
else:
    print(f"  WARNING: No base mesh identified. Set BASE_MESH explicitly.")


# ── Step 5: Extract platform limits ──────────────────────────────────────────
# Read Limit Location / Limit Rotation constraints on the control bone.
# Axis mapping (displacement from rest):
#   Blender X → robot X (mm)    Blender X rot → robot Rx (deg)
#   Blender Y → robot Y (mm)    Blender Y rot → robot Ry (deg)
#   Blender Z → robot Z (mm)    Blender Z rot → robot Rz (deg)

print("\nStep 5: Extracting platform limits...")

limits = {k: list(v) for k, v in DEFAULT_LIMITS.items()}
found_limits = False

control_pose = control_arm_obj.pose
control_pb = control_pose.bones.get(control_bone_name)
if control_pb:
    for con in control_pb.constraints:
        if con.type == 'LIMIT_LOCATION' and not con.mute:
            found_limits = True
            if con.use_min_x or con.use_max_x:
                limits['x'] = [round(con.min_x * 1000, 2), round(con.max_x * 1000, 2)]
            if con.use_min_y or con.use_max_y:
                limits['y'] = [round(con.min_y * 1000, 2), round(con.max_y * 1000, 2)]
            if con.use_min_z or con.use_max_z:
                limits['z'] = [round(con.min_z * 1000, 2), round(con.max_z * 1000, 2)]
            print(f"  Location limits: x={limits['x']} y={limits['y']} z={limits['z']}")

        if con.type == 'LIMIT_ROTATION' and not con.mute:
            found_limits = True
            if con.use_limit_x:
                limits['rx'] = [round(math.degrees(con.min_x), 2), round(math.degrees(con.max_x), 2)]
            if con.use_limit_y:
                limits['ry'] = [round(math.degrees(con.min_y), 2), round(math.degrees(con.max_y), 2)]
            if con.use_limit_z:
                limits['rz'] = [round(math.degrees(con.min_z), 2), round(math.degrees(con.max_z), 2)]
            print(f"  Rotation limits: rx={limits['rx']} ry={limits['ry']} rz={limits['rz']}")

if not found_limits:
    print(f"  No Limit Location/Rotation constraints on '{control_bone_name}'")
    print(f"  Using defaults: {limits}")
else:
    print(f"  Final limits: {limits}")


# ── Step 6: Export GLB (skin-free) ───────────────────────────────────────────

print("\nStep 6: Exporting GLB...")

glb_path = os.path.join(PROJECT_DIR, glb_file)

# Hide render-hidden meshes from viewport during export
visibility_backup = []
for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.hide_render and not obj.hide_viewport:
        visibility_backup.append(obj)
        obj.hide_viewport = True

# Unparent bone-parented meshes to prevent skins in GLB
parenting_backup = []
for obj in bpy.data.objects:
    if obj.type != 'MESH':
        continue
    if obj.parent and obj.parent.type == 'ARMATURE' and obj.parent_bone:
        parenting_backup.append({
            'obj': obj,
            'parent': obj.parent,
            'parent_bone': obj.parent_bone,
            'parent_type': obj.parent_type,
            'matrix_world': obj.matrix_world.copy(),
        })
        mat = obj.matrix_world.copy()
        obj.parent = None
        obj.matrix_world = mat

bpy.ops.export_scene.gltf(
    filepath=glb_path,
    export_format='GLB',
    export_apply=True,
    use_visible=True,
    export_yup=True,
    export_animations=False,
    export_skins=False,
)

# Restore parenting
for info in parenting_backup:
    obj = info['obj']
    obj.parent = info['parent']
    obj.parent_bone = info['parent_bone']
    obj.parent_type = info['parent_type']
    obj.matrix_world = info['matrix_world']

# Restore visibility
for obj in visibility_backup:
    obj.hide_viewport = False

file_size = os.path.getsize(glb_path) / (1024 * 1024)
print(f"  Exported {glb_path} ({file_size:.1f} MB)")
print(f"  Unparented/restored {len(parenting_backup)} meshes")
print(f"  Hidden/restored {len(visibility_backup)} render-hidden meshes")


# ── Step 7: Generate config JSON ─────────────────────────────────────────────

print("\nStep 7: Generating config JSON...")

config_legs = []
for ld in leg_data:
    entry = {
        'basePivot': ld['basePivot'],
        'platformPivotLocal': ld['platformPivotLocal'],
    }
    if ld['lowerMesh']:
        entry['lowerMesh'] = ld['lowerMesh']
    if ld['upperMesh']:
        entry['upperMesh'] = ld['upperMesh']
    config_legs.append(entry)

config = {
    'name': device_name,
    'type': 'hexapod',
    'model': glb_file,
    'platform': {
        'mesh': platform_mesh_name or 'TopPlate',
        'restPosition': platform_rest_pos,
    },
    'base': {
        'mesh': base_mesh_name or 'BasePlate',
    },
    'legs': config_legs,
    'limits': limits,
    'demoPose': DEFAULT_DEMO_POSE,
}

config_path = os.path.join(PROJECT_DIR, config_file)
with open(config_path, 'w') as f:
    f.write(compact_json(config))
    f.write('\n')

print(f"  Wrote {config_path}")
print(f"  {len(config_legs)} legs")


# ── Step 8: Register in panel.js ─────────────────────────────────────────────

print("\nStep 8: Checking panel.js registration...")

panel_path = os.path.join(PROJECT_DIR, 'js', 'panel.js')
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
                   f"\n    '{config_file}'\n" + match.group(2) +
                   panel_src[match.end():])
        with open(panel_path, 'w') as f:
            f.write(new_src)
        print(f"  Added '{config_file}' to configFiles in panel.js")
    else:
        print(f"  WARNING: Could not find configFiles array — add '{config_file}' manually")


# ── Summary ──────────────────────────────────────────────────────────────────

print(f"\n{'='*60}")
print(f"Hexapod import complete!")
print(f"{'='*60}")
print(f"  Config:    {config_file}")
print(f"  GLB:       {glb_file} ({file_size:.1f} MB)")
print(f"  Legs:      {len(legs)}")
print(f"  Platform:  {platform_mesh_name or '(none)'}")
print(f"  Base:      {base_mesh_name or '(none)'}")
print()

print("Leg details:")
for i, ld in enumerate(leg_data):
    print(f"  Leg {i+1}: {ld['lower_bone']} / {ld['upper_bone']}")
    print(f"    basePivot:         {ld['basePivot']}")
    print(f"    platformPivotLocal: {ld['platformPivotLocal']}")
    print(f"    meshes: {ld.get('lowerMesh', '?')}, {ld.get('upperMesh', '?')}")

print(f"\nLimits:")
for k, v in limits.items():
    unit = 'mm' if k in ('x', 'y', 'z') else 'deg'
    print(f"  {k:>2}: [{v[0]:>8}, {v[1]:>7}] {unit}")

print(f"\nTo test, open:")
print(f"  http://localhost:8000/threejs_scene.html?config={config_file}")

print(f"\nReview checklist:")
print(f"  1. All meshes load and appear correctly")
print(f"  2. Platform translation sliders move the platform")
print(f"  3. Rotation sliders tilt the platform correctly")
print(f"  4. Legs track correctly (no pass-through)")
print(f"  5. Demo pose button works")
print(f"  6. FK solver (leg length sliders) converges")
print(f"  7. Drag Platform mode works with TransformControls")

if not found_limits:
    print(f"\n⚠  No Limit constraints found on '{control_bone_name}'.")
    print(f"   Add Limit Location + Limit Rotation constraints in Blender,")
    print(f"   or edit the limits in {config_file} manually.")

print(f"{'='*60}")
