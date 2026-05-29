import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, StringProperty
from bpy.types import Operator
from math import cos, sin
from mathutils import Vector

from .constants import FK_FIELDS, IK_FIELDS, LEG_ORDER
from .rig_utils import store_base_pose


STANDARD_LEG_NAMES = {
    "fl": {
        "upper": "front_left_upper",
        "lower": "front_left_lower",
        "foot": "front_left_foot",
        "ik": "front_left_ik",
        "pole": "front_left_pole",
    },
    "fr": {
        "upper": "front_right_upper",
        "lower": "front_right_lower",
        "foot": "front_right_foot",
        "ik": "front_right_ik",
        "pole": "front_right_pole",
    },
    "rl": {
        "upper": "rear_left_upper",
        "lower": "rear_left_lower",
        "foot": "rear_left_foot",
        "ik": "rear_left_ik",
        "pole": "rear_left_pole",
    },
    "rr": {
        "upper": "rear_right_upper",
        "lower": "rear_right_lower",
        "foot": "rear_right_foot",
        "ik": "rear_right_ik",
        "pole": "rear_right_pole",
    },
}

QUADRUPED_PROFILES = {
    "MEDIUM": {
        "label": "Medium Quadruped",
        "root": {"head": (-0.26, 0.0, 0.0), "tail": (0.26, 0.0, 0.0)},
        "body": {"head": (0.0, -0.78, 0.92), "tail": (0.0, 0.74, 1.02)},
        "spine": [
            ("pelvis", (0.0, -0.78, 0.88), (0.0, -0.36, 0.94)),
            ("spine_01", (0.0, -0.36, 0.94), (0.0, 0.16, 0.99)),
            ("chest", (0.0, 0.16, 0.99), (0.0, 0.76, 1.02)),
        ],
        "neck": {"head": (0.0, 0.76, 1.02), "tail": (0.0, 1.08, 1.16)},
        "head": {"head": (0.0, 1.08, 1.16), "tail": (0.0, 1.44, 1.08)},
        "tail": [
            ("tail_01", (0.0, -0.78, 0.88), (0.0, -1.12, 0.94)),
            ("tail_02", (0.0, -1.12, 0.94), (0.0, -1.48, 0.84)),
        ],
        "leg_width_front": 0.30,
        "leg_width_rear": 0.34,
        "front_leg": {
            "anchor_parent": "chest",
            "guide": "scapula",
            "guide_head": (0.0, 0.56, 1.03),
            "guide_tail": (0.0, 0.42, 0.80),
            "upper_head": (0.0, 0.42, 0.80),
            "upper_tail": (0.0, 0.35, 0.55),
            "lower_tail": (0.0, 0.52, 0.18),
            "foot_tail": (0.0, 0.74, 0.05),
            "pole": (0.0, -0.18, 0.55),
        },
        "rear_leg": {
            "anchor_parent": "pelvis",
            "guide": "hip",
            "guide_head": (0.0, -0.72, 0.88),
            "guide_tail": (0.0, -0.86, 0.72),
            "upper_head": (0.0, -0.86, 0.72),
            "upper_tail": (0.0, -0.78, 0.48),
            "lower_tail": (0.0, -0.58, 0.18),
            "foot_tail": (0.0, -0.36, 0.05),
            "pole": (0.0, -1.28, 0.50),
        },
        "control_scale": 1.0,
    },
    "HORSE": {
        "label": "Horse",
        "root": {"head": (-0.34, 0.0, 0.0), "tail": (0.34, 0.0, 0.0)},
        "body": {"head": (0.0, -1.08, 1.28), "tail": (0.0, 0.98, 1.42)},
        "spine": [
            ("pelvis", (0.0, -1.08, 1.24), (0.0, -0.48, 1.32)),
            ("spine_01", (0.0, -0.48, 1.32), (0.0, 0.24, 1.40)),
            ("chest", (0.0, 0.24, 1.40), (0.0, 1.02, 1.42)),
        ],
        "neck": {"head": (0.0, 1.02, 1.42), "tail": (0.0, 1.46, 1.76)},
        "head": {"head": (0.0, 1.46, 1.76), "tail": (0.0, 1.88, 1.60)},
        "tail": [
            ("tail_01", (0.0, -1.08, 1.24), (0.0, -1.38, 1.12)),
            ("tail_02", (0.0, -1.38, 1.12), (0.0, -1.70, 0.96)),
        ],
        "leg_width_front": 0.30,
        "leg_width_rear": 0.34,
        "front_leg": {
            "anchor_parent": "chest",
            "guide": "scapula",
            "guide_head": (0.0, 0.78, 1.40),
            "guide_tail": (0.0, 0.56, 1.04),
            "upper_head": (0.0, 0.56, 1.04),
            "upper_tail": (0.0, 0.48, 0.74),
            "lower_tail": (0.0, 0.58, 0.26),
            "foot_tail": (0.0, 0.78, 0.06),
            "pole": (0.0, -0.20, 0.76),
        },
        "rear_leg": {
            "anchor_parent": "pelvis",
            "guide": "hip",
            "guide_head": (0.0, -1.00, 1.24),
            "guide_tail": (0.0, -1.10, 0.94),
            "upper_head": (0.0, -1.10, 0.94),
            "upper_tail": (0.0, -0.94, 0.62),
            "lower_tail": (0.0, -0.68, 0.24),
            "foot_tail": (0.0, -0.42, 0.06),
            "pole": (0.0, -1.56, 0.66),
        },
        "control_scale": 1.15,
    },
}


def scaled(point, scale):
    """Return a Vector point multiplied by the armature scale."""
    return Vector(point) * scale


def mirrored_point(point, x):
    """Return a profile point with an assigned X coordinate."""
    return (x, point[1], point[2])


def add_edit_bone(edit_bones, name, head, tail, scale, parent=None, connected=False, deform=True):
    """Create an edit bone with common parent and deform settings."""
    bone = edit_bones.new(name)
    bone.head = scaled(head, scale)
    bone.tail = scaled(tail, scale)
    bone.parent = parent
    bone.use_connect = connected
    bone.use_deform = deform
    return bone


def get_widget_collection(context, armature_name):
    """Create a collection for generated control-shape objects."""
    collection_name = f"{armature_name}_widgets"
    collection = bpy.data.collections.get(collection_name)
    if not collection:
        collection = bpy.data.collections.new(collection_name)
        context.scene.collection.children.link(collection)
    return collection


def create_widget_object(collection, name, vertices, edges):
    """Create a hidden mesh object used as a custom bone shape."""
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, edges, [])
    mesh.update()

    obj = bpy.data.objects.new(name, mesh)
    collection.objects.link(obj)
    obj.display_type = "WIRE"
    obj.hide_render = True
    obj.hide_viewport = True
    return obj


def circle_points(radius=1.0, segments=24, y_scale=1.0):
    """Return vertices and edges for a flat circle-like widget."""
    vertices = []
    edges = []
    for index in range(segments):
        angle = (index / segments) * 6.283185307179586
        vertices.append((radius * cos(angle), radius * y_scale * sin(angle), 0.0))
        edges.append((index, (index + 1) % segments))
    return vertices, edges


def make_widget_shapes(context, armature_name):
    """Build the custom control shapes used by the generated rig."""
    collection = get_widget_collection(context, armature_name)
    square_vertices = [(-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0)]
    square_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    diamond_vertices = [(0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)]
    diamond_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    body_vertices, body_edges = circle_points(radius=1.0, segments=32, y_scale=1.6)

    return {
        "root": create_widget_object(collection, f"{armature_name}_root_shape", square_vertices, square_edges),
        "body": create_widget_object(collection, f"{armature_name}_body_shape", body_vertices, body_edges),
        "foot": create_widget_object(collection, f"{armature_name}_foot_shape", square_vertices, square_edges),
        "pole": create_widget_object(collection, f"{armature_name}_pole_shape", diamond_vertices, diamond_edges),
    }


def assign_custom_shape(pose_bone, shape, scale_xyz):
    """Assign a custom shape to a pose bone when Blender supports it."""
    pose_bone.custom_shape = shape
    if hasattr(pose_bone, "use_custom_shape_bone_size"):
        pose_bone.use_custom_shape_bone_size = False
    if hasattr(pose_bone, "custom_shape_scale_xyz"):
        pose_bone.custom_shape_scale_xyz = scale_xyz
    else:
        pose_bone.custom_shape_scale = max(scale_xyz)


def assign_control_shapes(context, armature_object, scale, control_scale=1.0):
    """Assign custom widgets to generated control bones."""
    shapes = make_widget_shapes(context, armature_object.name)
    widget_scale = scale * control_scale
    assign_custom_shape(armature_object.pose.bones["root"], shapes["root"], (0.32 * widget_scale, 0.32 * widget_scale, 0.32 * widget_scale))
    assign_custom_shape(armature_object.pose.bones["body"], shapes["body"], (0.46 * widget_scale, 0.24 * widget_scale, 0.46 * widget_scale))

    for leg in LEG_ORDER:
        names = STANDARD_LEG_NAMES[leg]
        assign_custom_shape(armature_object.pose.bones[names["ik"]], shapes["foot"], (0.17 * widget_scale, 0.12 * widget_scale, 0.17 * widget_scale))
        assign_custom_shape(armature_object.pose.bones[names["pole"]], shapes["pole"], (0.09 * widget_scale, 0.09 * widget_scale, 0.09 * widget_scale))


def assign_bone_groups(armature_object):
    """Color generated control and deform bones in Blender versions that support it."""
    if not hasattr(armature_object.pose, "bone_groups"):
        return

    control_group = armature_object.pose.bone_groups.new(name="QWalk Controls")
    control_group.color_set = "THEME09"
    deform_group = armature_object.pose.bone_groups.new(name="QWalk Deform")
    deform_group.color_set = "THEME04"

    control_names = {"root", "body"}
    for leg in LEG_ORDER:
        control_names.add(STANDARD_LEG_NAMES[leg]["ik"])
        control_names.add(STANDARD_LEG_NAMES[leg]["pole"])

    for pose_bone in armature_object.pose.bones:
        pose_bone.bone_group = control_group if pose_bone.name in control_names else deform_group


def activate_body_control(armature_object):
    """Make the body control active when Blender allows it."""
    try:
        armature_object.data.bones.active = armature_object.data.bones["body"]
    except Exception:
        pass


def apply_standard_mapping(settings):
    """Fill scene settings with the generated armature's bone names."""
    settings.root_bone = "root"
    settings.body_bone = "body"

    for leg in LEG_ORDER:
        names = STANDARD_LEG_NAMES[leg]
        setattr(settings, IK_FIELDS[leg], names["ik"])
        upper, lower, foot = FK_FIELDS[leg]
        setattr(settings, upper, names["upper"])
        setattr(settings, lower, names["lower"])
        setattr(settings, foot, names["foot"])


def create_standard_quadruped(
    context,
    name,
    scale=1.0,
    add_ik_constraints=True,
    display_type="STICK",
    profile_key="MEDIUM",
):
    """Create and return a named quadruped armature object."""
    if context.object and context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    profile = QUADRUPED_PROFILES.get(profile_key, QUADRUPED_PROFILES["MEDIUM"])

    armature_data = bpy.data.armatures.new(name)
    armature_object = bpy.data.objects.new(name, armature_data)
    context.collection.objects.link(armature_object)

    bpy.ops.object.select_all(action="DESELECT")
    armature_object.select_set(True)
    context.view_layer.objects.active = armature_object

    armature_data.display_type = display_type
    armature_object.show_in_front = True

    bpy.ops.object.mode_set(mode="EDIT")
    bones = armature_data.edit_bones

    root = add_edit_bone(
        bones,
        "root",
        profile["root"]["head"],
        profile["root"]["tail"],
        scale,
        deform=False,
    )
    body = add_edit_bone(
        bones,
        "body",
        profile["body"]["head"],
        profile["body"]["tail"],
        scale,
        root,
        deform=False,
    )
    chain_parent = body
    named_bones = {"root": root, "body": body}
    for index, (bone_name, head, tail) in enumerate(profile["spine"]):
        bone = add_edit_bone(bones, bone_name, head, tail, scale, chain_parent, connected=index > 0)
        named_bones[bone_name] = bone
        chain_parent = bone

    neck = add_edit_bone(
        bones,
        "neck",
        profile["neck"]["head"],
        profile["neck"]["tail"],
        scale,
        named_bones["chest"],
        connected=True,
    )
    named_bones["neck"] = neck
    named_bones["head"] = add_edit_bone(
        bones,
        "head",
        profile["head"]["head"],
        profile["head"]["tail"],
        scale,
        neck,
        connected=True,
    )

    tail_parent = named_bones["pelvis"]
    for index, (bone_name, head, tail) in enumerate(profile["tail"]):
        tail_parent = add_edit_bone(bones, bone_name, head, tail, scale, tail_parent, connected=index > 0)
        named_bones[bone_name] = tail_parent

    leg_parents = {"fl": named_bones["chest"], "fr": named_bones["chest"], "rl": named_bones["pelvis"], "rr": named_bones["pelvis"]}
    for leg in LEG_ORDER:
        names = STANDARD_LEG_NAMES[leg]
        side = 1.0 if leg.endswith("l") else -1.0
        is_front = leg.startswith("f")
        leg_profile = profile["front_leg"] if is_front else profile["rear_leg"]
        x = side * (profile["leg_width_front"] if is_front else profile["leg_width_rear"])

        guide_name = f"{names['upper'].rsplit('_', 1)[0]}_{leg_profile['guide']}"
        guide = add_edit_bone(
            bones,
            guide_name,
            mirrored_point(leg_profile["guide_head"], x),
            mirrored_point(leg_profile["guide_tail"], x),
            scale,
            leg_parents[leg],
        )

        upper = add_edit_bone(
            bones,
            names["upper"],
            mirrored_point(leg_profile["upper_head"], x),
            mirrored_point(leg_profile["upper_tail"], x),
            scale,
            guide,
            True,
        )
        lower = add_edit_bone(
            bones,
            names["lower"],
            mirrored_point(leg_profile["upper_tail"], x),
            mirrored_point(leg_profile["lower_tail"], x),
            scale,
            upper,
            True,
        )
        add_edit_bone(
            bones,
            names["foot"],
            mirrored_point(leg_profile["lower_tail"], x),
            mirrored_point(leg_profile["foot_tail"], x),
            scale,
            lower,
            True,
        )
        add_edit_bone(
            bones,
            names["ik"],
            mirrored_point((0.0, leg_profile["foot_tail"][1] - 0.11, leg_profile["foot_tail"][2]), x),
            mirrored_point((0.0, leg_profile["foot_tail"][1] + 0.11, leg_profile["foot_tail"][2]), x),
            scale,
            root,
            deform=False,
        )
        add_edit_bone(
            bones,
            names["pole"],
            mirrored_point((0.0, leg_profile["pole"][1], leg_profile["pole"][2]), x - 0.07 * side),
            mirrored_point((0.0, leg_profile["pole"][1], leg_profile["pole"][2]), x + 0.07 * side),
            scale,
            root,
            deform=False,
        )

    bpy.ops.object.mode_set(mode="POSE")
    for pose_bone in armature_object.pose.bones:
        pose_bone.rotation_mode = "XYZ"
    store_base_pose(armature_object)

    if add_ik_constraints:
        for leg in LEG_ORDER:
            names = STANDARD_LEG_NAMES[leg]
            constraint = armature_object.pose.bones[names["foot"]].constraints.new(type="IK")
            constraint.name = "QWalk IK"
            constraint.target = armature_object
            constraint.subtarget = names["ik"]
            constraint.pole_target = armature_object
            constraint.pole_subtarget = names["pole"]
            constraint.chain_count = 3
            constraint.iterations = 24

    assign_control_shapes(context, armature_object, scale, profile.get("control_scale", 1.0))
    assign_bone_groups(armature_object)
    activate_body_control(armature_object)
    return armature_object


class QWG_OT_create_quadruped_armature(Operator):
    bl_idname = "qwg.create_quadruped_armature"
    bl_label = "Create Quadruped Armature"
    bl_description = "Create a starter quadruped armature compatible with QWalk"
    bl_options = {"REGISTER", "UNDO"}

    armature_name: StringProperty(
        name="Name",
        description="Name for the generated armature object and data",
        default="QWalk_Quadruped",
    )
    scale: FloatProperty(
        name="Scale",
        description="Overall size multiplier for the generated armature",
        default=1.0,
        min=0.01,
        soft_max=10.0,
    )
    body_profile: EnumProperty(
        name="Profile",
        description="Proportion template for the generated quadruped",
        items=(
            ("MEDIUM", "Medium Quadruped", "Dog/cat-like general quadruped proportions"),
            ("HORSE", "Horse", "Longer body, neck, and limb proportions"),
        ),
        default="MEDIUM",
    )
    add_ik_constraints: BoolProperty(
        name="Add IK Constraints",
        description="Add IK constraints from lower-leg bones to generated IK targets",
        default=True,
    )
    display_type: EnumProperty(
        name="Display",
        description="Viewport display style for the generated armature",
        items=(
            ("STICK", "Stick", "Clean rig-style display"),
            ("OCTAHEDRAL", "Octahedral", "Classic Blender bone shapes"),
            ("BBONE", "B-Bone", "Thick bendy-bone shapes"),
            ("WIRE", "Wire", "Wireframe bone shapes"),
        ),
        default="STICK",
    )
    map_after_create: BoolProperty(
        name="Map for QWalk",
        description="Fill QWalk bone mapping fields after creating the armature",
        default=True,
    )

    def execute(self, context):
        """Create the standard armature and optionally map it for QWalk."""
        armature = create_standard_quadruped(
            context,
            self.armature_name,
            self.scale,
            self.add_ik_constraints,
            self.display_type,
            self.body_profile,
        )

        if self.map_after_create:
            apply_standard_mapping(context.scene.qwg_settings)

        self.report({"INFO"}, f"Created quadruped armature {armature.name}.")
        return {"FINISHED"}
