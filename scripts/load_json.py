import bpy
import os
import numpy as np
import json


# SCENE_FILE = os.path.join("E:\\", "box3d", "build-debug", "test", "scene.json")
SCENE_FILE = os.path.join("E:\\", "box2d", "build-debug", "bin", "scene.json")

START_FRAME = 1

CAMERA_ORTHOGONAL = True
CAMERA_POSITION = [15, 15, 25]
CAMERA_LOOK_AT = [-5, -5, 0]
CAMERA_UP = [0, 0, 1]

MAIN_AREA_LIGHT_POSITION = [-10, 10, 10]
MAIN_AREA_LIGHT_LOOK_AT = [0, 0, 0]


def rotation(position, look_at, up):
    f = np.array(position) - np.array(look_at)
    f = f / np.linalg.norm(f) # normalized

    r = np.cross(up, f)

    u = np.cross(f, r)

    R = np.array([
        [r[0], r[1], r[2]],
        [u[0], u[1], u[2]],
        [f[0], f[1], f[2]]
    ]).transpose()

    sy = np.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(R[2, 0], sy)
        z = 0

    return [x, y, z]


with open(SCENE_FILE, 'r') as f:
    scene = json.load(f)

# get current scene
current_scene = bpy.context.scene

# delete all objects in the scene
for obj in current_scene.objects:
    obj.hide_viewport = False
    obj.select_set(True)
bpy.ops.object.delete()

meta = scene['meta']
frames = scene['frames']

for fixture in meta:
    if fixture['type'] == 'sphere':
        bpy.ops.mesh.primitive_uv_sphere_add(radius=fixture['radius'])
        sphere = bpy.context.object
        sphere.name = fixture['name']
    if fixture['type'] == 'cube':
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        cube = bpy.context.object
        cube.scale = (fixture['hf'][0], fixture['hf'][1], fixture['hf'][2])
        cube.name = fixture['name']
    if fixture['type'] == 'plane':
        bpy.ops.mesh.primitive_plane_add(size=1.0)
        plane = bpy.context.object
        plane.scale = (fixture['hf'][0], fixture['hf'][1], 1)
        plane.name = fixture['name']
    bpy.ops.object.shade_smooth()


for blender_frame, engine_frame in enumerate(frames, start=START_FRAME):
    for obj_name, pose in engine_frame.items():
        obj = bpy.data.objects[obj_name]

        obj.location = pose['position']
        obj.rotation_quaternion  = pose['quaternion']
        obj.keyframe_insert(data_path="location", frame=blender_frame)
        obj.keyframe_insert(data_path="rotation_quaternion", frame=blender_frame)

################################ add camera ################################

if CAMERA_ORTHOGONAL:
    camera_rotation = [0, 0, 0]
else:
    camera_rotation = rotation(CAMERA_POSITION, CAMERA_LOOK_AT, CAMERA_UP)

bpy.ops.object.camera_add(location=CAMERA_POSITION, rotation=camera_rotation)
camera_object = bpy.context.object

if CAMERA_ORTHOGONAL:
    camera_object.data.type = 'ORTHO'
    camera_object.data.ortho_scale = 100
    camera_rotation = [0, 0, 0]
    camera_position = [0, 10, 30]
else:
    camera_rotation = rotation(CAMERA_POSITION, CAMERA_LOOK_AT, CAMERA_UP)
    camera_position = CAMERA_POSITION

camera_object.rotation_euler = camera_rotation
camera_object.location = camera_position

bpy.context.scene.camera = camera_object

############################# add world light #############################

world = bpy.context.scene.world
world.use_nodes = True
nodes = world.node_tree.nodes
links = world.node_tree.links
# clear current nodes
nodes.clear()

# add background node
bg_node = nodes.new(type='ShaderNodeBackground')
# add environment texture node
env_texture_node = nodes.new(type='ShaderNodeTexEnvironment')
env_texture_node.location = -300, 0

# setup background light
script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
hdri_image_path = os.path.join(script_dir, "background.jpg")
env_texture_node.image = bpy.data.images.load(hdri_image_path)

# add output node
output_node = nodes.new(type='ShaderNodeOutputWorld')
output_node.location = 200, 0

links.new(env_texture_node.outputs['Color'], bg_node.inputs['Color'])
links.new(bg_node.outputs['Background'], output_node.inputs['Surface'])

# set transparant 
current_scene.render.film_transparent = True

########################### add main area light ###########################

bpy.ops.object.light_add(type='AREA', location=MAIN_AREA_LIGHT_POSITION)

main_al_rotation = bpy.context.object
main_al_rotation.data.size = 5
main_al_rotation.data.energy = 1000
main_al_rotation.rotation_euler = rotation(MAIN_AREA_LIGHT_POSITION, MAIN_AREA_LIGHT_LOOK_AT, [0, 0, 1])

#############################################################################

current_scene.render.engine = 'CYCLES'
current_scene.cycles.device = 'GPU'
current_scene.cycles.samples = 2048