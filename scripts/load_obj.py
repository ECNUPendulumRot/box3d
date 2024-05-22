import bpy
import os
import numpy as np

OBJ_DIR = os.path.join("F:\\", "box3d", "build-debug", "test", "obj_files")
START_FRAME = 1

CAMERA_POSITION = [15, 15, 25]
CAMERA_LOOK_AT = [-5, -5, 0]
CAMERA_UP = [0, 0, 1]


# setup camera
def camera_rotation():
    f = np.array(CAMERA_POSITION) - np.array(CAMERA_LOOK_AT)
    f = f / np.linalg.norm(f) # normalized

    r = np.cross(CAMERA_UP, f)

    u = np.cross(f, r)

    R = np.array([
        [r[0], r[1], r[2]],
        [u[0], u[1], u[2]],
        [f[0], f[1], f[2]]
    ]).transpose()

    print(R)

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


def load_objs(scene):
    obj_filenames = [f for f in os.listdir(OBJ_DIR) if f.endswith('.obj')]

    for obj_file in obj_filenames:
        bpy.ops.wm.obj_import(filepath=os.path.join(OBJ_DIR, obj_file), up_axis='Z')

    obj_list = sorted(scene.objects, key=lambda obj: obj.name)  # Sort objects by name

    return obj_list

bpy.ops.object.camera_add
current_scene = bpy.context.scene

# Delete all objects in the scene
for obj in current_scene.objects:
    obj.hide_viewport = False
    obj.select_set(True)
bpy.ops.object.delete()

# load objects
obj_list = load_objs(scene=current_scene)  

# setup frames
current_scene.frame_start = START_FRAME
current_scene.frame_end = START_FRAME + len(obj_list) - 1

# Hide all objects at first time
for frame in range(current_scene.frame_start, current_scene.frame_end + 1):
    for obj in obj_list:
        obj.hide_render = True
        obj.hide_viewport = True
        obj.keyframe_insert(data_path='hide_render', frame=frame)
        obj.keyframe_insert(data_path='hide_viewport', frame=frame)
        obj.keyframe_insert(data_path='hide_render', frame=frame)
        obj.keyframe_insert(data_path='hide_viewport', frame=frame)


for i, obj in enumerate(obj_list):
    current_scene.frame_set(START_FRAME + i)
    obj.hide_render = False
    obj.hide_viewport = False
    obj.keyframe_insert(data_path='hide_render', frame=START_FRAME + i)
    obj.keyframe_insert(data_path='hide_viewport', frame=START_FRAME + i)

for obj in obj_list:
    if obj.type == 'Mesh':
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.shade_smooth()
        obj.select_set(False)

current_scene.frame_set(START_FRAME)

# add camera
rotation = camera_rotation()
print(rotation)
bpy.ops.object.camera_add(location=CAMERA_POSITION, rotation=rotation)
camera_object = bpy.context.object
bpy.context.scene.camera = camera_object