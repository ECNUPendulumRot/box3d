import bpy
import os
import numpy as np
import json

SCENE_FILE = os.path.join("E:\\", "box3d", "build-debug", "test", "scene.json")
START_FRAME = 1

CAMERA_POSITION = [15, 15, 25]
CAMERA_LOOK_AT = [-5, -5, 0]
CAMERA_UP = [0, 0, 1]

MAIN_AREA_LIGHT_POSITION = [-10, 10, 10]
MAIN_AREA_LIGHT_LOOK_AT = [0, 0, 0]

with open(SCENE_FILE, 'r') as f:
    scene = json.load(f)

print(scene)
