import bpy
import csv
from mathutils import *
from math import *

from eulerdiff import *

# this script writes the position and orientation of the camera
# at every frame into a CSV file named 'camera_trajectory.csv'
# 
# execute script once to register handler (or whenever you change the code)
#
# WARNING:
# This script doesn't work correctly if camera isn't at the root!
# (for example, when it is parented to something or following a path)

def writeIntrinsics():
    cam = bpy.context.scene.camera
    r = bpy.context.scene.render
    with open(bpy.path.abspath('//camera_intrinsics.csv'), 'w', newline='') as csvfile:
        intr_file = csv.writer(csvfile, delimiter=',')
        intr_file.writerow([ \
            'focal length', \
            'focal length mm', \
            'image width', \
            'image height', \
            'sensor width mm', \
            'sensor height mm', \
            'clip start', \
            'clip end', \
            'color depth', \
            'depth depth'])
            
        W = r.resolution_x * r.resolution_percentage / 100
        H = r.resolution_y * r.resolution_percentage / 100
        
        focal = cam.data.lens * W / cam.data.sensor_width
            
        intr_file.writerow([ \
            focal, \
            cam.data.lens, \
            W, \
            H, \
            cam.data.sensor_width, \
            cam.data.sensor_height, \
            cam.data.clip_start, \
            cam.data.clip_end, \
            255, \
            255])

def RunPerFrame(scene):
    
    global previous_position
    global previous_rotation
    
    cam = bpy.context.scene.camera
    
    current_position = cam.location.copy()
    current_rotation = cam.rotation_euler.copy()
    
    print("current frame:", scene.frame_current)
    print("position", current_position)
    print("rotation", current_rotation)

    if scene.frame_current == scene.frame_start:
        # overwrite existing CSV file and write CSV header
        # x y z alpha beta gamma
        with open(bpy.path.abspath('//camera_trajectory_relative.csv'), 'w', newline='') as csvfile:
            posfile = csv.writer(csvfile, delimiter=',')
            posfile.writerow(['deltaX', 'deltaY', 'deltaZ', 'delta_alpha', 'delta_beta', 'delta_gamma'])
    
    # write position and rotation relative to previous frame
    with open(bpy.path.abspath('//camera_trajectory_relative.csv'), 'a', newline='') as csvfile:
        posfile = csv.writer(csvfile, delimiter=',')
        
        #print(previous_position)
        #print(previous_rotation)
        #print(current_position)
        #print(current_rotation)
        
        # calculate relative rotation
        # get inverse of previous rotation
        prev_rot_inv = previous_rotation.to_matrix()
        prev_rot_inv.transpose()
        # apply it to current rotation (i.e. new-old)
        delta_rot = euler_difference(previous_rotation, current_rotation)
        
        # calculate relative movement
        delta_pos = current_position - previous_position # in global frame
        delta_pos.rotate(prev_rot_inv) # in camera frame
        delta_pos = blender_to_matlab_transl(delta_pos) # in camera frame used by Matlab code
        
        print('delta position:', delta_pos)
        print('delta rotation:', euler_to_degstring(delta_rot), delta_rot)
        
        previous_position = current_position;
        previous_rotation = current_rotation;
        
        posfile.writerow([ \
            delta_pos.x, delta_pos.y, delta_pos.z, \
            delta_rot.x, delta_rot.y, delta_rot.z])


# remove function handler from previous execution of this script
bpy.app.handlers.render_post.clear()

# execute RunPerFrame on every frame
bpy.app.handlers.render_post.append(RunPerFrame)

previous_position = bpy.context.scene.camera.location.copy()
previous_rotation = bpy.context.scene.camera.rotation_euler.copy()

writeIntrinsics()
