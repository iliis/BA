import bpy
import csv

# this script writes the position and orientation of the camera
# at every frame into a CSV file named 'camera_trajectory.csv'
# 
# execute script once to register handler (or whenever you change the code)

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
            65535])

def RunPerFrame(scene):
    
    global previous_position
    global previous_rotation
    
    cam = bpy.context.scene.camera
    print("current frame:", scene.frame_current)
    print("position", cam.location)
    print("rotation", cam.rotation_euler)
    
    if scene.frame_current == scene.frame_start:
        # overwrite existing CSV file and write CSV header
        # x y z alpha beta gamma
        with open(bpy.path.abspath('//camera_trajectory.csv'), 'w', newline='') as csvfile:
            posfile = csv.writer(csvfile, delimiter=',')
            posfile.writerow(['X', 'Y', 'Z', 'alpha', 'beta', 'gamma'])
        
        with open(bpy.path.abspath('//camera_trajectory_relative.csv'), 'w', newline='') as csvfile:
            posfile = csv.writer(csvfile, delimiter=',')
            posfile.writerow(['deltaX', 'deltaY', 'deltaZ', 'delta_alpha', 'delta_beta', 'delta_gamma'])
   
    with open(bpy.path.abspath('//camera_trajectory.csv'), 'a', newline='') as csvfile:
        posfile = csv.writer(csvfile, delimiter=',')
        posfile.writerow([-cam.location.x, cam.location.y, cam.location.z, \
        -cam.rotation_euler.x, cam.rotation_euler.y, cam.rotation_euler.z])
    
    # write position and rotation relative to previous frame
    with open(bpy.path.abspath('//camera_trajectory_relative.csv'), 'a', newline='') as csvfile:
        posfile = csv.writer(csvfile, delimiter=',')
        
        current_position = cam.location.copy()
        current_rotation = cam.rotation_euler.copy()
        
        print(previous_position)
        print(previous_rotation)
        print(current_position)
        print(current_rotation)
        
        pos = current_position - previous_position
        rot = current_rotation.copy()
        rot.rotate(previous_rotation)
        
        print(pos)
        print(rot)
        
        previous_position = current_position;
        previous_rotation = current_rotation;
        
        posfile.writerow([pos.x, pos.y, pos.z, \
        rot.x, rot.y, rot.z])
    
    print("rendered a frame!")



# remove function handler from previous execution of this script
bpy.app.handlers.render_post.clear()

# execute RunPerFrame on every frame
bpy.app.handlers.render_post.append(RunPerFrame)

previous_position = bpy.context.scene.camera.location.copy()
previous_rotation = bpy.context.scene.camera.rotation_euler.copy()

writeIntrinsics()