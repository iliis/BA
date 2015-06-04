import bpy
from math import *
from mathutils import *

def euler_to_degstring(rot):
    s = 'x=%.2f°, y=%.2f°, z=%.2f°' % (degrees(rot.x), degrees(rot.y), degrees(rot.z))
    return s

blender_to_matlab_rot = Euler((radians(180),0,0),'ZYX')

class HelloWorldPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_hello_world"
    bl_label = "Hello World"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"
    
    def draw(self, context):
        
        S = bpy.context.scene
        
        self.layout.label(text="current:")
        self.layout.label(text=str(bpy.context.scene.camera.location))
        r = S.camera.rotation_euler
        self.layout.label(euler_to_degstring(r))
        
        current_frame = S.frame_current
        
        current_position = S.camera.location.copy()
        current_rotation = r.copy()
        
        S.frame_set(current_frame - 1)
        self.layout.label("previous:")
        self.layout.label(str(S.camera.location))
        self.layout.label(euler_to_degstring(S.camera.rotation_euler))
        
        
        delta_rot = current_rotation
        old_rot = S.camera.rotation_euler.to_matrix().copy()
        old_rot.transpose()
        delta_rot.rotate(old_rot)
        
        delta_pos = current_position - S.camera.location
        delta_pos.rotate(old_rot)
        delta_pos.rotate(blender_to_matlab_rot)
        
        self.layout.label(text="delta: " + str(delta_pos))
        
        self.layout.label(text="delta: " + euler_to_degstring(delta_rot))
        
        S.frame_set(current_frame)
        


#bpy.utils.unregister_class(HelloWorldPanel)
bpy.utils.register_class(HelloWorldPanel)