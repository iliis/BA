import bpy
from math import *
from mathutils import *

def euler_to_degstring(rot):
    s = 'x=%.2f°, y=%.2f°, z=%.2f°' % (degrees(rot.x), degrees(rot.y), degrees(rot.z))
    return s

def blender_to_matlab_rot(rot):
    rot = rot.copy()
    rot.rotate_axis('X', radians(180))
    return rot

def blender_to_matlab_transl(transl):
    transl = transl.copy()
    transl.rotate(Euler((radians(180),0,0),'ZYX'))
    return transl

def euler_difference(old_rot, new_rot):
    
    old_rot = blender_to_matlab_rot(old_rot)
    new_rot = blender_to_matlab_rot(new_rot)
    
    old_mat_inv = old_rot.to_matrix()
    old_mat_inv.transpose()
    
    delta = new_rot.copy()
    delta.rotate(old_mat_inv)
    
    return delta


#e1 = bpy.context.selected_objects[0].rotation_euler
#e2 = bpy.context.selected_objects[1].rotation_euler