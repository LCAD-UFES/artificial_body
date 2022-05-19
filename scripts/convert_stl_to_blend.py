import bpy
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:] # get all args after "--"

stl_in = argv[0]
obj_out = argv[1]

bpy.ops.import_mesh.stl(filepath=stl_in, axis_forward='X', axis_up='Z')

bpy.ops.object.select_all(action='DESELECT')

bpy.data.objects['Camera'].select_set(True) # Blender 2.8x
bpy.ops.object.delete() 
bpy.data.objects['Cube'].select_set(True) # Blender 2.8x
bpy.ops.object.delete() 
bpy.data.objects['Light'].select_set(True) # Blender 2.8x
bpy.ops.object.delete() 

#https://docs.blender.org/api/current/bpy.ops.export_scene.html
bpy.ops.export_scene.obj(filepath=obj_out, axis_forward='X', axis_up='Z', global_scale=1.0, use_materials=False)
