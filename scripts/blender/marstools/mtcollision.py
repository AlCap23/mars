'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtcollision.py

Created on 7 Jan 2014

@author: Malte Langosz

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
from bpy.types import Operator
from bpy.props import FloatProperty, EnumProperty
import math
import mathutils
import marstools.mtmaterials as mtmaterials
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs
from mathutils import Vector

def register():
    print("Registering mtcollision...")


def unregister():
    print("Unregistering mtcollision...")

class CreateCollisionObjects(Operator):
    """Select n bodies to create collision objects for."""
    bl_idname = "object.create_collision_objects"
    bl_label = "Create collision objects for all selected Links"
    bl_options = {'REGISTER', 'UNDO'}

    property_colltype = EnumProperty(
        name = 'coll_type',
        default = 'box',
        description = "collision type",
        items = mtdefs.geometrytypes)

    def execute(self, context):

        visuals = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "visual":
                visuals.append(obj)
            obj.select = False

        if visuals == []:
            bpy.ops.error.message('INVOKE_DEFAULT', type="CreateCollisions Error", message="Not enough bodies selected.")
            return{'CANCELLED'}
        for vis in visuals:
            nameparts = vis.name.split('_')
            if nameparts[0] == 'visual':
                nameparts[0] = 'collision'
            collname = '_'.join(nameparts)
            materialname = vis.data.materials[0].name if len(vis.data.materials) > 0 else "None"
            bBox = vis.bound_box
            center = mtutility.calcBoundingBoxCenter(bBox)
            rotation = mathutils.Matrix.Identity(4)
            size = list(vis.dimensions)
            if self.property_colltype in ['cylinder', 'capsule']:
                axes = ('X', 'Y', 'Z')
                long_side = axes[size.index(max(size))]
                #xyequal = (size[0] - size[1])
                height = max(size)
                radii = [s for s in size if s != height]
                radius = max(radii)/2 if radii != [] else height/2
                size = (radius, height)
                if long_side == 'X':
                    rotation = mathutils.Matrix.Rotation(math.pi/2, 4, 'Y')
                elif long_side == 'Y':
                    rotation = mathutils.Matrix.Rotation(math.pi/2, 4, 'X')
                #FIXME: apply rotation for moved cylinder object?
            elif self.property_colltype == 'sphere':
                size = max(size)/2
            rotation_euler = (vis.matrix_world*rotation).to_euler()
            center = vis.matrix_world.to_translation() + vis.matrix_world.to_quaternion()*center
            if self.property_colltype != 'capsule':
                ob = mtutility.createPrimitive(collname, self.property_colltype, size,
                                               mtdefs.layerTypes["collision"], materialname, center,
                                               rotation_euler)
            elif self.property_colltype == 'capsule':
                height = max(height-2*radius, 0.001) #prevent height from turning negative
                size = (radius, height)
                zshift = height/2
                ob = mtutility.createPrimitive(collname, 'cylinder', size,
                               mtdefs.layerTypes["collision"], materialname, center,
                               rotation_euler)
                sph1 = mtutility.createPrimitive('tmpsph1', 'sphere', radius,
                               mtdefs.layerTypes["collision"], materialname, center+rotation*mathutils.Vector((0,0,zshift)),
                               rotation_euler)
                sph2 = mtutility.createPrimitive('tmpsph2', 'sphere', radius,
                               mtdefs.layerTypes["collision"], materialname, center-rotation*mathutils.Vector((0,0,zshift)),
                               rotation_euler)
                mtutility.selectObjects([ob, sph1, sph2], True, 0)
                bpy.ops.object.join()
                ob['height'] = height
                ob['radius'] = radius
            elif self.property_colltype == 'mesh':
                pass
                #TODO: copy mesh!!
            ob.MARStype = "collision"
            ob["geometryType"] = self.property_colltype
            if vis.parent:
                ob.select = True
                bpy.ops.object.transform_apply(scale=True)
                vis.parent.select = True
                bpy.context.scene.objects.active = vis.parent
                bpy.ops.object.parent_set()
                ob.parent_type = vis.parent_type
                ob.parent_bone = vis.parent_bone
        return{'FINISHED'}


