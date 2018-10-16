import os
import numpy as np
import pybullet as pb
import pybullet_data
from bulletwrapper import BulletHook, ObjectInfo
from bulletwrapper.util.bullet_util import add_obj

class GroundPlaneHook(BulletHook):

    def after_reset(self, sim):
        path_to_plane = os.path.join( pybullet_data.getDataPath(), 'plane.urdf' )
        plane_id = pb.loadURDF( path_to_plane )

        return ObjectInfo(path_to_plane, plane_id)


class OBJCreatorHook(BulletHook):
    '''
    Create a custom pybullet object at a given time and position.
    NOTE: pybullet only supports .obj file for custom object model. (See pb.createCollisionShape().)
    '''
    
    default_add_obj_kwargs = {
        'R': np.eye(3),
        'fix_base': False,
        'base_mass': .01,
        'base_inertialframe_position': np.zeros(3),
        'scale': 1.,
    }

    def __init__(self, path_to_obj, position, time_to_create=0., **kwargs):

        add_obj_kwargs = self.default_add_obj_kwargs.copy()
        for kw, arg in kwargs.items():
            assert kw in self.default_add_obj_kwargs
            add_obj_kwargs[kw] = arg

        add_obj_kwargs['path_to_obj'] = path_to_obj
        add_obj_kwargs['t'] = position

        self.path_to_obj = path_to_obj
        self.time_to_create = time_to_create
        self.add_obj_kwargs = add_obj_kwargs
        self.created = False

    def create(self):
        body_id = add_obj(**self.add_obj_kwargs)
        self.created = True

        return body_id


    def after_reset(self, sim):
        if sim.sim_time >= self.time_to_create:
            body_id = self.create()
            obj_info = ObjectInfo( self.path_to_obj, body_id )
            sim.objects.append(obj_info)
            return obj_info


    def after_step(self, sim, hooks_output):
        if not self.created and sim.sim_time >= self.time_to_create:
            body_id = self.create()
            obj_info = ObjectInfo( self.path_to_obj, body_id )
            sim.objects.append(obj_info)
            return obj_info

