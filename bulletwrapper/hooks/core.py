import os
import numpy as np
import pybullet as pb
import pybullet_data
from bulletwrapper import BulletHook
from bulletwrapper.util.bullet_util import add_obj

class GroundPlaneHook(BulletHook):

    def after_reset(self, pb_state):
        path_to_plane = pybullet_data.getDataPath()
        plane_id = pb.loadURDF(os.path.join( path_to_plane, 'plane.urdf'))


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
    }

    def __init__(self, path_to_obj, position, time_to_create=0., **kwargs):

        add_obj_kwargs = self.default_add_obj_kwargs.copy()
        for kw, arg in kwargs.items():
            assert kw in default_add_obj_kwargs
            add_obj_kwargs[kw] = arg

        add_obj_kwargs['path_to_obj'] = path_to_obj
        add_obj_kwargs['t'] = position

        self.time_to_create = time_to_create
        self.add_obj_kwargs = add_obj_kwargs
        self.created = False

    def create(self):
        body_id = add_obj(**self.add_obj_kwargs)
        self.created = True

        return body_id


    def after_reset(self, pb_state):
        if pb_state.sim_time >= self.time_to_create:
            body_id = self.create()
            return body_id


    def after_step(self, pb_state, step_output):
        if not self.created and pb_state.sim_time >= self.time_to_create:
            body_id = self.create()
            return body_id

