import numpy as np
from bulletwrapper import BulletHook
from bulletwrapper.hooks.core import _make_low_high, _identity
from .pack_box import make_packed_cuboids_scene
from functools import partial
from bulletwrapper.util.bullet_util import add_obj
from bulletwrapper import ObjectInfo

import time

def random_rectangle( width, height):
    w_low, w_high = _make_low_high(width)
    h_low, h_high = _make_low_high(height)

    W = np.random.uniform(w_low, w_high)
    H = np.random.uniform(h_low, h_high)

    return W,H

def random_scalar(x):
    low, high = _make_low_high(x)
    return np.random.uniform(low, high)

class BoxPackingHook(BulletHook):

    default_make_packed_cuboids_scene_kwargs = {
        'box_center': 0,
        'inplane_rot_angles': [0],
        'slack': 0.,
        'block_filter': None,
    }

    default_add_obj_kwargs = {
        'fix_base': True,
        'scale': 1.,
        'base_mass': .05,
        'base_inertialframe_position': np.zeros(3),
        'texture_file': None,
        'rgb': None,
    }

    def __init__(self, 
        path_to_obj, 
        category_name,
        box_dim, 
        box_center,
        mesh_scale = 1.0,
        inplane_rot_angles = [0], 
        slack = 0., 
        block_filter = None
        ):

        self.kwargs_setters = {
            'path_to_obj': partial(_identity, x=path_to_obj),
            'mesh_scale': partial(_identity, mesh_scale),
            # 'mesh_scale': partial(random_scalar, x=mesh_scale),
            'box_dim': partial(random_rectangle, width=box_dim[0], height=box_dim[1]),
            'box_center': partial(random_rectangle, width=box_center[0], height=box_center[1]),
            'inplane_rot_angles': partial(_identity, x=inplane_rot_angles),
            'slack': partial(random_scalar, x=slack),
            'block_filter': partial(_identity, x=block_filter),
        }
        self.path_to_obj = path_to_obj
        self.mesh_scale = mesh_scale
        self.category_name = category_name
        self.elapsed1 = 0.
        self.elapsed2 = 0.

    def after_reset(self, sim):
        # reset kwargs of make_packed_cuboids_scene()
        make_packed_cuboids_scene_kwargs = self.default_make_packed_cuboids_scene_kwargs.copy()
        already_set_kws = []
        for kw, fn in self.kwargs_setters.items():
            assert kw not in already_set_kws
            arg = fn()
            make_packed_cuboids_scene_kwargs[kw] = arg
            already_set_kws.append(kw)

        # self.make_packed_cuboids_kwargs = make_packed_cuboids_kwargs

        start = time.time()
        poses = make_packed_cuboids_scene(**make_packed_cuboids_scene_kwargs)
        self.elapsed1 += time.time() - start
        start = time.time()
        for pose in poses:
            add_obj_kwargs = self.default_add_obj_kwargs.copy()
            add_obj_kwargs.update({
                'path_to_obj': self.path_to_obj,
                'scale': self.mesh_scale,
                'position': pose[1],
                'orientation': pose[0],
                })
            body_id = add_obj(**add_obj_kwargs)
            obj_info = ObjectInfo( self.path_to_obj, self.mesh_scale, body_id, self.category_name )
            sim.objects.append(obj_info)

        self.elapsed2 += time.time() - start

    def after_step(self, sim, hooks_output):
        pass   

    def before_end(self, sim, hooks_output):
        pass

    def close(self):
        pass
