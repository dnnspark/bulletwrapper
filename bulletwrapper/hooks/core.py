import os
import numpy as np
import pybullet as pb
import pybullet_data
import glob
import random
from functools import partial
from bulletwrapper import BulletHook, ObjectInfo
from bulletwrapper.util.bullet_util import add_obj
from bulletwrapper import data_root
from bulletwrapper.util.transformations import random_quaternion

class GroundPlaneHook(BulletHook):

    def after_reset(self, sim):
        path_to_plane = os.path.join( pybullet_data.getDataPath(), 'plane.urdf' )
        plane_id = pb.loadURDF( path_to_plane )

        return ObjectInfo(path_to_plane, 1., plane_id, 'plane')

def _make_str_list(x):
    '''
    x: str or [str]
    '''

    if isinstance(x, str):
        return [x]
    else:
        assert all([isinstance(k,str) for k in x])
        return x

def _make_low_high(x):
    '''
    x: int or (int, int)
    '''
    try:
        low, high = x
    except TypeError:
        low = high = x
    return low, high


def random_obj_file(obj_files):

    obj_files = _make_str_list(obj_files)
    obj_file = random.choice(obj_files)
    return obj_file

def random_position(x, y, z):

    x_min, x_max = _make_low_high(x)
    y_min, y_max = _make_low_high(y)
    z_min, z_max = _make_low_high(z)

    xx = np.random.uniform(x_min, x_max)
    yy = np.random.uniform(y_min, y_max)
    zz = np.random.uniform(z_min, z_max)

    return xx, yy, zz

def random_size_to_scale(size, model_size):

    low, high = _make_low_high(size)

    _size = np.random.uniform(low, high)
    scale = _size / model_size

    return scale

def random_texture_file(texture_image_dir):
    texture_files = glob.glob(os.path.join(texture_image_dir, '*'))
    assert len(texture_files) > 0
    texture_file = random.choice(texture_files)

    return texture_file

def random_color():
    return np.random.uniform(size=3)



class OBJCreatorHook(BulletHook):
    '''
    Create a custom pybullet object at a given time and position.
    NOTE: pybullet only supports .obj file for custom object model. (See pb.createCollisionShape().)
    '''
    
    default_add_obj_kwargs = {
        'position': np.zeros(3),
        'orientation': np.eye(3),
        'fix_base': False,
        'scale': 1.,
        'base_mass': .01,
        'base_inertialframe_position': np.zeros(3),
        'texture_file': None,
        'rgb': None,
    }

    def __init__(self, time_to_create, kwargs_setters, category_name):
        self._time_to_create = time_to_create
        self.kwargs_setters = kwargs_setters
        self.category_name = category_name

    def create(self):
        body_id = add_obj(**self.add_obj_kwargs)
        self.created = True

        return body_id

    def set_time_to_create(self):

        try:
            low, high = self._time_to_create
        except TypeError:
            low = high = self._time_to_create
        self.time_to_create = np.random.uniform(low, high)

    def after_reset(self, sim):

        # reset kwargs of add_obj()
        add_obj_kwargs = self.default_add_obj_kwargs.copy()
        already_set_kws = []
        for kw, fn in self.kwargs_setters.items():
            assert kw not in already_set_kws
            arg = fn()
            add_obj_kwargs[kw] = arg
            already_set_kws.append(kw)
        self.add_obj_kwargs = add_obj_kwargs

        self.path_to_obj = add_obj_kwargs['path_to_obj']
        self.mesh_scale = add_obj_kwargs['scale']
        self.set_time_to_create()

        self.created = False
        if sim.sim_time >= self.time_to_create:
            body_id = self.create()
            obj_info = ObjectInfo( self.path_to_obj, self.mesh_scale, body_id, self.category_name )
            sim.objects.append(obj_info)
            return obj_info

    def after_step(self, sim, hooks_output):
        if not self.created and sim.sim_time >= self.time_to_create:
            body_id = self.create()
            obj_info = ObjectInfo( self.path_to_obj, self.mesh_scale, body_id, self.category_name )
            sim.objects.append(obj_info)
            return obj_info

def _identity(x):
    return x

class BasicOBJHook(OBJCreatorHook):

    def __init__(self, category_name, time_to_create=0., **kwargs):

        self._time_to_create = time_to_create

        kwargs_setters = {kw: partial(_identity, x=arg) for kw, arg in kwargs.items()}
        self.kwargs_setters = kwargs_setters
        self.category_name = category_name

class TexturedGroundPlaneHook(OBJCreatorHook):
    '''
    Create a textured plane on start of simulation.

    texture_file: str
        path to texture image
    size: float or (float, float)
        size of plane in meter.
    '''

    def __init__(self, texture_file, size=1.):

        kwargs_setters = {
            'path_to_obj': lambda: os.path.join(data_root, 'models/unit_plane.obj'),
            'scale': partial(random_size_to_scale, size=size, model_size=1.),
            'fix_base': lambda: True,
            'texture_file': lambda: texture_file,
            }

        self._time_to_create = 0.
        self.kwargs_setters = kwargs_setters
        self.category_name = 'plane'

class RandomTexturedGroundPlaneHook(OBJCreatorHook):
    '''
    Create a plane with random texture on start of simulation.

    texture_image_dir: str
        path to folder containing texture images.
    '''

    def __init__(self, texture_image_dir, size=1.):

        kwargs_setters = {
            'path_to_obj': lambda: os.path.join(data_root, 'models/unit_plane.obj'),
            'scale': partial(random_size_to_scale, size=size, model_size=1.),
            'fix_base': lambda: True,
            'texture_file': partial(random_texture_file, texture_image_dir=texture_image_dir),
            'rgb': random_color,
            }

        self._time_to_create = 0.
        self.kwargs_setters = kwargs_setters
        self.category_name = 'plane'


class RandomFreeFallObject(OBJCreatorHook):
    '''
    Release object at a 3D position on start of simulation.
        - random orientation
        - fixed scale
    '''

    def __init__(self, category_name, path_to_obj, time_to_create=0., x=0., y=0., height=1., scale=1.):

        kwargs_setters = {
            'path_to_obj': partial(random_obj_file, obj_files=path_to_obj),
            'position': partial(random_position, x=x, y=y, z=height),
            'orientation': random_quaternion,
            'scale': lambda: scale,
        }

        self._time_to_create = time_to_create
        self.kwargs_setters = kwargs_setters
        self.category_name = category_name

