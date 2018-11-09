import os
import numpy as np
import random
import glob
from functools import partial
from bulletwrapper.hooks import OBJCreatorHook
import pybullet_data
from bulletwrapper.hooks.core import random_size_to_scale, random_texture_file, random_color

# PATH_TO_TRAY_OBJ = os.path.join( pybullet_data.getDataPath(), 'tray/tray_textured.obj' )
PATH_TO_TRAY_OBJ = os.path.join( pybullet_data.getDataPath(), 'tray/tray_textured4.obj' )
TRAY_SIZE = 0.82 # size of bottom side

class TrayHook(OBJCreatorHook):

    def __init__(self, size=1.):

        kwargs_setters = {
            'path_to_obj': lambda: PATH_TO_TRAY_OBJ,
            'scale': partial(random_size_to_scale, size=size, model_size=TRAY_SIZE),
            'fix_base': lambda: True,
            }

        self._time_to_create = 0.
        self.kwargs_setters = kwargs_setters
        self.category_name = 'tray'

class RandomTexturedTrayHook(OBJCreatorHook):

    def __init__(self, texture_image_dir, size=1.):

        kwargs_setters = {
            'path_to_obj': lambda: PATH_TO_TRAY_OBJ,
            'scale': partial(random_size_to_scale, size=size, model_size=TRAY_SIZE),
            'fix_base': lambda: True,
            'texture_file': partial(random_texture_file, texture_image_dir=texture_image_dir),
            'rgb': random_color,
            }

        self._time_to_create = 0.
        self.kwargs_setters = kwargs_setters
        self.category_name = 'tray'

