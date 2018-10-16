import os
import numpy as np
from bulletwrapper.hooks import OBJCreatorHook
import pybullet_data

PATH_TO_TRAY_OBJ = os.path.join( pybullet_data.getDataPath(), 'tray/tray_textured.obj' )
TRAY_DIMENSION = 1.155

class TrayOBJHook(OBJCreatorHook):

    default_add_obj_kwargs = OBJCreatorHook.default_add_obj_kwargs

    def __init__(self, tray_size=1.155):

        scale = tray_size / TRAY_DIMENSION

        kwargs = dict(
            path_to_obj = PATH_TO_TRAY_OBJ,
            position = np.zeros(3),
            time_to_create = 0.,
            fix_base = True,
            scale = scale,
        )

        super().__init__(**kwargs)




