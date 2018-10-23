from .core import BulletSimulator, BulletHook, StopSimulation, ObjectInfo

import os
root = os.path.dirname(os.path.realpath(__file__))
data_root = os.path.join(root, 'data')
