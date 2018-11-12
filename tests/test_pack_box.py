import os
import numpy as np
import pybullet as pb
import yaml
import tempfile

VIS = False
# VIS = True
if VIS:
    import matplotlib.pyplot as plt

# CONNECT_MODE = pb.DIRECT
CONNECT_MODE = pb.GUI

TEST_DATA_DIR = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'test_data')
CUBOID_OBJ_PATH = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'test_data', 'cuboid.obj')

def block_filter(block, R, zoffset):
    return not np.allclose( np.abs(R[-1]), np.array([0,0,1]) )

def test_pack_box():

    ground_plane = RandomTexturedGroundPlaneHook(
        size = 1.,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/carpet'),
        )

    cuboid_in_box = BoxPackingHook(
        path_to_obj = CUBOID_OBJ_PATH,
        box_dim = [ (1.7, 2.3), (2.7, 3.3) ],
        box_center = [(-0.2, 0.2), (-0.1, 0.1)],
        inplane_rot_angles = [-6, -4, -2, 0, 2, 4, 6],
        slack = (0, 0.03),
        block_filter = block_filter,
        )

    cache_dir = tempfile.mkdtemp()
    
    dataset_writer_1 = DatasetWriter(
        path_to_dataset = cache_dir,
        base_format = 'C1_%04d.png',
        path_to_yml = os.path.join(cache_dir, 'anno_c1.yml')
        )
