import os
import numpy as np
import pybullet as pb
import yaml
import tempfile
import time
from bulletwrapper import BulletSimulator
from bulletwrapper.hooks import RandomTexturedGroundPlaneHook
from bulletwrapper.hooks.ogl_cameras import  RandomStaticCameraHook
from bulletwrapper.hooks.box_packing import BoxPackingHook
from bulletwrapper.dataset import DatasetWriter

from imageio import imread
from bulletwrapper.util.project_util import project_mesh
from bulletwrapper.util.transform_util import chain_Rts as chain
from bulletwrapper.util.transformations import quaternion_matrix as quat2R
from bulletwrapper.util import object_loader

VIS = False
# VIS = True
if VIS:
    import matplotlib.pyplot as plt

CONNECT_MODE = pb.DIRECT
# CONNECT_MODE = pb.GUI

TEST_DATA_DIR = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'test_data')
CUBOID_OBJ_PATH = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'test_data', 'cuboid.obj')

# def block_filter(block, R, zoffset):
def block_filter(block):
    return not np.allclose( np.abs(block.R[-1]), np.array([0,0,1]) )


def test_pack_box():

    ground_plane = RandomTexturedGroundPlaneHook(
        size = 30.,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/carpet'),
        )

    # mesh_scale = 1.0
    mesh_scale = .7

    box_in_box = BoxPackingHook(
        path_to_obj = CUBOID_OBJ_PATH,
        category_name = 'boxy_obj',
        mesh_scale = mesh_scale,
        box_dim = [ (2.5, 2.8), (3.5, 3.8) ],
        # box_center = [(-0.3, 0.3), (-0.2, 0.2)],
        box_center = [(-1.3, -0.7), (-0.2, 0.2)],
        inplane_rot_angles = [-6, -4, -2, 0, 2, 4, 6],
        slack = (0, 0.1),
        block_filter = block_filter,
        )

    cache_dir = tempfile.mkdtemp()
    print('\ncache_dir: %s' % cache_dir)
    
    dataset_writer = DatasetWriter(
        path_to_dataset = cache_dir,
        base_format = 'C1_%04d.png',
        yml_filename = 'anno_c1.yml',
        )

    camera = RandomStaticCameraHook(

        K = [616.262451172, 0.0, 330.200531006, 0.0, 616.415588379, 235.531219482, 0.0, 0.0, 1.0],
        img_shape = (480, 640),
        position = [(-6., 6.), (-6., 6.), 8.],
        lookat = np.zeros(3),
        up = 'up',
        light_src = [(-5.0, 5.0), (-5.0, 5.0), (5.0, 8.0)],
        start = 0.,
        dataset_writer = dataset_writer,
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=0.00001,
        # max_time=1000.0,
        hooks=[
            ground_plane,
            box_in_box,
            camera,
            ]
        )

    start_time = time.time()
    for _ in range(4):
        out = sim.reset()
        while sim.running:
            out = sim.step()
        # assert len(sim.objects) == N + 2
    elapsed = time.time() - start_time
    sim.close()

    with open( os.path.join(cache_dir, 'anno_c1.yml') ) as f:
        anno = yaml.load(f)

    for example in anno:
        rgb = imread( os.path.join(cache_dir, example['path_to_rgb']) )
        depth = imread( os.path.join(cache_dir, example['path_to_depth']) )
        labels = imread( os.path.join( cache_dir, example['path_to_label'] ) )

        K = np.reshape(example['intrinsics'], (3,3))
        img_shape = (480,640)

        world_to_cam = example['world_to_cam']
        world_to_cam = (quat2R(world_to_cam['R'])[:3,:3],  np.array(world_to_cam['t']) )

        projected_foreground = np.zeros(img_shape)
        for obj in example['objects']:
            if obj['category_name'] in ['plane', 'tray']:
                continue

            mesh_scale = obj['mesh_scale']

            mesh = object_loader.OBJFile( os.path.join(cache_dir, obj['path_to_obj']), None)
            mesh.vertices = [ [mesh_scale * x for x in v] for v in mesh.vertices ]
            object_to_world = obj['object_to_world']
            object_to_world = (quat2R(object_to_world['R'])[:3,:3],  np.array(object_to_world['t']) )
            object_to_cam = chain(object_to_world, world_to_cam)
            R,t = object_to_cam
            mask = project_mesh(mesh, R, t, K, img_shape)
            projected_foreground[mask>0] = 1.

        foreground_from_labels = np.float32(labels > 0)

        inter = np.float32( np.logical_and(projected_foreground>0, foreground_from_labels>0) )
        union = np.float32( np.logical_or(projected_foreground>0, foreground_from_labels>0) )

        overlap = inter.sum() / union.sum()

        assert overlap > .9
