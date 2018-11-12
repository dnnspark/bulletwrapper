import os
import numpy as np
import pybullet as pb
import yaml
import tempfile
from bulletwrapper import BulletSimulator
from bulletwrapper.hooks import RandomTexturedGroundPlaneHook, RandomFreeFallObject
from bulletwrapper.hooks.trays import RandomTexturedTrayHook
from bulletwrapper.hooks.ogl_cameras import StaticOGLCameraHook
from bulletwrapper.dataset import DatasetWriter
import pybullet_data
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

DUCK_OBJ_PATH = os.path.join(pybullet_data.getDataPath(), 'duck.obj')
TEST_DATA_DIR = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'test_data')

def test_dataset_two_cams():

    ground_plane = RandomTexturedGroundPlaneHook(
        size = 1.2,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/carpet'),
        )

    tray = RandomTexturedTrayHook(
        size=0.6,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/wood'),
        )

    duck_1 = RandomFreeFallObject(
        category_name = 'rubber_duck',
        path_to_obj = DUCK_OBJ_PATH,
        x = (-.05, .05),
        y = (-.05, .05),
        height = .5,
        scale = 0.1,
        time_to_create = .3
        )

    duck_2 = RandomFreeFallObject(
        category_name = 'rubber_duck',
        path_to_obj = DUCK_OBJ_PATH,
        x = (-.05, .05),
        y = (-.05, .05),
        height = .5,
        scale = 0.1,
        time_to_create = 1.3
        )

    duck_3 = RandomFreeFallObject(
        category_name = 'rubber_duck',
        path_to_obj = DUCK_OBJ_PATH,
        x = (-.05, .05),
        y = (-.05, .05),
        height = .5,
        scale = 0.1,
        time_to_create = 2.3
        )

    cache_dir = tempfile.mkdtemp()
    print('\ncache_dir: %s' % cache_dir)

    dataset_writer_1 = DatasetWriter(
        path_to_dataset = cache_dir,
        base_format = 'C1_%04d.png',
        yml_filename = 'anno_c1.yml',
        )

    dataset_writer_2 = DatasetWriter(
        path_to_dataset = cache_dir,
        base_format = 'C2_%04d.png',
        yml_filename = 'anno_c2.yml',
        )

    camera_1 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([.8, .8, 1.3]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        # start = 0.,
        # interval = .25,
        dataset_writer = dataset_writer_1,
        )

    camera_2 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([-.7, -.7, 1.2]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        # start = 0.,
        # interval = .25,
        dataset_writer = dataset_writer_2,
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=5.,
        hooks=[
            ground_plane,
            tray,
            duck_1,
            duck_2,
            duck_3,
            camera_1,
            camera_2,
            ],
        )

    for _ in range(3):

        out = sim.reset()
        while sim.running:
            out = sim.step()
        assert len(sim.objects) == 3 + 1 + 1
    sim.close()


    with open( os.path.join(cache_dir, 'anno_c1.yml') ) as f:
        anno_c1 = yaml.load(f)
    with open( os.path.join(cache_dir, 'anno_c2.yml') ) as f:
        anno_c2 = yaml.load(f)


    for anno in [anno_c1, anno_c2]:

        for example in anno:
            rgb = imread( os.path.join(cache_dir, example['path_to_rgb']) )
            depth = imread( os.path.join(cache_dir, example['path_to_depth']) )
            labels = imread( os.path.join( cache_dir, example['path_to_label'] ) )

            K = np.reshape(example['intrinsics'], (3,3))
            img_shape = (400,400)

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

            foreground_from_labels = np.float32(labels > 1)

            inter = np.float32( np.logical_and(projected_foreground>0, foreground_from_labels>0) )
            union = np.float32( np.logical_or(projected_foreground>0, foreground_from_labels>0) )

            overlap = inter.sum() / union.sum()

            assert overlap > .9
