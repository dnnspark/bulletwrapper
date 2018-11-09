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

    dataset_writer_1 = DatasetWriter(
        path_to_dataset = cache_dir,
        base_format = 'C1_%04d.png',
        path_to_yml = os.path.join(cache_dir, 'anno_c1.yml')
        )

    dataset_writer_2 = DatasetWriter(
        path_to_dataset = cache_dir,
        base_format = 'C2_%04d.png',
        path_to_yml = os.path.join(cache_dir, 'anno_c2.yml')
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

    for ann1, ann2 in zip(anno_c1, anno_c2):
        rgb1 = imread( os.path.join(cache_dir, ann1['path_to_rgb']) )
        rgb2 = imread( os.path.join(cache_dir, ann2['path_to_rgb']) )
        assert rgb1.shape == (400,400,4)
        assert rgb2.shape == (400,400,4)

        depth1 = imread( os.path.join(cache_dir, ann1['path_to_depth']) )
        depth2 = imread( os.path.join(cache_dir, ann2['path_to_depth']) )
        assert depth1.shape == (400,400)
        assert depth2.shape == (400,400)

        label1 = imread( os.path.join(cache_dir, ann1['path_to_label']) )
        label2 = imread( os.path.join(cache_dir, ann2['path_to_label']) )
        assert label1.shape == (400,400)
        assert label2.shape == (400,400)

        if VIS:
            plt.figure(1), plt.clf(), plt.imshow(np.concatenate([rgb1, rgb2], axis=1))
            plt.figure(2), plt.clf(), plt.imshow(np.concatenate([depth1, depth2], axis=1))
            plt.figure(3), plt.clf(), plt.imshow(np.concatenate([label1, label2], axis=1))
            plt.show(block=False)
            plt.pause(.5)

    print('\ncache_dir: %s' % cache_dir)
    assert set( np.unique(label1) ) == set([0,1,2,3,4])
    assert set( np.unique(label2) ) == set([0,1,2,3,4])
