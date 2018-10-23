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
from bulletwrapper.util.transformations import random_quaternion
import pybullet_data
from imageio import imread

VIS = False
# VIS = True
if VIS:
    import matplotlib.pyplot as plt

CONNECT_MODE = pb.DIRECT
# CONNECT_MODE = pb.GUI

TEST_DATA_DIR = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'test_data')
TLESS_OBJ_PATH = os.path.join(TEST_DATA_DIR, 'tless_reconst_model_samples', 'obj_%02d.obj')

def test_tray_tless():

    ground_plane = RandomTexturedGroundPlaneHook(
        size = .3,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/carpet'),
        )

    tray = RandomTexturedTrayHook(
        size=0.15,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/wood'),
        )

    N = 7

    tless_objects = []
    for t in range(N):
        time_to_create = .4 * t

        obj_id = np.random.choice([1,10])

        tless_obj = RandomFreeFallObject(
            path_to_obj = TLESS_OBJ_PATH % obj_id,
            x = (-.05, .05),
            y = (-.05, .05),
            height = .2,
            time_to_create = time_to_create,
            scale = .001,  # tless obj file use millimeter unit. E.g. obj_01 is about 6cm.
            )

        tless_objects.append(tless_obj)


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
        position = np.array([.3, .3, .5]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        dataset_writer = dataset_writer_1,
        )

    camera_2 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([-.4, -.4, .6]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        dataset_writer = dataset_writer_2,
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=3.2,
        hooks=[
            ground_plane,
            tray,
            camera_1,
            camera_2,
            ] + tless_objects,
        )

    for _ in range(2):
        out = sim.reset()
        while sim.running:
            out = sim.step()
        assert len(sim.objects) == N + 2
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
    print('\ncache dir: %s' % cache_dir)
