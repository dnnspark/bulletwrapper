import os
import numpy as np
import pybullet as pb
import yaml
import tempfile
from bulletwrapper import BulletSimulator
from bulletwrapper.hooks import GroundPlaneHook, OBJCreatorHook
from bulletwrapper.hooks.trays import TrayOBJHook
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

DUCK_OBJ_PATH = os.path.join(pybullet_data.getDataPath(), 'duck.obj')

def test_tray_ducks():

    ground_plane = GroundPlaneHook()
    tray = TrayOBJHook(tray_size=0.15)

    N = 10

    ducks = []
    for t in range(N):
        x = np.random.uniform(low=-.05, high=.05)
        y = np.random.uniform(low=-.05, high=.05)
        z = .4
        time_to_create = .4 * t

        duck = OBJCreatorHook(
            path_to_obj = DUCK_OBJ_PATH,
            position = np.array([x, y, z]),
            rotation = random_quaternion(),
            time_to_create = time_to_create,
            scale = .07, 
            )

        ducks.append(duck)


    # data_root = 'datasets/test_dataset/'
    data_root = tempfile.mkdtemp()

    dataset_writer_1 = DatasetWriter(
        path_to_dataset = data_root,
        base_format = 'C1_%04d.png',
        path_to_yml = os.path.join(data_root, 'anno_c1.yml')
        )

    dataset_writer_2 = DatasetWriter(
        path_to_dataset = data_root,
        base_format = 'C2_%04d.png',
        path_to_yml = os.path.join(data_root, 'anno_c2.yml')
        )


    camera_1 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([.3, .3, 1.2]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        dataset_writer = dataset_writer_1,
        )

    camera_2 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([-.4, -.4, 1.2]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        dataset_writer = dataset_writer_2,
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=6.,
        hooks=[
            ground_plane,
            tray,
            camera_1,
            camera_2,
            ] + ducks,
        )

    out = sim.reset()
    while sim.running:
        out = sim.step()
    assert len(sim.objects) == N + 1

    with open( os.path.join(data_root, 'anno_c1.yml') ) as f:
        anno_c1 = yaml.load(f)
    with open( os.path.join(data_root, 'anno_c2.yml') ) as f:
        anno_c2 = yaml.load(f)

    for ann1, ann2 in zip(anno_c1, anno_c2):
        rgb1 = imread( os.path.join(data_root, ann1['path_to_rgb']) )
        rgb2 = imread( os.path.join(data_root, ann2['path_to_rgb']) )
        assert rgb1.shape == (400,400,4)
        assert rgb2.shape == (400,400,4)

        depth1 = imread( os.path.join(data_root, ann1['path_to_depth']) )
        depth2 = imread( os.path.join(data_root, ann2['path_to_depth']) )
        assert depth1.shape == (400,400)
        assert depth2.shape == (400,400)

        label1 = imread( os.path.join(data_root, ann1['path_to_label']) )
        label2 = imread( os.path.join(data_root, ann2['path_to_label']) )
        assert label1.shape == (400,400)
        assert label2.shape == (400,400)

        if VIS:
            plt.figure(1), plt.clf(), plt.imshow(np.concatenate([rgb1, rgb2], axis=1))
            plt.figure(2), plt.clf(), plt.imshow(np.concatenate([depth1, depth2], axis=1))
            plt.figure(3), plt.clf(), plt.imshow(np.concatenate([label1, label2], axis=1))
            plt.show(block=False)
            plt.pause(.5)

DATA_DIR = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'data')
TLESS_OBJ_PATH = os.path.join(DATA_DIR, 'tless_reconst_model_samples', 'obj_%02d.obj')

def test_tray_tless():

    ground_plane = GroundPlaneHook()
    tray = TrayOBJHook(tray_size=0.12)

    N = 10

    tless_objects = []
    for t in range(N):
        x = np.random.uniform(low=-.05, high=.05)
        y = np.random.uniform(low=-.05, high=.05)
        z = .4
        time_to_create = .4 * t

        obj_id = np.random.choice([1,3])

        tless_obj = OBJCreatorHook(
            path_to_obj = TLESS_OBJ_PATH % obj_id,
            position = np.array([x, y, z]),
            rotation = random_quaternion(),
            time_to_create = time_to_create,
            scale = .001,  # tless obj file use millimeter unit. E.g. obj_01 is about 6cm.
            )

        tless_objects.append(tless_obj)


    # data_root = 'datasets/test_dataset/'
    data_root = tempfile.mkdtemp()

    dataset_writer_1 = DatasetWriter(
        path_to_dataset = data_root,
        base_format = 'C1_%04d.png',
        path_to_yml = os.path.join(data_root, 'anno_c1.yml')
        )

    dataset_writer_2 = DatasetWriter(
        path_to_dataset = data_root,
        base_format = 'C2_%04d.png',
        path_to_yml = os.path.join(data_root, 'anno_c2.yml')
        )


    camera_1 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([.25, .25, .8]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        dataset_writer = dataset_writer_1,
        )

    camera_2 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([-.3, -.3, .7]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
        dataset_writer = dataset_writer_2,
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=6.,
        hooks=[
            ground_plane,
            tray,
            camera_1,
            camera_2,
            ] + tless_objects,
        )

    out = sim.reset()
    while sim.running:
        out = sim.step()
    assert len(sim.objects) == N + 1

    with open( os.path.join(data_root, 'anno_c1.yml') ) as f:
        anno_c1 = yaml.load(f)
    with open( os.path.join(data_root, 'anno_c2.yml') ) as f:
        anno_c2 = yaml.load(f)

    for ann1, ann2 in zip(anno_c1, anno_c2):
        rgb1 = imread( os.path.join(data_root, ann1['path_to_rgb']) )
        rgb2 = imread( os.path.join(data_root, ann2['path_to_rgb']) )
        assert rgb1.shape == (400,400,4)
        assert rgb2.shape == (400,400,4)

        depth1 = imread( os.path.join(data_root, ann1['path_to_depth']) )
        depth2 = imread( os.path.join(data_root, ann2['path_to_depth']) )
        assert depth1.shape == (400,400)
        assert depth2.shape == (400,400)

        label1 = imread( os.path.join(data_root, ann1['path_to_label']) )
        label2 = imread( os.path.join(data_root, ann2['path_to_label']) )
        assert label1.shape == (400,400)
        assert label2.shape == (400,400)

        if VIS:
            plt.figure(1), plt.clf(), plt.imshow(np.concatenate([rgb1, rgb2], axis=1))
            plt.figure(2), plt.clf(), plt.imshow(np.concatenate([depth1, depth2], axis=1))
            plt.figure(3), plt.clf(), plt.imshow(np.concatenate([label1, label2], axis=1))
            plt.show(block=False)
            plt.pause(.5)
