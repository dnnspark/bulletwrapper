import numpy as np
import os
import pybullet as pb
import pybullet_data
from bulletwrapper import BulletSimulator, StopSimulation
from bulletwrapper.hooks import GroundPlaneHook, OBJCreatorHook
from bulletwrapper.hooks.ogl_cameras import StaticOGLCameraHook

# from imageio import imwrite

CONNECT_MODE = pb.DIRECT
# CONNECT_MODE = pb.GUI

DUCK_OBJ_PATH = os.path.join(pybullet_data.getDataPath(), 'duck.obj')

def maybe_collect_images(step_out):
    images = []
    for key in step_out.output.keys():
        if 'static_ogl_camera' in key:
            I = step_out.output[key]
            images.append(I)
    return images

def test_static_opengl_cameras():
    '''
    Identical to test_obj_adder.test_obj_adder_multiple_in_sequence(), but
    add a single-shot camera.
    '''

    # copy-and-paste
    ground_plane = GroundPlaneHook()

    height = 3.

    duck_1 = OBJCreatorHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 0.,
        )

    duck_2 = OBJCreatorHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 1.,
        )

    duck_3 = OBJCreatorHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 2.,
        )

    camera_1 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([10., 10., 7.]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = 0.,
        # interval = .5,
        interval = .1,
        )

    camera_2 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([-10., -10., 6.]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = 0.,
        # interval = .5,
        interval = .1,
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=5.,
        hooks=[
            ground_plane,
            duck_1,
            duck_2,
            duck_3,
            camera_1,
            camera_2,
            ],
        )

    num_images = 0

    step_out = sim.reset()
    images = maybe_collect_images(step_out)
    for I in images:
        assert I.shape == (400,400,4)
    num_images += len(images)
    while True:
        try:
            step_out = sim.step()
            images = maybe_collect_images(step_out)
            for I in images:
                assert I.shape == (400,400,4)
            num_images += len(images)

        except StopSimulation:
            break;
    assert num_images == 100
