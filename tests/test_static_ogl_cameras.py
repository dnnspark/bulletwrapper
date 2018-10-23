import numpy as np
import os
import pybullet as pb
import pybullet_data
from bulletwrapper import BulletSimulator
from bulletwrapper.hooks import GroundPlaneHook, BasicOBJHook
from bulletwrapper.hooks.ogl_cameras import StaticOGLCameraHook

CONNECT_MODE = pb.DIRECT
# CONNECT_MODE = pb.GUI

DUCK_OBJ_PATH = os.path.join(pybullet_data.getDataPath(), 'duck.obj')

def maybe_collect_images(step_out):
    images = []
    for key in step_out.output.keys():
        if 'static_ogl_camera' in key:
            I, D, L = step_out.output[key]
            images.append(I)
    return images


def test_static_opengl_cameras():
    '''
    Based on test_obj_adder.test_obj_adder_multiple_in_sequence().

    Take the first picture upon reset (start=0.), 
    and then take pictures every 0.1 seconds until time is up.
    '''

    # copy-and-paste
    ground_plane = GroundPlaneHook()

    height = 3.

    duck_1 = BasicOBJHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 0.,
        )

    duck_2 = BasicOBJHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 1.,
        )

    duck_3 = BasicOBJHook(
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



    for _ in range(2):
        out = sim.reset()
        num_images = 0

        images = maybe_collect_images(out)
        for I in images:
            assert I.shape == (400,400,4)
        num_images += len(images)

        while sim.running:
            out = sim.step()

            images = maybe_collect_images(out)
            for I in images:
                assert I.shape == (400,400,4)
            num_images += len(images)

        assert num_images == 100

    sim.close()


def test_static_opengl_cameras_one_final_shot():
    '''
    Identical to above, but take only one pictures before simulation terminates (start=np.inf).
    '''

    ground_plane = GroundPlaneHook()

    height = 3.

    duck_1 = BasicOBJHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 0.,
        )

    duck_2 = BasicOBJHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 1.,
        )

    duck_3 = BasicOBJHook(
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
        start = np.inf,
        )

    camera_2 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([-10., -10., 6.]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = np.inf,
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



    for _ in range(2):
        out = sim.reset()
        num_images = 0

        images = maybe_collect_images(out)
        assert len(images) == 0

        while sim.running:
            out = sim.step()

            images = maybe_collect_images(out)
            if len(images) > 0:
                assert len(images) == 2
                assert sim.terminated
                for I in images:
                    assert I.shape == (400,400,4)

    sim.close()


def test_static_opengl_cameras_one_shot_at_2s():
    '''
    Identical to above, but take only one pictures at t=2s
    '''

    ground_plane = GroundPlaneHook()

    height = 3.

    duck_1 = BasicOBJHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 0.,
        )

    duck_2 = BasicOBJHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,height]),
        time_to_create = 1.,
        )

    duck_3 = BasicOBJHook(
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
        start = 2.,
        )

    camera_2 = StaticOGLCameraHook(
        K = [1075.65091572, 0.0, 210.06888344, 0.0, 1073.90347929, 174.72159802, 0.0, 0.0, 1.0],
        img_shape = (400,400),
        position = np.array([-10., -10., 6.]),
        lookat = np.array([0., 0., 0.]),
        up = 'up',
        start = 2.,
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


    for _ in range(2):

        out = sim.reset()

        images = maybe_collect_images(out)
        assert len(images) == 0

        num_images = 0

        while sim.running:
            out = sim.step()

            images = maybe_collect_images(out)
            num_images += len(images)
            if len(images) > 0:
                assert len(images) == 2
                assert sim.sim_time - 2. < 0.003
                for I in images:
                    assert I.shape == (400,400,4)

        assert num_images == 2

    sim.close()

