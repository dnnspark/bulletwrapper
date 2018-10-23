import os
import numpy as np
import pybullet as pb
from bulletwrapper import BulletSimulator
from bulletwrapper.hooks import RandomTexturedGroundPlaneHook, RandomFreeFallObject
from bulletwrapper.hooks.trays import RandomTexturedTrayHook
import pybullet_data

CONNECT_MODE = pb.DIRECT
# CONNECT_MODE = pb.GUI

DUCK_OBJ_PATH = os.path.join(pybullet_data.getDataPath(), 'duck.obj')
TEST_DATA_DIR = os.path.join( os.path.dirname( os.path.dirname(os.path.realpath(__file__)) ), 'test_data')

def test_plane():

    ground_plane = RandomTexturedGroundPlaneHook(
        size = 1.,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/carpet'),
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=3.,
        hooks=[
            ground_plane,
            ],
        )

    for _ in range(5):
        sim.reset()
        while sim.running:
            sim.step() 
        assert len(sim.objects) == 1
    sim.close()

def test_duck_on_plane():

    ground_plane = RandomTexturedGroundPlaneHook(
        size = 1.,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/wood'),
        )

    duck = RandomFreeFallObject(
        path_to_obj = DUCK_OBJ_PATH,
        x = (-.1, .1),
        y = (-.1, .1),
        height = 1.,
        scale = 0.1,
        )


    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=3.,
        hooks=[
            ground_plane,
            duck,
            ],
        )

    for _ in range(3):
        sim.reset()
        while sim.running:
            sim.step() 
        assert len(sim.objects) == 2
    sim.close()


def test_ducks_on_tray_on_plane():

    ground_plane = RandomTexturedGroundPlaneHook(
        size = 1.,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/carpet'),
        )

    tray = RandomTexturedTrayHook(
        size=0.3,
        texture_image_dir=os.path.join(TEST_DATA_DIR, 'textures/wood'),
        )

    duck_1 = RandomFreeFallObject(
        path_to_obj = DUCK_OBJ_PATH,
        x = (-.1, .1),
        y = (-.1, .1),
        height = 1.,
        scale = 0.1,
        time_to_create = 0.
        )

    duck_2 = RandomFreeFallObject(
        path_to_obj = DUCK_OBJ_PATH,
        x = (-.1, .1),
        y = (-.1, .1),
        height = 1.,
        scale = 0.1,
        time_to_create = .5
        )

    duck_3 = RandomFreeFallObject(
        path_to_obj = DUCK_OBJ_PATH,
        x = (-.1, .1),
        y = (-.1, .1),
        height = 1.,
        scale = 0.1,
        time_to_create = 1.
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=3.,
        hooks=[
            ground_plane,
            tray,
            duck_1,
            duck_2,
            duck_3,
            ],
        )

    for _ in range(3):

        sim.reset()
        while sim.running:
            sim.step() 
        assert len(sim.objects) == 5
    sim.close()
