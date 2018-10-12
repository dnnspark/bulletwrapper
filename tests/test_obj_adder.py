import os
import numpy as np
import pybullet as pb
from bulletwrapper import BulletSimulator, StopSimulation
from bulletwrapper.hooks import GroundPlaneHook, OBJCreatorHook
import pybullet_data

CONNECT_MODE = pb.DIRECT
# CONNECT_MODE = pb.GUI

DUCK_OBJ_PATH = os.path.join(pybullet_data.getDataPath(), 'duck.obj')


def test_obj_adder_single():

    ground_plane = GroundPlaneHook()

    duck = OBJCreatorHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,1]),
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=10.,
        hooks=[
            ground_plane,
            duck,
            ],
        )

    sim.reset()
    while sim.running:
        sim.step() 
    assert len(sim.objects) == 1

def test_obj_adder_multiple_in_sequence():

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

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=10.,
        hooks=[
            ground_plane,
            duck_1,
            duck_2,
            duck_3,
            ],
        )

    out = sim.reset()
    assert len(sim.objects) == 1
    while sim.running:
        out = sim.step()
    assert len(sim.objects) == 3

def test_obj_adder_multiple_simultaneous():

    ground_plane = GroundPlaneHook()

    time_to_create = 1.


    duck_1 = OBJCreatorHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([-1,-1,2]),
        time_to_create = time_to_create,
        )

    duck_2 = OBJCreatorHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([0,0,4]),
        time_to_create = time_to_create,
        )

    duck_3 = OBJCreatorHook(
        path_to_obj = DUCK_OBJ_PATH,
        position = np.array([1,1,6]),
        time_to_create = time_to_create,
        )

    sim = BulletSimulator(
        mode=CONNECT_MODE,
        max_time=10.,
        hooks=[
            ground_plane,
            duck_1,
            duck_2,
            duck_3,
            ],
        )

    out = sim.reset()
    while sim.running:
        out = sim.step()
    assert len(sim.objects) == 3
