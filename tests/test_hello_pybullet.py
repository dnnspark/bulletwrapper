import os
import pybullet as pb
import pybullet_data
from bulletwrapper import BulletSimulator, StopSimulation
from bulletwrapper import BulletHook
from bulletwrapper.hooks import GroundPlaneHook

# CONNECT_MODE = pb.GUI
CONNECT_MODE = pb.DIRECT

def test_hello_pybullet():
    sim = BulletSimulator(mode=CONNECT_MODE, max_time=5.)

    sim.reset()
    while not sim.terminated:
        sim.step()

class R2D2CreatorHook(BulletHook):

    def __init__(self):
        self.r2d2_urdf_path = os.path.join(pybullet_data.getDataPath(), 'r2d2.urdf')

    def after_reset(self, sim):
        start_position = [0,0,1]
        start_rotation = pb.getQuaternionFromEuler([0,0,0])
        self.box_id = pb.loadURDF(self.r2d2_urdf_path, start_position, start_rotation)

    def after_step(self, sim, hooks_output):

        position, orientation = pb.getBasePositionAndOrientation(self.box_id)

def test_r2d2():

    ground_plane = GroundPlaneHook()
    r2d2 = R2D2CreatorHook()

    sim = BulletSimulator(
            mode = CONNECT_MODE,
            max_time = 2,
            hooks = [
                ground_plane,
                r2d2,
            ],
        )

    sim.reset()
    while not sim.terminated:
        output = sim.step()

    # while True:
    #     try:
    #         sim.step()
    #     except StopSimulation:
    #         break;
