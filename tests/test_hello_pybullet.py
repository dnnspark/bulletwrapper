from bulletwrapper import BulletSimulator, StopSimulation
import pybullet as pb
import pybullet_data
from bulletwrapper import BulletHook

def test_hello_pybullet():
    sim = BulletSimulator(mode=pb.DIRECT, max_time=5.)

    sim.reset()
    while True:
        try:
            sim.step()
        except StopSimulation:
            break;

class PlaneAdder(BulletHook):

    _id = 'plane_adder'

    def after_reset(self, pb_state):
        pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        plane_id = pb.loadURDF("plane.urdf")

class R2D2Adder(BulletHook):

    _id = 'r2d2_adder'

    # def after_init(self):
    #     pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

    def after_reset(self, pb_state):
        start_position = [0,0,1]
        start_rotation = pb.getQuaternionFromEuler([0,0,0])
        self.box_id = pb.loadURDF("r2d2.urdf", start_position, start_rotation)

    def after_step(self, pb_state, step_output):

        position, orientation = pb.getBasePositionAndOrientation(self.box_id)

def test_r2d2():

    plane_adder = PlaneAdder()
    r2d2_adder = R2D2Adder()

    sim = BulletSimulator(
            mode = pb.GUI,
            max_time = 2.,
            hooks = [
                plane_adder,
                r2d2_adder,
            ],
        )

    sim.reset()
    while True:
        try:
            sim.step()
        except StopSimulation:
            break;
