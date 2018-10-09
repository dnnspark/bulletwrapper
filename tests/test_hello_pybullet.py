from bulletwrapper import BulletSimulator, StopSimulation
import pybullet as pb

def test_hello_pybullet():
    sim = BulletSimulator(mode=pb.DIRECT, max_time=5.)
    sim.reset()

    while True:
        try:
            sim.step()
        except StopSimulation:
            break;
