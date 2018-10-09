from bulletwrapper import BulletSimulator, StopSimulation

def test_hello_pybullet():
    sim = BulletSimulator(max_time=5.)
    sim.reset()

    while True:
        try:
            sim.step()
        except StopSimulation:
            break;
