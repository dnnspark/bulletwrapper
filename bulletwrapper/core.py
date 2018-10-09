import pybullet as pb 
import abc

class BulletSimulator():

    def __init__(self,
        mode = pb.GUI,
        timestep = 0.0002,
        hooks = [],
        max_time = None, # indefinite
        ):

        self.client_id = pb.connect(mode)
        self.timestep = timestep
        self.hooks = hooks
        self.max_time = max_time

    def init(self):

        pb.setGravity(0, 0, -9.81)
        pb.setTimeStep(self.timestep)
        pb.setPhysicsEngineParameter(numSolverIterations=40)
        # pb.setAdditionalSearchPath(BENCHMARK_DATASETS_ROOT)

    def reset(self):

        pb.resetSimulation(self.client_id)
        self.init()
        self.sim_time = 0.

        for hook in self.hooks:
            hook.after_reset(BulletState(self))

    def step(self):

        if self.max_time is not None and self.sim_time + self.timestep > self.max_time:
            raise StopSimulation

        pb.stepSimulation(self.client_id)
        self.sim_time += self.timestep

        step_output = StepOutput()
        for hook in self.hooks:
            out = hook.after_step(BulletState(self), step_output)
            step_output.add(hook.id, out)

class BulletHook(abc.ABC):

    @property
    def id(self):
        return self._id

    def after_reset(self, pb_state):
        pass   

    def after_step(self, pb_state, step_output):
        pass   

class BulletState():

    def __init__(self, pb_simulator):
        self.sim_time = pb_simulator.sim_time

class StepOutput():

    def __init__(self):
        self.output = {}

    def add(self, hook_id, out):
        assert not hook_id in self.output
        self.output[hook_id] = out

class StopSimulation(Exception):
    pass


