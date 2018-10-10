import pybullet as pb 
import collections
import itertools 
import re

class BulletSimulator():

    def __init__(self,
        mode = pb.GUI,
        timestep = 0.0002,
        hooks = [],
        max_time = None, # indefinite
        ):

        self.client_id = pb.connect(mode)
        self.timestep = timestep
        self.max_time = max_time
        self.hooks = hooks

    def init(self):

        pb.setGravity(0, 0, -9.81)
        pb.setTimeStep(self.timestep)
        pb.setPhysicsEngineParameter(numSolverIterations=40)
        # pb.setAdditionalSearchPath(BENCHMARK_DATASETS_ROOT)

    def reset(self):

        pb.resetSimulation(self.client_id)
        self.init()
        self.sim_time = 0.

        step_output = StepOutput()
        for hook in self.hooks:
            output = hook.after_reset(BulletState(self))
            if output is not None:
                step_output.add(hook.id, output)

        return step_output

    def step(self):

        if self.max_time is not None and self.sim_time + self.timestep > self.max_time:
            raise StopSimulation

        pb.stepSimulation(self.client_id)
        self.sim_time += self.timestep

        step_output = StepOutput()
        for hook in self.hooks:
            output = hook.after_step(BulletState(self), step_output)
            if output is not None:
                step_output.add(hook.id, output)

        return step_output

class InstanceCounterMeta(type):

    def __init__(cls, name, bases, attrs):
        super().__init__(name, bases, attrs)
        cls._ids = itertools.count(1)

def camel2snake(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

class BulletHook(metaclass=InstanceCounterMeta):

    def __new__(cls, *args, **kwargs):
        instance = super().__new__(cls)
        instance._id = camel2snake( cls.__name__ ) + '-%02d' % next(cls._ids)

        return instance

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

    @property
    def is_empty(self):
        return len(self.output) == 0

class StopSimulation(Exception):
    pass


