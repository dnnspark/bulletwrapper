import pybullet as pb 
import collections
import itertools 
import re
from bulletwrapper.dataset import Pose, ObjectPose

class BulletSimulator():
    '''
    openai-gym style interface of pybullet.
    '''

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
        self.terminated = None

    @property
    def running(self):
        assert self.terminated is not None
        return not self.terminated

    def init(self):

        pb.setGravity(0, 0, -9.81)
        pb.setTimeStep(self.timestep)
        pb.setPhysicsEngineParameter(numSolverIterations=40)

    def reset(self):

        pb.resetSimulation(self.client_id)
        self.init()
        self.sim_time = 0.
        self.objects = []
        self.terminated = False

        reset_output = HooksOutput()
        for hook in self.hooks:
            output = hook.after_reset(self)
            if output is not None:
                reset_output.add(hook.id, output)

        return reset_output

    def step(self):
        '''
        Proceed one step in simulation.
        1. pb.stepSimulation()
        2. Call .after_step() methods of all hooks.
            - it can use outputs of previous hooks
            - if any of the hook raise StopSimulation, 
                all outputs of the current step are ignored,
                and the before_end() methods are called (see .terminate()).
        '''

        try:
            step_output = self._step()
            return step_output
        except StopSimulation:
            exit_output = self._terminate()
            return exit_output

    def _step(self):

        if self.max_time is not None and self.sim_time + self.timestep > self.max_time:
            raise StopSimulation

        pb.stepSimulation(self.client_id)
        self.sim_time += self.timestep

        step_output = HooksOutput()
        for hook in self.hooks:
            output = hook.after_step(self, step_output)
            if output is not None:
                step_output.add(hook.id, output)

        return step_output

    def _terminate(self):

        exit_output = HooksOutput()
        for hook in self.hooks:
            output = hook.before_end(self, exit_output)
            if output is not None:
                exit_output.add(hook.id, output)

        for hook in self.hooks:
            hook.close()

        pb.disconnect()
        self.terminated = True

        return exit_output

    def get_object_poses(self):

        object_poses = []
        for obj_info in self.objects:
            path_to_obj, body_id = obj_info
            pos, quat =  pb.getBasePositionAndOrientation(body_id)
            object_pose = ObjectPose(path_to_obj, Pose(pos, quat))

            object_poses.append(object_pose)

        return object_poses

class InstanceCounterMeta(type):
    '''
    Every instance of the class made using this meta-class has unique id.
    '''

    def __init__(cls, name, bases, attrs):
        super().__init__(name, bases, attrs)
        cls._ids = itertools.count(1)

def camel2snake(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

class BulletHook(metaclass=InstanceCounterMeta):
    '''
    This hook is a template interface for adding modular components to BulletSimulation.
    It tells what to do at each stage in simulation.
    1. after_reset()
        This tells what to do after pb.resetSimulation() and setting up basic simulation parameters.
        See BulletSimulation.reset().
    2. after_step()
        This tells what to do after pb.stepSimulation()
        See BulletSimulation._step()
    3. before_end()
        This tells what to do before terminating the simulation.
        The termination is triggered by time-up, or one of the hook's call for termination.
        See BulletSimulation._terminate().
    '''

    def __new__(cls, *args, **kwargs):
        instance = super().__new__(cls)
        instance._id = camel2snake( cls.__name__ ) + '-%02d' % next(cls._ids)

        return instance

    @property
    def id(self):
        return self._id

    def after_reset(self, sim):
        pass   

    def after_step(self, sim, hooks_output):
        pass   

    def before_end(self, sim, hooks_output):
        pass

    def close(self):
        pass

class HooksOutput():

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


ObjectInfo = collections.namedtuple('ObjectInfo', 'path_to_obj body_id')

