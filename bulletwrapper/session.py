import collections
import itertools
import re

import numpy as np
import pybullet as pb

from bulletwrapper.dataset import ObjectPose, Pose


class BulletSession:
    """
    openai-gym style interface of pybullet.
    """

    def __init__(
        self,
        mode=pb.GUI,
        timestep=0.0002,
        max_time=None,  # indefinite
        num_solver_iterations=20,
        gravity=-9.81,
        hooks=[],
        seed=None,
    ):

        self.client_id = pb.connect(mode)
        self.timestep = timestep
        self.max_time = max_time
        self.num_solver_iterations = num_solver_iterations
        self.gravity = gravity
        self.hooks = hooks
        # self.buffers = {}

        self.terminated = None

        if seed:
            np.random.seed(seed)

    def __enter__(self):
        return self

    def __exit__(self, exctype, value, traceback):
        self.close()

    @property
    def is_running(self):
        assert self.terminated is not None
        return not self.terminated

    def register_hook(self, hook):
        self.hooks.append(hook)

    def register_buffer(self, name, buffer_data):
        assert name not in self.buffers
        self.buffers[name] = buffer_data

    def clear_buffer(self, name):
        if name in self.buffers:
            del self.buffers[name]

    def get_buffer(self, name):
        return self.buffers.get(name, None)

    def init(self):

        # These are overridden after pb.resetSimulation().
        pb.setGravity(0, 0, self.gravity)
        pb.setTimeStep(self.timestep)
        pb.setPhysicsEngineParameter(numSolverIterations=self.num_solver_iterations)

        self.sim_time = 0.0
        self.terminated = False

    def reset(self):

        pb.resetSimulation(self.client_id)
        self.init()
        self.buffers = {}

        reset_output = []
        for hook in self.hooks:
            output = hook.after_reset(self)
            reset_output.append(output)

        return reset_output

    def step(self):
        """
        Proceed one step in simulation.
        1. pb.stepSimulation()
        2. Call .after_step() methods of all hooks.
            - it can use outputs of previous hooks
            - if any of the hook raise StopSimulation,
                all outputs of the current step are ignored,
                and the before_end() methods are called (see .terminate()).
        """

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

        step_output = []
        for hook in self.hooks:
            output = hook.after_step(self, step_output)
            step_output.append(output)

        return step_output

    def _terminate(self):

        exit_output = []
        for hook in self.hooks:
            output = hook.before_end(self, exit_output)
            exit_output.append(output)

        self.terminated = True

        return exit_output

    def close(self):
        for hook in self.hooks:
            hook.close()
        pb.disconnect()


class BulletHook:
    """
    This hook is a template interface for adding modular components to BulletSimulation.
    It tells what to do at each stage in simulation.
    1. after_reset()
        This tells what to do after pb.resetSimulation() and setting up basic simulation parameters.
        See BulletSimulation.reset().
    2. after_step()
        This tells what to do after pb.stepSimulation()
        See BulletSimulation._step()
    3. before_end()
        This tells what to do before terminating one rollout of the simulation.
        The termination is triggered by time-up, or one of the hook's call for termination.
        See BulletSimulation._terminate().
    4. close()
        This tells what to do before closing the simulator.
    """

    def after_reset(self, sess):
        pass

    def after_step(self, sess, hooks_output):
        pass

    def before_end(self, sess, hooks_output):
        pass

    def close(self):
        pass


class StopSimulation(Exception):
    pass
