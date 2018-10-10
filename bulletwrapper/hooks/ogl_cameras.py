import os
import numpy as np
import pybullet as pb
from bulletwrapper import BulletHook

class StaticOGLCameraHook(BulletHook):

    def __init__(self,
            position,
            lookat = np.array([0., 0., 0.]),
            up = np.array([0., 1., 0.]),
            start = 0.,
            interval = None,
        ):

        self.start = start
        self.interval = interval
        self.last_caputured = None


    def after_reset(self, pb_state):
        if pb_state.sim_time >= self.start:
            img = self.capture()
            self.last_caputured = 0.
            return img

    def after_step(self, pb_state, step_output):

        sim_time = pb_state.sim_time

        if sim_time >= self.start and \
            ( self.last_caputured is None or sim_time - self.last_caputured >= self.interval ):
            img = self.capture()
            self.last_caputured = sim_time

            return img

    def capture(self):
        return np.zeros((3,3), np.uint8)

