import os
import numpy as np
import pybullet as pb
from bulletwrapper import BulletHook
from bulletwrapper.util.camera_util import ogl_viewmat_from_lookat, ogl_projmat

class StaticOGLCameraHook(BulletHook):

    '''
    if start == 0.,
        take the first picture upon reset,
    if start == np.inf, 
        take only one picture before the simulation terminates.
    if interval is not None,
        take the first picture at sim_time=start, and then take picture every <interval> seconds until the time is up.
    if interval is None,
        take only one picture when start == sim_time
    '''

    def __init__(self,
            # intrinsics
            K,
            img_shape,
            # extrinsics
            position,
            lookat = np.array([0., 0., 0.]),
            up = 'up',
            light_src = [1,1,1],
            start = np.inf,
            interval = None,
        ):

        self.start = start 
        self.interval = interval
        self.last_caputured = None

        # OpenGL camera parameters
        self.img_shape = H,W = img_shape
        K = np.reshape(K, (3,3))
        projmat = ogl_projmat(K, H, W, z_near=.1, z_far=100)
        viewmat = ogl_viewmat_from_lookat(lookat, position, up)

        # pb.getCameraImage accept row-major list as input
        self.pb_P = list(projmat.T.ravel())
        self.pb_V = list(viewmat.T.ravel())

        self.light_src = light_src

    def after_reset(self, pb_state):
        if pb_state.sim_time >= self.start:
            img = self.capture()
            self.last_caputured = 0.
            return img

    def after_step(self, pb_state, hooks_output):

        sim_time = pb_state.sim_time

        if sim_time >= self.start and (
            self.last_caputured is None or (
                 self.interval is not None and sim_time - self.last_caputured >= self.interval
                 )
            ):
            img = self.capture()
            self.last_caputured = sim_time

            return img

    def before_end(self, pb_state, hooks_output):

        if self.start == np.inf:
            img = self.capture()
            self.last_caputured = pb_state.sim_time
            return img

    def capture(self):

        H,W = self.img_shape
        pb_P, pb_V = self.pb_P, self.pb_V
        light_src = self.light_src
        img_arr = pb.getCameraImage(
                W, H, pb_V, pb_P, 
                shadow=1, lightDirection=light_src, renderer=pb.ER_BULLET_HARDWARE_OPENGL,
                )

        I = np.uint8( img_arr[2] ).reshape(H,W,4)

        return I
