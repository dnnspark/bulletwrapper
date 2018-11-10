import os
import numpy as np
import pybullet as pb
from bulletwrapper import BulletHook
from bulletwrapper.util.camera_util import (
    ogl_viewmat_from_lookat, ogl_projmat, pose_from_lookat)
from bulletwrapper.dataset import Pose
from bulletwrapper.hooks.core import random_position

class OGLCameraHook(BulletHook):
    '''
    Base class for OpenGL Camera. Should be subclassed.
    '''

    def __init__(self, arg, **kwargs):
        raise NotImplementedError()

    def after_reset(self, sim):
        sim_time = sim.sim_time
        self.last_caputured = None
        if sim_time >= self.start:
            return self.process(sim)

    def after_step(self, sim, hooks_output):

        sim_time = sim.sim_time

        if sim_time >= self.start and (
            self.last_caputured is None or (
                 self.interval is not None and sim_time - self.last_caputured >= self.interval
                 )
            ):

            return self.process(sim)

    def before_end(self, sim, hooks_output):

        if self.start == np.inf:
            return self.process(sim)

    def close(self):
        if not self.dataset_writer is None:
            self.dataset_writer.close()

    def process(self, sim):

        rgb, depth, label = self.capture()
        self.last_caputured = sim.sim_time

        if self.dataset_writer is not None:
            object_poses = sim.get_object_poses()
            self.dataset_writer.write(
                rgb, depth,label,
                object_poses, 
                self.world_to_cam, self.K,
                sim.sim_time,
                )

        return rgb, depth, label

    def capture(self):

        H,W = self.img_shape
        pb_P, pb_V = self.pb_P, self.pb_V
        light_src = self.light_src
        img_arr = pb.getCameraImage(
                W, H, pb_V, pb_P, 
                shadow=1, lightDirection=light_src, renderer=self.renderer,
                )

        I = np.uint8( img_arr[2] ).reshape(H,W,4)
        D = np.float64( img_arr[3] ).reshape(H,W)
        L = np.uint8( img_arr[4] ).reshape(H,W)

        return I, D, L

class StaticOGLCameraHook(OGLCameraHook):

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
            renderer = pb.ER_TINY_RENDERER,
            # renderer = pb.ER_BULLET_HARDWARE_OPENGL,
            light_src = [1,1,1],
            start = np.inf,
            interval = None,
            dataset_writer=None
        ):

        self.start = start 
        self.interval = interval
        self.dataset_writer = dataset_writer
        self.last_caputured = None

        # OpenGL camera parameters
        self.img_shape = H,W = img_shape
        K = np.reshape(K, (3,3))

        self.K = K
        R, t = pose_from_lookat(lookat, position, up)
        self.world_to_cam = Pose(t, R)

        projmat = ogl_projmat(K, H, W, z_near=.1, z_far=100)
        viewmat = ogl_viewmat_from_lookat(lookat, position, up)

        # pb.getCameraImage accept row-major list as input
        self.pb_P = list(projmat.T.ravel())
        self.pb_V = list(viewmat.T.ravel())

        self.renderer = renderer
        self.light_src = light_src


class RandomStaticCameraHook(OGLCameraHook):
    '''
    Similar to StaticOGLCameraHook, but allow randomizing the following properties.
        - camera position
        - camera lookat
        - light src
    , all uniform-randomly picked from 3D boxes.
    '''

    def __init__(self,
            # intrinsics
            K,
            img_shape,
            # extrinsics
            position,
            lookat = np.array([0., 0., 0.]),
            up = 'up',
            renderer = pb.ER_TINY_RENDERER,
            # renderer = pb.ER_BULLET_HARDWARE_OPENGL,
            light_src = [1,1,1],
            start = np.inf,
            interval = None,
            dataset_writer=None
        ):

        self.start = start 
        self.interval = interval
        self.dataset_writer = dataset_writer
        self.last_caputured = None

        # OpenGL camera parameters
        self.img_shape = H,W = img_shape
        K = np.reshape(K, (3,3))

        self.K = K
        # R, t = pose_from_lookat(lookat, position, up)
        # self.world_to_cam = Pose(t, R)
        self.position_ = position
        self.lookat_ = lookat
        self.up = up

        projmat = ogl_projmat(K, H, W, z_near=.1, z_far=100)
        # viewmat = ogl_viewmat_from_lookat(lookat, position, up)

        # pb.getCameraImage accept row-major list as input
        self.pb_P = list(projmat.T.ravel())
        # self.pb_V = list(viewmat.T.ravel())

        self.renderer = renderer
        # self.light_src = light_src
        self.light_src_ = light_src

    def set_random_values(self):

        # 1. world-to-cam pose
        # camera position should not be at the (0,0,z) to avoid singularity.
        done = False
        while not done:
            position = random_position(*self.position_)
            if not np.allclose(position[:2], 0):
                done = True

        lookat = random_position(*self.lookat_)
        R, t = pose_from_lookat(lookat, position, self.up)
        self.world_to_cam = Pose(t, R)
        viewmat = ogl_viewmat_from_lookat(lookat, position, self.up)
        self.pb_V = list(viewmat.T.ravel())

        # 2. light src
        self.light_src = list(random_position(*self.light_src_))

    def after_reset(self, sim):
        sim_time = sim.sim_time
        self.last_caputured = None

        self.set_random_values()

        if sim_time >= self.start:
            return self.process(sim)
