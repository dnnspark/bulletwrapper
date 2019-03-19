import logging
import os
import numpy as np

import pybullet_data
import pybullet as pb

from bulletwrapper.session import BulletHook, BulletSession
from bulletwrapper.util.logger import setup_logger

LOG = setup_logger(__name__, None)


class SimpleSceneSetup(BulletHook):

    def after_reset(self, sess):

        data_path = pybullet_data.getDataPath()

        plane_id = pb.loadURDF(os.path.join(data_path, "plane.urdf"))

        position = np.array([0, 0, 3])
        orientation = np.array([0, 0, 0, 1.])

        r2d2_body_id = pb.loadURDF(os.path.join(data_path, "r2d2.urdf"), position, orientation)

        sess.register_buffer("r2d2_body_id", r2d2_body_id)

def main():

    kwargs = dict(
        max_time=3.
        )

    with BulletSession(**kwargs) as sess:
        sess.register_hook(SimpleSceneSetup())
        sess.reset()
        while sess.is_running:
            sess.step()
    LOG.info("Session is closed.")

if __name__ == '__main__':
    main()
