import pybullet as pb

from bulletwrapper.initialize.hook import GridLayersInitializer
from bulletwrapper.proto.geometric_pb2 import SceneGeometry
from bulletwrapper.session import BulletHook, BulletSession


def bullet_kwargs_from_cfg(cfg):
    """
    """
    if cfg.BULLET.MODE.upper() == "GUI":
        mode = pb.GUI
    elif cfg.BULLET_MODE.upper() == "DIRECT":
        mode = pb.DIRECT

    ret = {
        "mode": mode,
        "timestep": cfg.BULLET.TIMESTEP,
        "max_time": cfg.BULLET.MAX_TIME,
        "num_solver_iterations": cfg.BULLET.NUM_SOLVER_ITERATIONS,
        "gravity": cfg.BULLET.GRAVITY,
    }

    return ret


def sample_final_scene(cfg):
    """
    """
    kwargs = bullet_kwargs_from_cfg(cfg)
    with BulletSession(**kwargs) as sess:
        sess.register_hook(GridLayersInitializer(cfg))

        for scene_idx in range(cfg.NUM_SCENES):
            sess.reset()
            while sess.is_running:
                sess.step()

            scene = SceneGeometry()

            yield scene
