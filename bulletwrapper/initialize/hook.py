import collections
import os
import tempfile

import numpy as np
import pybullet as pb
import pybullet_data
import trimesh

from bulletwrapper.session import BulletHook, BulletSession
from bulletwrapper.util.logger import setup_logger

from .grid_layers import sample_grid_layers

LOG = setup_logger(__name__, None)

Mesh = collections.namedtuple(
    "Mesh", ["mesh_path", "urdf_path", "obj_path", "vertices"]
)


class GridLayersInitializer(BulletHook):
    """
    """

    def __init__(self, cfg):
        # mesh_paths,
        # weights_on_models,
        # num_layers,
        # layer_size,
        # layer_center,
        # gap_between_layers,
        # only_inplane_rotations,
        # rotation_axis_range,
        # rotation_angle_range,
        # ):

        meshes = []
        unit = cfg.INIT.GRID.MESH_UNIT.lower()
        if unit == "m":
            scale = 1.
        elif unit == "cm":
            scale = 0.01
        elif unit == "mm":
            scale = 0.001
        else:
            raise ValueError("Wrong mesh unit: {}".format(unit))

        for mesh_path in cfg.INIT.GRID.MESH_PATHS:
            LOG.info("Loading {}".format(mesh_path))
            mesh = trimesh.load_mesh(mesh_path)
            if scale != 1.:
                mesh.apply_scale(scale)
            tmpdir = tempfile.mkdtemp()
            trimesh.exchange.urdf.export_urdf(mesh, tmpdir)
            prefix = os.path.join(tmpdir, os.path.basename(tmpdir))
            urdf_path = prefix + ".urdf"
            obj_path = prefix + "_convex_piece_0.obj"
            decimated_mesh = trimesh.load_mesh(obj_path)
            meshes.append(Mesh(mesh_path, urdf_path, obj_path, np.array(decimated_mesh.vertices)))

        self._meshes = meshes
        self._cfg = cfg


    def after_reset(self, sess):
        # load infinite plane
        data_path = pybullet_data.getDataPath()
        plane_id = pb.loadURDF(os.path.join(data_path, "plane.urdf"))

        # # A list of path to the mesh files; typically .obj or .ply
        # _C.INIT.GRID.MESH_PATHS = ()
        # # Frequency weight on each model.
        # # By default, it uses uniform distribution over the models.
        # _C.INIT.GRID.WEIGHTS_ON_MODEL = (1,) # uniform
        # # Number of layers.
        # _C.INIT.GRID.NUM_LAYERS = 3
        # # Range of xy-dimension of layers.
        # # ((x_min, y_min), (x_max, y_max))
        # _C.INIT.GRID.LAYER_SIZE = ((1., 1.), (1., 1.))
        # # Range of center of layers.
        # # ((x_min, y_min), (x_max, y_max))
        # _C.INIT.GRID.LAYER_CENTER = ((0., 0), (0., 0.))
        # # Extra-padding between layers.
        # # The height of each layer is computed as following:
        # # if ONLY_INPLANE_ROATION is True:
        # #   max(width, height, depth) + GAP_BETWEEN_LAYERS
        # # else:
        # #   diagonal_of_bounding_cube + GAP_BETWEEN_LAYERS
        # _C.INIT.GRID.GAP_BETWEEN_LAYERS = 0.01
        # # If True, use only inplane rotation for model poses;
        # # If False, allow arbitrary 3D rotation.
        # _C.INIT.GRID.ONLY_INPLANE_ROTATION = True
        # # The 3D rotation uses (axis, angle) paramterization.
        # # The axis is a unit-norm 3D vector.
        # # The axis is uniformly sampled from a 3D box represented as
        # #   ((x_min, y_min, z_min), (x_max, y_max, z_max))
        # # , and then normalized to unit-norm.
        # _C.INIT.GRID.ROTATION_AXIS_RANGE = ((-1., -1., -1.), (1., 1., 1.))
        # # The rotation angle is uniformly sampled.
        # _C.INIT.GRID.ROTATION_RANGE = (-30., 30.)
        #     num_layers,
        #     model_vertices,
        #     model_weights,
        #     layer_size_range,
        #     layer_center_range,
        #     use_only_inplane_rotation,
        #     rotation_axis_range,
        #     rotation_angle_range,
        #     seed,

        GRID = self._cfg.INIT.GRID

        objects = sample_grid_layers(
            num_layers=GRID.NUM_LAYERS,
            model_vertices=[mesh.vertices for mesh in self._meshes],
            model_weights=GRID.WEIGHTS_ON_MODEL,
            layer_size_range=GRID.LAYER_SIZE,
            layer_center_range=GRID.LAYER_CENTER,
            use_only_inplane_rotation=GRID.ONLY_INPLANE_ROTATION,
            rotation_axis_range=GRID.ROTATION_AXIS_RANGE,
            rotation_angle_range=GRID.ROTATION_ANGLE_RANGE,
            gap_between_layers=GRID.GAP_BETWEEN_LAYERS,
            )

