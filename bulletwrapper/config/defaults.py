from yacs.config import CfgNode as CN

_C = CN()

#-----------------------------------------------------------------------------
# Bullet configuration.
#-----------------------------------------------------------------------------

_C.BULLET = CN()
# If "GUI", run real-time pybullet built-in visualizer; if "DIRECT", not.
_C.BULLET.MODE = "GUI"
# timestep in physics simulation.
_C.BULLET.TIMESTEP = 0.0002
# maximum duration of each simulation (-1 if indefinite)
_C.BULLET.MAX_TIME = -1.
# numSolverIteration parameter of bullet
_C.BULLET.NUM_SOLVER_ITERATIONS = 20

#-----------------------------------------------------------------------------
# Scene set configuration
#-----------------------------------------------------------------------------
# Number of scenes
_C.NUM_SCENES = 10
# A list of path to the mesh files; typically .obj or .ply
_C.MESH_PATH = ()

#-----------------------------------------------------------------------------
# Object pose initialization configuration.
#-----------------------------------------------------------------------------
_C.INIT = CN()
# For now, "grid" initialzation is the only option.
_C.INIT.USE_GRID = True

#-----------------------------------------------------------------------------
# Grid initialization configuration.
#-----------------------------------------------------------------------------
_C.INIT.GRID = CN()
# Number of layers.
_C.INIT.GRID.NUM_LAYERS = 3
# If True, use LAYER_SIZE; if Flase use NUM_OBJECTS_PER_LAYER
_C.INIT.GRID.USE_LAYER_SIZE = True
_C.INIT.GRID.NUM_OBJECTS_PER_LAYER = (4, 4)
_C.INIT.GRID.LAYER_SIZE = (1., 1.)
# Extra-padding between layers.
# The height of each layer is computed as following:
# if ONLY_INPLANE_ROATION is True:
#   max(width, height, depth) + GAP_BETWEEN_LAYERS
# else:
#   diagonal_of_bounding_cube + GAP_BETWEEN_LAYERS
_C.INIT.GRID.GAP_BETWEEN_LAYERS = 0.01
# If True, use only inplane rotation for model poses;
# If False, allow arbitrary 3D rotation.
_C.INIT.GRID.ONLY_INPLANE_ROTATION = True
# The 3D rotation uses (axis, angle) paramterization.
# The axis is a unit-norm 3D vector.
# The axis is uniformly sampled from a 3D box represented as
#   ((x_min, y_min, z_min), (x_max, y_max, z_max))
# , and then normalized to unit-norm.
_C.INIT.GRID.ROTATION_AXIS_RANGE = ((-1., -1., -1.), (1., 1., 1.))
# The rotation angle is uniformly sampled.
_C.INIT.GRID.ROTATION_RANGE = (-30., 30.)




#-----------------------------------------------------------------------------
# Misc. options
#-----------------------------------------------------------------------------
_C.OUTPUT_DIR = "."

def get_default_cfg():
  """
  Get a yacs CfgNode object with default values for my_project.
  Return a clone so that the defaults will not be altered
  This is for the "local variable" use pattern
  """
  return _C.clone()
