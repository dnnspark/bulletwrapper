from yacs.config import CfgNode as CN

_C = CN()

# numpy random seed
_C.NUMPY_RANDOM_SEED = 1234

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
# gravity constant
_C.BULLET.GRAVITY = -9.81

#-----------------------------------------------------------------------------
# Scene set configuration
#-----------------------------------------------------------------------------
# Number of scenes
_C.NUM_SCENES = 10

#-----------------------------------------------------------------------------
# Object pose initialization configuration.
#-----------------------------------------------------------------------------
_C.INIT = CN()
# For now, "grid" initialzation is the only option.
_C.INIT.USE_GRID = True
# Use inifinite groundplane on z=0.
_C.INIT.USE_GROUNDPLANE = True

#-----------------------------------------------------------------------------
# Grid initialization configuration.
#-----------------------------------------------------------------------------
_C.INIT.GRID = CN()
# A list of path to the mesh files; typically .obj or .ply
_C.INIT.GRID.MESH_PATHS = ()
# Length unit of mesh:
_C.INIT.GRID.MESH_UNIT = "m"
# Frequency weight on each model.
# By default, it uses uniform distribution over the models.
_C.INIT.GRID.WEIGHTS_ON_MODEL = (1,) # uniform
# Number of layers.
_C.INIT.GRID.NUM_LAYERS = 3
# Range of xy-dimension of layers.
# ((x_min, y_min), (x_max, y_max))
_C.INIT.GRID.LAYER_SIZE = ((1., 1.), (1., 1.))
# Range of center of layers.
# ((x_min, y_min), (x_max, y_max))
_C.INIT.GRID.LAYER_CENTER = ((0., 0), (0., 0.))
# Extra-padding between layers.
# The height of each layer is computed as following:
# if ONLY_INPLANE_ROATION is True:
#   max(width, height, depth) + GAP_BETWEEN_LAYERS
# else:
#   diagonal_of_bounding_cube + GAP_BETWEEN_LAYERS
_C.INIT.GRID.GAP_BETWEEN_LAYERS = 0.
# If True, use only inplane rotation for model poses;
# If False, allow arbitrary 3D rotation.
_C.INIT.GRID.ONLY_INPLANE_ROTATION = True
# The 3D rotation uses (axis, angle) paramterization.
# The axis is a unit-norm 3D vector.
# The axis is uniformly sampled from a 3D box represented as
#   ((x_min, y_min, z_min), (x_max, y_max, z_max))
# , and then normalized to unit-norm.
_C.INIT.GRID.ROTATION_AXIS_RANGE = ((-1., -1., -1.), (1., 1., 1.))
# Range of rotation angle in degree.
_C.INIT.GRID.ROTATION_ANGLE_RANGE = (-30., 30.)




#-----------------------------------------------------------------------------
# Output options
#-----------------------------------------------------------------------------
_C.OUTPUT = CN()

# root of output directory
_C.OUTPUT.OUTPUT_DIR = "."
# protobuf files writing
_C.OUTPUT.SCENE_GEOMETRIES_FILE = "scene_geometries.pb"

def get_default_cfg():
  """
  Get a yacs CfgNode object with default values for my_project.
  Return a clone so that the defaults will not be altered
  This is for the "local variable" use pattern
  """
  return _C.clone()
