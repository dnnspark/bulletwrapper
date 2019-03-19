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
_C.NUM_SOLVER_ITERATIONS = 20

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
