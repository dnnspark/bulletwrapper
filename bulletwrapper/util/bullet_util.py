import numpy as np
import pybullet as pb
from bulletwrapper.util.transformations import quaternion_from_matrix as R2quat

def add_obj(path_to_obj, 
            position, orientation,
            fix_base, scale, 
            base_mass, base_inertialframe_position, 
            texture_file=None, rgb=None):
    '''
    path_to_obj: str
    orientation: np.ndarray, (3,3) or (4,)
    scale: float
    fix_base: bool
    texture_file: str or None
    rgb: (float, float, float) or (3,) np.ndarray
    '''

    R = orientation

    assert isinstance(R, np.ndarray)
    if R.shape == (3,3):
        quat = R2quat(R)
    else:
        assert len(R) == 4
        quat = R 

    collision_shape_id = pb.createCollisionShape(shapeType=pb.GEOM_MESH, meshScale=np.full(3, scale), fileName=path_to_obj)
    visual_shape_id = pb.createVisualShape(shapeType=pb.GEOM_MESH, meshScale=np.full(3, scale), fileName=path_to_obj)
    if fix_base:
        # simply omitting baseMass makes it fixed object.
        bullet_body_id = pb.createMultiBody(
                            basePosition=position, baseOrientation=quat, 
                            baseCollisionShapeIndex=collision_shape_id, baseVisualShapeIndex=visual_shape_id,
                            )
    else:
        bullet_body_id = pb.createMultiBody(
                            baseMass=base_mass, baseInertialFramePosition=base_inertialframe_position,
                            basePosition=position, baseOrientation=quat,
                            baseCollisionShapeIndex=collision_shape_id, baseVisualShapeIndex=visual_shape_id,
                            )

    if texture_file is not None:
        texture_id = pb.loadTexture(texture_file)
    else:
        texture_id = None

    if rgb is not None:
        rgba = [c for c in rgb] + [1.]
    else:
        rgba = None

    if texture_id is not None and rgba is not None:
        pb.changeVisualShape(bullet_body_id, -1, textureUniqueId=texture_id, rgbaColor=rgba)
    elif texture_id is not None and rgba is None:
        pb.changeVisualShape(bullet_body_id, -1, textureUniqueId=texture_id)
    elif texture_id is None and rgba is not None:
        pb.changeVisualShape(bullet_body_id, -1, rgbaColor=rgba)

    return bullet_body_id