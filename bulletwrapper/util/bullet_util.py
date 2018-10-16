import numpy as np
import pybullet as pb
from bulletwrapper.util.transformations import quaternion_from_matrix as R2quat

def add_obj(path_to_obj, rotation, t, fix_base, scale, base_mass, base_inertialframe_position):
    assert isinstance(rotation, np.ndarray)
    if rotation.shape == (3,3):
        quat = R2quat(rotation)
    else:
        assert len(rotation) == 4
        quat = rotation

    collision_shape_id = pb.createCollisionShape(shapeType=pb.GEOM_MESH, meshScale=np.full(3, scale), fileName=path_to_obj)
    visual_shape_id = pb.createVisualShape(shapeType=pb.GEOM_MESH, meshScale=np.full(3, scale), fileName=path_to_obj)
    if fix_base:
        bullet_body_id = pb.createMultiBody(
                            # basePosition=t, baseOrientation=R2quat(R), 
                            basePosition=t, baseOrientation=quat, 
                            baseCollisionShapeIndex=collision_shape_id, baseVisualShapeIndex=visual_shape_id,
                            )
    else:
        bullet_body_id = pb.createMultiBody(
                            baseMass=base_mass, baseInertialFramePosition=base_inertialframe_position,
                            # basePosition=t, baseOrientation=R2quat(R), 
                            basePosition=t, baseOrientation=quat,
                            baseCollisionShapeIndex=collision_shape_id, baseVisualShapeIndex=visual_shape_id,
                            )


    return bullet_body_id