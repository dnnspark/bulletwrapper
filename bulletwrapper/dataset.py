import numpy as np

class Pose():

    def __init__(self, pos, quat):
        self.pos = pos
        self.quat = quat

    def to_dict(self):
        dct = {
            'pos': list(self.pos),
            'quat': list(self.quat),
        }
        return dct

class ObjectPose():

    def __init__(self, path_to_obj, pose):
        self.path_to_obj = path_to_obj
        self.pose = pose

    def to_dict(self):
        dct = {
            'path_to_rgb': self.path_to_rgb,
            'pose': self.pose.to_dict(),
        }
        return dct


class Example():
    '''
    Data fields
    ===========
        - path_to_rgb: str
        - object_poses: [ObjectPose]
        - cam_pose: Pose
        - intrinsics: list
    '''

    def __init__(self, path_to_rgb, object_poses, cam_pose, K):
        self.path_to_rgb = path_to_rgb
        self.object_poses = object_poses
        self.cam_pose = cam_pose
        self.K = K

    def to_dict(self):

        K = self.K
        if isinstance(self.K, np.ndarray):
            K = list(K.ravel())
        else:
            K = list(K)

        dct = {
            'path_to_rgb': self.path_to_rgb,
            'objects': [obj_pose.to_dict() for obj_pose in self.object_poses],
            'cam_pose': self.cam_pose.to_dict(),
            'intrinsics': K,
        }
        return dct

class DatasetMaker():
    '''
    * Container for the following info:
        - where to save images?
        - where to save yaml file?
        - where kk

    * Functionalities:

    dataset_writer = DatasetWriter(
        path_to_rgb,
        path_to_depth,
        path_to_segment,
        file_format,
        path_to_yml,
    )



    img = self.capture()
    object_poses = sim.get_object_poses()
    cam_pose, K = self.cam_params

    dataset_writer.write(img, object_poses, cam_pose, K)



    '''




#     def __init__(self)

# class Dataset():

if __name__ == '__main__':

    import yaml

    x1 = {
        'path_to_rgb': 'some_path', 
        'path_to_obj': 'another_path',
        'pose': {
            'pos': [0.,0.,0.],
            'quat': [0.,0.,0.,1.],
            },
        'cam_pose': {
            'pos': [0.,0.,0.],
            'quat': [0.,0.,0.,1.],
            },
        'intrinsics': [1., 0., 1., 0., 1.,1., 0., 0., 1.],
        }

    x2 = {
        'path_to_rgb': 'some_path_2', 
        'path_to_obj': 'another_path_2',
        'pose': {
            'pos': [0.,0.,0.],
            'quat': [0.,0.,0.,1.],
            },
        'cam_pose': {
            'pos': [0.,0.,0.],
            'quat': [0.,0.,0.,1.],
            },
        'intrinsics': [1., 0., 1., 0., 1.,1., 0., 0., 1.],
        }

    with open('tmp.yml','w') as f:
        f.write( yaml.dump([x1,x2]) )
    import pdb; pdb.set_trace()
