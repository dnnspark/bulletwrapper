import os
import numpy as np
from imageio import imwrite
import yaml
from bulletwrapper.util.transformations import quaternion_from_matrix

class Pose():

    def __init__(self, t, R):
        self.t = t 
        self.R = R 

    def to_dict(self):

        R, t = self.R, self.t
        if isinstance(R, np.ndarray) and R.shape == (3,3):
            R = quaternion_from_matrix(R)
            # R = np.ravel(R)

        t = [float(x) for x in t]
        R = [float(x) for x in R]

        dct = {
            't': t,
            'R': R,
        }
        return dct

class ObjectPose():

    def __init__(self, path_to_obj, mesh_scale, pose, category_name):
        self.path_to_obj = path_to_obj
        self.mesh_scale = mesh_scale
        self.pose = pose
        self.category_name = category_name

    def to_dict(self):
        dct = {
            'path_to_obj': self.path_to_obj,
            'mesh_scale': self.mesh_scale,
            'object_to_world': self.pose.to_dict(),
            'category_name': self.category_name,
        }
        return dct


class Example():
    '''
    Data fields
    ===========
        - path_to_rgb: str
        - path_to_depth: str
        - path_to_label: str
        - min_depth: float
        - max_depth: float
        - object_poses: [ObjectPose]
        - cam_pose: Pose
        - intrinsics: list
        - sim_time: float
    '''

    def __init__(self, 
            path_to_rgb, path_to_depth, path_to_label, 
            min_depth, max_depth,
            object_poses, cam_pose, K,
            sim_time):

        self.path_to_rgb = path_to_rgb
        self.path_to_depth = path_to_depth
        self.path_to_label = path_to_label
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.object_poses = object_poses
        self.cam_pose = cam_pose
        self.K = K
        self.sim_time = sim_time

    def to_dict(self):

        K = self.K
        if isinstance(self.K, np.ndarray):
            K = K.ravel()
        K = [float(x) for x in K]

        dct = {
            'path_to_rgb': self.path_to_rgb,
            'path_to_depth': self.path_to_depth,
            'path_to_label': self.path_to_label,

            'min_depth': float(self.min_depth),
            'max_depth': float(self.max_depth),

            'objects': [obj_pose.to_dict() for obj_pose in self.object_poses],
            'world_to_cam': self.cam_pose.to_dict(),
            'intrinsics': K,
        }
        if self.sim_time is not None:
            dct['sim_time'] = float(self.sim_time)
        return dct

def maybe_makedirs(dirpath):

    if not os.path.exists(dirpath):
        os.makedirs(dirpath)

class DatasetWriter():
    '''
    dataset_writer.write(
        rgb, depth, label,
        object_poses, 
        cam_pose, K,
        )
    '''


    def __init__(self, 
        # path_to_rgb, path_to_depth, path_to_label, 
        path_to_dataset,
        base_format,
        path_to_yml,
        ):


        maybe_makedirs( path_to_dataset )

        rgb_path = os.path.join('rgb/', base_format)
        depth_path = os.path.join('depth/', base_format)
        label_path = os.path.join('label/', base_format)

        maybe_makedirs( os.path.dirname(os.path.join(path_to_dataset, rgb_path))  )
        maybe_makedirs( os.path.dirname(os.path.join(path_to_dataset, depth_path)) ) 
        maybe_makedirs( os.path.dirname(os.path.join(path_to_dataset, label_path)) ) 

        self.dataset_root = path_to_dataset

        self.rgb_path = rgb_path
        self.depth_path = depth_path
        self.label_path = label_path

        self.yml_file = open(path_to_yml, 'w')
        self.idx = 0

    def _write_rgb(self, rgb, idx):
        _filename = self.rgb_path % idx
        filename = os.path.join( self.dataset_root, _filename )
        imwrite(filename, rgb)

        return _filename

    def _write_depth(self, depth, idx):

        min_depth, max_depth = np.min(depth), np.max(depth)

        D = (depth - min_depth) / (max_depth - min_depth)
        D = np.uint8( 255. * D)

        _filename = self.depth_path % idx
        filename = os.path.join( self.dataset_root, _filename )
        imwrite(filename, D)

        return _filename, min_depth, max_depth

    def _write_label(self, label, idx):

        _filename = self.label_path % idx
        filename = os.path.join( self.dataset_root, _filename )
        imwrite(filename, label)

        return _filename


    def write(self, rgb, depth, label, object_poses, cam_pose, K, sim_time=None):

        idx = self.idx

        rgb_file = self._write_rgb(rgb, idx)
        depth_file, min_depth, max_depth = self._write_depth(depth, idx)
        label_file = self._write_label(label, idx)

        ex = Example(
            rgb_file,
            depth_file,
            label_file,
            min_depth,
            max_depth,
            object_poses,
            cam_pose,
            K,
            sim_time,
            )

        anno = [ex.to_dict()]
        self.yml_file.write(yaml.dump(anno))

        self.idx += 1

    def close(self):
        self.yml_file.close()
