import logging
from collections import namedtuple

import numpy as np

from datagen2d.layout.simple.sampling import maybe_sample
from bulletwrapper.util.multiview_util import to_4x4

LOG = logging.getLogger(__name__)


class CuboidFace(namedtuple("CuboidFace", ["width", "height", "R", "zoffset"])):
    """
    This class represent a 2D rectangle associated with a top-down view of a cuboid.
    Each cuboid gives rise to 6 sides * 4 orientation = 24 axis-aligned blocks.
    The rectangle is represented by its width and height; this data is used to
    compute the layout of blocks.
    The block also has fields of rotation matrix and z-offset, which are used to
    convert the 2D position of block to a full 3D pose of cuboid.

    Attributes
    ----------
    width: int
        width of rectangle
    height: int
        height of rectangle
    R: np.ndarray
        3x3 rotation matrix
    zoffset: float
        the height of model's origin wrt the ground plane.
    """
    __slots__ = ()


def cuboid_to_faces(vertices, inplane_rot_angles, angle_weights, height_offset):
    """
    CRUCIAL assumption:
        vertices collectively represent an axis-aligned
        3D cuboid centered at origin.

    Convert a set of vertices of a 3D model to multiple 2D rectangle
    representation (i.e. CuboidFace). In the simplest case where no
    inplane rotation is considered, a cuboid is converted 24 faces (6
    faces of cuboid x 4 ways to 90-degree rotate each face.) For each
    inplane rotation angle, we consider a tight axis-aligned bounding
    box of the rotated rentangle as a face. Therefore, the final number
    of faces is 6 x 4 x (NUMBER_OF_INPLANE_ROTATION_ANGLES).

    Parameters
    ----------
    vertices: np.ndarray
        (N,3) float tensor, each row is a vertex.
    inplane_rot_angles: [float]
        all possible in-plane rotation angles
    angle_weights: [float]
        distribution of inplane_rot_angles

    Return
    ------
    blocks: [CuboidFace]
        A list of N*24 blocks, where N = len(inplane_rot_angles)

    weights: [float]
        distribution of blocks.
    """

    if len(inplane_rot_angles) != len(angle_weights):
        LOG.error(
            "There must be as many in-place rotation angles as angle weights.")
        raise ValueError

    axes = np.array([
        [1, 0, 0],
        [-1, 0, 0],
        [0, 1, 0],
        [0, -1, 0],
        [0, 0, 1],
        [0, 0, -1],
    ])

    faces, weights = [], []
    for xaxis in axes:
        for yaxis in axes:
            if np.dot(xaxis, yaxis) != 0:
                continue
            zaxis = np.cross(xaxis, yaxis)

            R0 = np.array([xaxis, yaxis, zaxis]).T  # object-to-world rotation

            for idx, theta in enumerate(inplane_rot_angles):
                _theta = np.pi * theta / 180.
                c, s = np.cos(_theta), np.sin(_theta)
                R_inplane = np.array([
                    [c, -s, 0],
                    [s,  c, 0],
                    [0,  0, 1],
                ])
                R = R_inplane.dot(R0)

                transformed = np.dot(R, vertices.T).T
                x1y1z1 = np.min(transformed, axis=0)
                x2y2z2 = np.max(transformed, axis=0)
                # assert np.allclose(x1y1z1 + x2y2z2 , 0) # for perfect cuboid

                diagonal = x2y2z2 - x1y1z1
                W, H = diagonal[:2]

                zoffset = -x1y1z1[-1] + height_offset

                faces.append(CuboidFace(W, H, R, zoffset))
                weights.append(angle_weights[idx])

    return faces, weights


def pack_bin(faces, weights, box, dist_slack=0.):
    """
    2D bin-packing divide-and-conquer algorithm.

    Parameters
    ----------
    faces: [CuboidFaces]
        (N,2) float tensor representing dimensions of 2D faces.
        The first column are widths, the second heights.
    weights: [float]
        distribution offaces
    box: [float]
        (width, height)
    dist_slack: dict or float
        uniform distrubution of slack.

    Return
    ------
    face_layout: [ (int, (float, float)) ]
        a list of (index, (start_x, start_y))
        `index` is index in `faces`.
    """

    faces_dim = np.array([(face.width, face.height) for face in faces])
    slack = maybe_sample(dist_slack)
    # clip the padding, so that any face that can fit without padding
    # can be used. Without this clipping, a number of faces will be
    # classified invalid, which yields a loosely packed scene.
    slack = np.clip(slack, 0., box - faces_dim)

    faces_dim_w_slack = faces_dim + slack

    # randomly choose a face that fits.
    fit = faces_dim_w_slack <= np.expand_dims(box, 0)
    fit, = np.where(fit[:, 0] * fit[:, 1])

    if len(fit) == 0:
        # no faces fit.
        return []

    sub_weights = np.array([weights[k] for k in fit])
    sub_weights /= np.sum(sub_weights)

    # TODO(david) Properly control the random state via an instance of numpy.random.RandomState
    pick = np.random.choice(fit, p=sub_weights)
    face_dim_w_slack = faces_dim_w_slack[pick]

    W, H = box
    w, h = face_dim_w_slack

    # randomly choose one of the two ways to split the remaining area into two rectangular areas.
    split = np.random.choice(2)
    if split == 0:
        # split horizontally.
        two_residual_boxes = [
            [W - w,  h],
            [W,  H - h],
        ]
    else:
        # split vertically.
        two_residual_boxes = [
            [W - w,   H],
            [w,   H - h],
        ]

    # randomly choose one of the four corners to place the face.
    corner = np.random.choice(4)
    if corner == 0:
        # upper-left corner
        layout = [(pick, (0., 0.))]

        two_offsets = [
            [w,  0.],
            [0., h]
        ]

    elif corner == 1:
        # upper-right corner
        layout = [(pick, (W - w, 0.))]

        if split == 0:
            two_offsets = [
                [0., 0.],
                [0., h],
            ]
        else:
            two_offsets = [
                [0., 0.],
                [W - w,  h],
            ]
    elif corner == 2:
        # lower-left corner
        layout = [(pick, (0., H - h))]

        if split == 0:
            two_offsets = [
                [w,  H - h],
                [0., 0.],
            ]
        else:
            two_offsets = [
                [w,  0.],
                [0., 0.],
            ]

    else:  # corner == 3:
        # lower-right corner
        layout = [(pick, (W - w, H - h))]

        if split == 0:
            two_offsets = [
                [0., H - h],
                [0.,  0.],
            ]
        else:
            two_offsets = [
                [0., 0.],
                [W - w, 0.],
            ]

    for residual_box, offsets in zip(two_residual_boxes, two_offsets):
        sub_layout = pack_bin(faces, weights, residual_box, dist_slack)
        sub_layout = [(idx, (start_x + offsets[0], start_y + offsets[1]))
                      for idx, (start_x, start_y) in sub_layout]
        layout += sub_layout

    return layout


def compute_poses(faces, face_layout, box_dim, box_center):
    """
    Convert 2D layout to 3D poses, assuming the commonground assumption.

    Parameters
    -----
    faces: [CuboidFace]
        A list of cuboid faces. output of `cuboid_to_faces`.
    face_layout: [ (int, (float, float)) ]
        a list of (index, (start_x, start_y)), output of `pack_bin()`.
        `index` is index in `faces`.
    box_dim: (float, float)
        (width, height) of box
    box_center: (float, float)
        (x_center, y_center), the position of box center in world coordinate.


    Return
    ------
    poses: [np.ndarray]
        A list of 4x4 np.ndarray, each represents the pose of box in the
        scene in world's frame
    """

    poses = []
    box_dim, box_center = np.array(box_dim), np.array(box_center)
    for idx, (x1, y1) in face_layout:
        face = faces[idx]

        R = face.R
        # The positions of cuboid and container are defined wrt center-of-mass.
        xy_offset = np.array(
            [x1, y1]) + .5 * np.array([face.width, face.height]) - .5 * box_dim + box_center
        z_offset = face.zoffset

        position = np.array(list(xy_offset) + [z_offset])
        poses.append(to_4x4(R, position))

    return poses


def make_packed_cuboids_scene(model_vertices, dist_mesh_scale, dist_box_dim,
                              dist_box_center, inplane_rot_angles, angle_weights,
                              dist_slack, height_offset=0., face_filter=None):
    """
    Entry point of generating randomized scene geometry of tightly
    packed box scene. This returns a set of poses of a cuboid that
    compose a scene that is roughly tightly packed with the cuboid with
    various orientations. To add diversity, one can control
    `inplane_rot_angles` or change `dist_slack` to add more relaxation.

    Parameters
    -----
    model_vertices: (N,3) np.ndarray
        vertcies of model
    dist_mesh_scale: dict or float
        distribution of scaling factor of the mesh
    dist_box_dim: dict or [float]
        distribution of xy dimension of the box
    dist_box_center: dict or [float]
         distribution of xy coordinate of box center
    inplace_rot_angles: [float]
        a list of rotation angles.
        each instance is rotated wrt world's z-axis by an angle randomly chosen from this list.
    angle_weights: [float]
        distribution of inplane_rot_angles
    dist_slack:
        range of slack. this controls how tightly you want to pack the box.
        after rotation, the object will be "padded" by a distance sampled uniformly randomly in this range.
    face_filter: callable
        a function that takes a CuboidFace instance as input and return a boolean.
        This can be used to filter the 2D templates (e.g. excluding some sides of the cuboid.)

    Return
    ------
    poses: [np.ndarray]
        list of 3D poses of objects in world's frame.
    """

    # sample scene-level params
    mesh_scale = maybe_sample(dist_mesh_scale)
    box_dim = maybe_sample(dist_box_dim)
    box_center = maybe_sample(dist_box_center)

    vertices = model_vertices * mesh_scale

    faces, weights = cuboid_to_faces(vertices, inplane_rot_angles, angle_weights, height_offset)
    if face_filter is not None:
        raise ValueError("Not supported now.")
        # faces = [face for face in faces if face_filter(face)]
    face_layout = pack_bin(faces, weights, box_dim, dist_slack=dist_slack)

    poses = compute_poses(faces, face_layout, box_dim, box_center)

    return poses
