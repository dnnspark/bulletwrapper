import numpy as np
from bulletwrapper.util.object_loader import OBJFile

class Block():
    '''
    This class represent a 2D rectangle associated with a top-down view of a cuboid.
    Each cuboid gives rise to 6 sides * 4 orientation = 24 axis-aligned blocks.
    The rectangle is represented by its width and height; this data is used to 
    compute the layout of blocks.
    The block also has fields of rotation matrix and z-offset, which are used to
    convert the 2D position of block to a full 3D pose of cuboid.
    '''

    def __init__(self, width, height, R, zoffset):
        self.dim = (width, height)
        self.R = R
        self.zoffset = zoffset


def cuboid_to_blocks(vertices, inplane_rot_angles=[0]):
    '''
    CRUCIAL assumption:
        vertices collectively represent an axis-aligned 3D cuboid centered at origin.

    Input
    =====
        vertices: np.ndarray
            (N,3) float tensor, each row is a vertex.

    Return
    ======
        blocks: [Block]
            A list of N*24 blocks, where N = len(inplane_rot_angles)

        # blocks: np.ndarray
        #     (N*24,2) float tensor, where N = len(inplane_rot_angles)

        # rot_mat: np.ndarray
        #     (N*24,3,3) float tensor that contains N 3x3 rotation matrices 
        #     associated with blocks.
    '''

    axes = np.array([
            [ 1, 0, 0],
            [-1, 0, 0],
            [ 0, 1, 0],
            [ 0,-1, 0],
            [ 0, 0, 1],
            [ 0, 0,-1],
        ])

    # Rs, blocks, zoffsets = [], [], []
    blocks = []
    for xaxis in axes:
        for yaxis in axes:
            if np.dot(xaxis, yaxis) != 0:
                continue
            zaxis = np.cross(xaxis, yaxis)

            R0 = np.array([xaxis, yaxis, zaxis]).T # object-to-world rotation

            for theta in inplane_rot_angles:
                _theta = np.pi * theta/180.
                c,s = np.cos(_theta), np.sin(_theta)
                R_inplane = np.array([
                    [c, -s, 0],
                    [s,  c, 0],
                    [0,  0, 1],
                    ])
                R = np.dot(R_inplane, R0)
                # R = np.dot(R0, R_inplane)

                transformed = np.dot( R, vertices.T ).T
                x1y1z1 = np.min(transformed, axis=0)
                x2y2z2 = np.max(transformed, axis=0)
                # assert np.allclose(x1y1z1 + x2y2z2 , 0) # for perfect cuboid

                diagonal = x2y2z2 - x1y1z1
                W,H = diagonal[:2]

                zoffset = -x1y1z1[-1]

                blocks.append( Block(W, H, R, zoffset) )

    return blocks

def pack_bin(blocks, box, slack=0.):
    '''
    2D bin-packing divide-and-conquer algorithm.

    Input
    =====
        blocks: [Blocks]
            (N,2) float tensor representing dimensions of 2D blocks. 
            The first column are widths, the second heights.

        box: [float, float]
            width, height

        slack: (float, float)
            range of slack

    Return
    ======
        block_layout: [ (int, (float, float)) ]
            a list of (index, (start_x, start_y))

    '''

    blocks_dim = np.array([block.dim for block in blocks])
    blocks_dim_w_slack = blocks_dim + slack

    # randomly choose a block that fits.
    fit = blocks_dim_w_slack <= np.expand_dims(box, 0)
    fit = np.where(np.logical_and(fit[:,0], fit[:,1]))[0]

    if len(fit) == 0:
        # no blocks fit.
        return []

    pick = np.random.choice(fit)
    block_dim_w_slack = blocks_dim_w_slack[pick]

    W,H = box
    w,h = block_dim_w_slack

    # randomly choose one of the two ways to split the remaining area into two rectangular areas.
    split = np.random.choice(2)
    if split == 0:
        # split horizontally.
        two_residual_boxes = [
            [W-w,  h],
            [W,  H-h], 
            ]

    else:
        # split vertically.
        two_residual_boxes = [
            [W-w,   H],
            [w,   H-h],
            ]

    # randomly choose one of the four corners to place the block.
    corner = np.random.choice(4)
    if corner == 0:
        # upper-left corner
        layout = [ (pick, (0., 0.)) ]

        two_offsets = [
            [w,  0.],
            [0., h]
        ]

    elif corner == 1:
        # upper-right corner
        layout = [ (pick, (W-w, 0.)) ]

        if split == 0:
            two_offsets = [
                [0., 0.],
                [0., h],
            ]
        else:
            two_offsets = [
                [0., 0.],
                [W-w,  h],
            ]
    elif corner == 2:
        # lower-left corner
        layout = [ (pick, (0., H-h)) ]

        if split == 0:
            two_offsets = [
                [w,  H-h],
                [0., 0.],
            ]
        else:
            two_offsets = [
                [w,  0.],
                [0., 0.],
            ]

    else: #corner == 3:
        # lower-right corner
        layout = [ (pick, (W-w, H-h)) ]

        if split == 0:
            two_offsets = [
                [0., H-h],
                [0.,  0.],
            ]
        else:
            two_offsets = [
                [0., 0.],
                [W-w, 0.],
            ]

    for residual_box, offsets in zip(two_residual_boxes, two_offsets):
        sub_layout = pack_bin(blocks, residual_box, slack)
        sub_layout = [(idx, (start_x+offsets[0], start_y+offsets[1])) for idx, (start_x, start_y) in sub_layout]
        layout += sub_layout

    return layout

def compute_poses(blocks, block_layout, box_dim, box_center):

    # positions = []
    poses = []
    box_dim, box_center = np.array(box_dim), np.array(box_center)
    for idx, (x1, y1) in block_layout:
        block = blocks[idx]

        R = block.R
        xy_offset = np.array([x1,y1]) + .5 * np.array(block.dim) - .5 * box_dim + box_center
        z_offset = block.zoffset

        position = list(xy_offset) + [z_offset]
        poses.append( (R, position))

    return poses

def make_packed_cuboids_scene(path_to_obj, mesh_scale, box_dim, box_center, inplane_rot_angles, slack, block_filter=None):
    vertices = np.array( OBJFile(path_to_obj, None).vertices ) * mesh_scale
    blocks = cuboid_to_blocks(vertices, inplane_rot_angles=inplane_rot_angles)
    if block_filter is not None:
        blocks = [block for block in blocks if block_filter(block)]
    block_layout = pack_bin(blocks, box_dim, slack=slack)

    poses = compute_poses(blocks, block_layout, box_dim, box_center)

    return poses
