import numpy as np
from bulletwrapper.util.transform_util import normalize, to_4x4
from bulletwrapper.util.transform_util import chain_Rts as chain
from bulletwrapper.util.transform_util import inv_Rt as inv


def pose_from_lookat(target, camera_location, camera_up='up'):
    '''
    Compute world-to-cam pose, given 
        1) 3d position of camera,
        2) its look-at point,
            1) and 2) are in world coordinate.
        3) its up-direction
            what direction should appear in the upper region in the image.

    This uses standard pinhole camera coordinate:
        https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    '''

    if camera_up == 'up':
        up = [0,0,  1]
    elif camera_up == 'down':
        up = [0,0, -1]

    zaxis = normalize( np.array(target) - np.array(camera_location) )
    xaxis = normalize( np.cross(zaxis, np.array(up)) )
    yaxis = np.cross(zaxis, xaxis)

    # R = np.array([xaxis, yaxis, zaxis])
    # t = -np.dot(R, np.reshape(camera_location, (3,1))).ravel()

    R = np.array([xaxis, yaxis, zaxis]).T
    t = np.array(camera_location)

    cam_to_world = (R,t)
    world_to_cam = inv(*cam_to_world)

    return world_to_cam

def ogl_viewmat(world_to_cam):
    '''
    Convert camera pose in world coordinate to OpenGL view matrix.
    Pinhole camera coordinate and OpenGL camera coordinate is different by
    180-degree rotation wrt x-axis; The effect of this is 
        1) Camera looks at -z direction.
        2) y-axis is pointing toward upper region of image.
    '''

    # rotate 180 wrt x-axis
    Rx = -np.eye(3)
    Rx[0,0] = 1.
    tx = np.zeros(3)

    view = chain( world_to_cam, (Rx,tx))
    V = to_4x4(*view)

    return V

def ogl_viewmat_from_lookat(target, camera_location, camera_up='up'):

    world_to_cam = pose_from_lookat(target, camera_location, camera_up)
    return ogl_viewmat(world_to_cam)

def ogl_projmat(K, H, W, z_near=0.1,  z_far=1000., inv=False):
    '''
    Convert camera intrinsics to OpenGL projection matrix.
    '''

    if inv:
        K_inv = K
    else:
        K_inv = np.linalg.inv(K)

    corners = np.array([
            [0, 0, 1], # (-1,-1)
            [W, 0, 1], # ( 1,-1)
            [0, H, 1], # (-1, 1)
            [W, H, 1], # ( 1, 1)
        ]).T

    rays = np.dot(K_inv, corners)

    # z-normalized rays
    rays = rays / rays[-1]

    near_plane = rays * z_near
    far_plane = rays * z_far

    frustrum = np.concatenate([near_plane, far_plane], axis=1)

    # rotate 180 degree wrt x-axis to convert to an opengl camera.
    R = -np.eye(3)
    R[0,0] = 1.
    frustrum = np.dot(R, frustrum).T

    ndc_vertices = np.array([
        (-1,  1, -1),
        ( 1,  1, -1),
        (-1, -1, -1),
        ( 1, -1, -1),

        (-1,  1,  1),
        ( 1,  1,  1),
        (-1, -1,  1),
        ( 1, -1,  1),
        ])

    def make_block(input, output):
        zeros = np.zeros(4)
        xyz1 = np.append(input, [1])

        crosses = np.outer(output, np.array([input[0], input[1], 1]))

        A = np.block([
                [ xyz1, zeros, zeros],
                [zeros,  xyz1, zeros],
                [zeros, zeros,  xyz1],
            ])
        A = np.concatenate([A, -crosses], axis=1)

        b = -input[-1]*output

        return A,b

    blocks = [make_block(_in, _out) for _in, _out in zip(frustrum, ndc_vertices)]
    A_blocks, b_blocks = list(zip(*blocks))
    A = np.concatenate(A_blocks, axis=0)
    b = np.concatenate(b_blocks)
    try:
        h = np.linalg.lstsq(A,b, rcond=None)[0]
    except:
        h = np.linalg.lstsq(A,b, rcond=-1)[0]
    H = np.insert(h, -1, -1).reshape(4,4)

    return H
