import numpy as np
import functools

def sample_grid_layers(
    num_layers,
    model_vertices,
    model_weights,
    layer_size_range,
    layer_center_range,
    use_only_inplane_rotation,
    rotation_axis_range,
    rotation_angle_range,
    gap_between_layers,
):
    # Determine heights of layer from bounding boxes of models.
    dimensions = [vertices.max(axis=0) - vertices.min(axis=0) for vertices in model_vertices]

    if use_only_inplane_rotation:
        # maximal length among all axis and all models
        layer_height = np.array(dimensions).max()
    else:
        # maximal diagonal length among all models.
        layer_height = np.array([np.linalg.norm(dim) for dim in dimensions]).max()
    import pdb; pdb.set_trace()

    sample_single_layer = functools.partial(sample_grid_single_layer,
                                            model_vertices=model_vertices,
                                            model_weights=model_weights,
                                            layer_size_range=layer_size_range,
                                            layer_center_range=layer_center_range,
                                            use_only_inplane_rotation=use_only_inplane_rotation,
                                            rotation_axis_range=rotation_axis_range,
                                            rotation_angle_range=rotation_angle_range,
                                            )

    z_offset = gap_between_layers
    for layer in range(num_layers):
        poses = sample_single_layer()
        import pdb; pdb.set_trace()
        # lift it by z_offset
        z_offset += layer_height


def sample_grid_single_layer(
    model_vertices,
    model_weights,
    layer_size_range,
    layer_center_range,
    use_only_inplane_rotation,
    rotation_axis_range,
    rotation_angle_range,
):
    """
    Sample a grid configuration of objects with random orientations.

    model_vertices: [np.ndarray]
        A list of (N,3) np.ndarray
        vertcies of model
    model_weights: [float]
        frequency weights on models.
        It should be of same length of `model_vertices`, or None
    layer_size_range: ((float, float), (float, float))
        range of xy-dimension of layer
        ((x_min, y_min), (x_max, y_max))
    layer_center_range: ((float, float), (float, float))
        range of center of layer
        ((x_min, y_min), (x_max, y_max))
    use_only_inplane_rotation: bool
        if True, use only rotation in xy-plane.
        if False, use arbitrary 3D rotation
        The 3D rotation is parameterized in (axis, angle).
    rotation_axis_range: ((float, float), (float, float), (float, float))
        Range of rotation axis, a unit-norm 3D vector.
        A vector is sampled from a 3D box, and then normalized to have unit-norm.
        Ignored if use_only_inplane_rotation is True,
    rotation_angle_range: (float, float)
        Range of rotation angles, a unit-norm 3D vector.
    seed: int
        seed for np.random
    """

    if len(model_weights) == 1:
        # uniform
        model_weights = tuple(list(model_weights) * len(model_vertices))

    model_weights = np.array(model_weights)
    # normalize
    model_weights = model_weights / model_weights.sum()

    layer_size = np.random.uniform(*layer_size_range)
    layer_center = np.random.uniform(*layer_center_range)
    import pdb; pdb.set_trace()

    # model_vertices, dist_mesh_scale, dist_box_dim,
    #                           dist_box_center, inplane_rot_angles, angle_weights,
    #                           dist_slack, height_offset=0., face_filter=None):
