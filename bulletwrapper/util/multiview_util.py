import numpy as np

def to_4x4(R, t):
    """
    Convert R|t representation of transformations to 4x4 numpy array.
    """
    dtype = R.dtype
    T = np.concatenate(
        [np.concatenate([R, np.expand_dims(t, -1)], 1), np.zeros((1, 4))], 0)
    T[-1, -1] = 1.
    return T.astype(dtype)
