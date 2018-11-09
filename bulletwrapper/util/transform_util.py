import numpy as np

def col(vec):
    if vec.ndim == 1:
        return np.expand_dims(vec, -1)
    else:
        assert vec.ndim == 2
        assert vec.shape[1] == 1
        return vec

def normalize(vec):
    assert vec.ndim in [1, 2]
    if vec.ndim == 2:
        assert vec.shape[0] == 1 or vec.shape[1] == 1
    return vec / np.linalg.norm(vec)

def to_4x4(R,t):
    T = np.concatenate( [np.concatenate([R, col(t)], 1), np.zeros( (1,4) )], 0 )
    T[-1,-1] = 1.
    return T

def chain_Rts( *Rts ):

    """
    Left-multiply the transformation, from first to last Rt argument.
    """

    T = np.eye(4)
    for R,t in Rts:
        _T = to_4x4(R, t)
        T = np.dot(_T, T)

    return T[:3,:3], T[:3,-1]
    
def inv_Rt(R, t):
    R2 = R.T
    t2 = -np.dot(R2, t)
    return R2, t2
