import numpy as np
from scipy.spatial.transform import Rotation as R


def quaternion_from_matrix(M):
    r = R.from_matrix(M)
    return r.as_quat()

def quaternion_multiply(p, q):
    vp = p[:3]
    wp = p[3]
    vq = q[:3]
    wq = q[3]

    vpq = wp * vq + wq * vp + np.cross(vp, vq)
    wpq = wp * wq - vp.dot(vq)

    return np.array([*vpq, wpq])

def quaternion_conjugate(q):
    vq = q[:3]
    wq = q[3]

    return np.array([*vq, wq])
