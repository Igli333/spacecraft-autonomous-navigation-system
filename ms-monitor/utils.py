import numpy as np
import numpy.linalg as la


# These helper functions were taken from Basilisks utilities.
def hillFrame(rc_N: np.ndarray, vc_N: np.ndarray) -> np.ndarray:
    """
    Compute the Hill frame DCM HN
    :param rc_N: inertial position vector
    :param vc_N: inertial velocity vector
    :return: HN: DCM that maps from the inertial frame N to the Hill (i.e. orbit) frame H
    """
    ir = rc_N / la.norm(rc_N)
    h = np.cross(rc_N, vc_N)
    ih = h / la.norm(h)
    itheta = np.cross(ih, ir)

    return np.array([ir, itheta, ih])


def MRP2C(q):
    """
    MRP2C

    	C = MRP2C(Q) returns the direction cosine
    	matrix in terms of the 3x1 MRP vector Q.
    """

    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    qm = np.linalg.norm(q)
    d1 = qm * qm
    S = 1 - d1
    d = (1 + d1) * (1 + d1)
    C = np.zeros((3, 3))
    C[0, 0] = 4 * (2 * q1 * q1 - d1) + S * S
    C[0, 1] = 8 * q1 * q2 + 4 * q3 * S
    C[0, 2] = 8 * q1 * q3 - 4 * q2 * S
    C[1, 0] = 8 * q2 * q1 - 4 * q3 * S
    C[1, 1] = 4 * (2 * q2 * q2 - d1) + S * S
    C[1, 2] = 8 * q2 * q3 + 4 * q1 * S
    C[2, 0] = 8 * q3 * q1 + 4 * q2 * S
    C[2, 1] = 8 * q3 * q2 - 4 * q1 * S
    C[2, 2] = 4 * (2 * q3 * q3 - d1) + S * S
    C = C / d
    return C


def C2MRP(C):
    """
    C2MRP

    	Q = C2MRP(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 3x1 MRP vector Q where the
    	MRP vector is chosen such that :math:`|Q| <= 1`.
    """

    b = C2EP(C)
    q = np.array([
        b[1] / (1 + b[0]),
        b[2] / (1 + b[0]),
        b[3] / (1 + b[0])
    ])
    return q


def C2EP(C):
    """
    C2EP
        Q = C2EP(C) translates the 3x3 direction cosine matrix
        C into the corresponding 4x1 euler parameter vector Q,
        where the first component of Q is the non-dimensional
        Euler parameter Beta_0 >= 0. Transformation is done
        using the Stanley method.
    """
    tr = np.trace(C)
    b2 = np.array([(1 + tr) / 4,
                   (1 + 2 * C[0, 0] - tr) / 4,
                   (1 + 2 * C[1, 1] - tr) / 4,
                   (1 + 2 * C[2, 2] - tr) / 4
                   ])
    case = np.argmax(b2)
    b = b2
    if case == 0:
        b[0] = np.sqrt(b2[0])
        b[1] = (C[1, 2] - C[2, 1]) / 4 / b[0]
        b[2] = (C[2, 0] - C[0, 2]) / 4 / b[0]
        b[3] = (C[0, 1] - C[1, 0]) / 4 / b[0]
    elif case == 1:
        b[1] = np.sqrt(b2[1])
        b[0] = (C[1, 2] - C[2, 1]) / 4 / b[1]
        if b[0] < 0:
            b[1] = -b[1]
            b[0] = -b[0]
        b[2] = (C[0, 1] + C[1, 0]) / 4 / b[1]
        b[3] = (C[2, 0] + C[0, 2]) / 4 / b[1]
    elif case == 2:
        b[2] = np.sqrt(b2[2])
        b[0] = (C[2, 0] - C[0, 2]) / 4 / b[2]
        if b[0] < 0:
            b[2] = -b[2]
            b[0] = -b[0]
        b[1] = (C[0, 1] + C[1, 0]) / 4 / b[2]
        b[3] = (C[1, 2] + C[2, 1]) / 4 / b[2]
    elif case == 3:
        b[3] = np.sqrt(b2[3])
        b[0] = (C[0, 1] - C[1, 0]) / 4 / b[3]
        if b[0] < 0:
            b[3] = -b[3]
            b[0] = -b[0]
        b[1] = (C[2, 0] + C[0, 2]) / 4 / b[3]
        b[2] = (C[1, 2] + C[2, 1]) / 4 / b[3]
    return b
