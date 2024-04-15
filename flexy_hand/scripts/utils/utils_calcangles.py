import numpy as np

def sgn(a):
    if a >= 0:
        return 1
    else:
        return -1

def vector2points(A, B):
    """
    Calculate the vector between 2 points => Vector AB
    """

    AB = (B[0]-A[0], B[1]-A[1], B[2]-A[2])

    return AB

def angle2vectors(u, v):
    """
    Calculate the angle between 2 vectors
    """

    # Unit vectors
    u_u = u / np.linalg.norm(u)
    v_u = v / np.linalg.norm(v)

    # Calculate angle
    #return np.arccos(np.clip(np.dot(u_u, v_u), -1.0, 1.0))
    return np.arccos(np.dot(u_u, v_u) / (np.linalg.norm(u_u) * np.linalg.norm(v_u)))

def angle3points(A, B, C):
    """
    Calculate the angle between 3 points => Angle between the vector AB and AC.
    """

    # Vectors
    AB = vector2points(A, B)
    AC = vector2points(A, C)

    return angle2vectors(AB, AC)