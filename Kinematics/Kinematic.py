import math
import numpy as np

def Inverse_Kinematics(Px, Py, Pz):
    L1 = 37.3
    L2 = 20
    L3 = 44.5

    th1 = -math.atan2(Py, Px);

    r = math.sqrt(Px ^ 2 + Py ^ 2);
    k = math.sqrt(r ^ 2 + (Pz - L1) ^ 2);

    th3 = -math.acos((k ^ 2 - L2 ^ 2 - L3 ^ 2) / (2 * L2 * L3));

    beta = math.atan2(Pz - L1, r);
    alpha = math.atan2(L3 * math.sin(th3), L2 + L3 * math.cos(th3));
    th2 = (beta - alpha);

    th1 = abs(np.rad2deg(th1));
    th2 = abs(np.rad2deg(th2) - 90)
    th3 = abs(np.rad2deg(th3))

    th = [th1, th2, th3]
    return th