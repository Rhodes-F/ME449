import numpy as np
import modern_robotics as mr


def F_from_spring(MList, Slist, thetalist, stiffness, springPos, restLength):
    M = np.matmul(
        np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(MList[0], MList[1]), MList[2]), MList[3]), MList[4]),
                  MList[5]), MList[6])

    config = mr.FKinSpace(M, Slist, thetalist)

    dist = ((springPos[0] - config[0][3]) ** 2 + (springPos[1] - config[1][3]) ** 2 + (
            springPos[2] - config[2][3]) ** 2) ** .5
    Fscal = (dist - restLength) * stiffness

    xdist = -(springPos[0] - config[0][3])
    ydist = -(springPos[1] - config[1][3])
    zdist = -(springPos[2] - config[2][3])

    Fs = np.array([xdist / dist, ydist / dist,
                   zdist / dist]) * Fscal
    Fb = Fs @ config[:3, :3]

    Ftip = np.array([0, 0, 0, Fb[0], Fb[1], Fb[2]])

    return Ftip


# noinspection PyTypeChecker
def puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt, damping, stiffness, springPos, restLength):
    taulist = [0, 0, 0, 0, 0, 0]
    Ftip = [0, 0, 0, 0, 0, 0]
    thetamat = [thetalist]
    dthetamat = [dthetalist]
    num = t / dt
    i = 0
    while i < num:
        Ftip = F_from_spring(Mlist, Slist, thetalist, stiffness, springPos, restLength)
        ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
        [thetalist, dthetalist] = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)
        thetamat = np.append(thetamat, np.array([thetalist]), axis=0)
        np.savetxt("thetamat.csv", thetamat, delimiter=",")
        dthetamat = np.append(dthetamat, np.array([dthetalist]), axis=0)
        taulist = -damping * dthetalist

        i += 1
    np.savetxt("thetamat.csv", thetamat, delimiter=",")
    return thetamat, dthetamat


# defining the parameters for the rest of the code
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [[0, 0, 0, 0, 0, 0],
         [0, 1, 1, 1, 0, 1],
         [1, 0, 0, 0, -1, 0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0, 0, 0, 0, 0.81725, 0],
         [0, 0, 0.425, 0.81725, 0, 0.81725]]




### Part 1a good dt
# puppet(thetalist=[0, 0, 0, 0, 0, 0], dthetalist=[0, 0, 0, 0, 0, 0], g=(0, 0, -9.8), Mlist=Mlist, Slist=Slist,
#        Glist=Glist, t=5, dt=.0025, damping=0, stiffness=0, springPos=[0, 0, 0],
#        restLength=0)
#
# ## Part 1b bad dt
# puppet(thetalist=[0, 0, 0, 0, 0, 0], dthetalist=[0, 0, 0, 0, 0, 0], g=(0, 0, -9.8), Mlist=Mlist, Slist=Slist,
#        Glist=Glist, t=5, dt=.01, damping=0, stiffness=0, springPos=[0, 0, 0],
#        restLength=0)
#
# ## Part 2 decreasing energy
# puppet(thetalist=[0, 0, 0, 0, 0, 0], dthetalist=[0, 0, 0, 0, 0, 0], g=(0, 0, -9.8), Mlist=Mlist, Slist=Slist,
#        Glist=Glist, t=5, dt=.01, damping=1, stiffness=0, springPos=[0, 0, 0],
#        restLength=0)
#
# ## Part 2 increasing energy
# puppet(thetalist=[0, 0, 0, 0, 0, 0], dthetalist=[0, 0, 0, 0, 0, 0], g=(0, 0, -9.8), Mlist=Mlist, Slist=Slist,
#        Glist=Glist, t=5, dt=.01, damping=-.01, stiffness=0, springPos=[0, 0, 0],
#        restLength=0)
#
# ## Part 3 a
# puppet(thetalist=[0, -1, 0, 0, 0, 0], dthetalist=[0, 0, 0, 0, 0, 0], g=(0, 0, 0), Mlist=Mlist, Slist=Slist, Glist=Glist,
#        t=10, dt=.01, damping=0, stiffness=25, springPos=[0, 0, 1],
#        restLength=0)
#
# ## Part 3 b
# puppet(thetalist=[0, -1, 0, 0, 0, 0], dthetalist=[0, 0, 0, 0, 0, 0], g=(0, 0, 0), Mlist=Mlist, Slist=Slist, Glist=Glist,
#        t=10, dt=.01, damping=2, stiffness=15, springPos=[0, 0, 1],
#        restLength=0)
