import numpy as np
import modern_robotics as mr


def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt):
    global integxerr

    xdinv = mr.TransInv(Xd)

    Vd = (1 / dt) * mr.se3ToVec(mr.MatrixLog6(xdinv @ Xdnext))

    xinv = mr.TransInv(X)

    firs = mr.Adjoint(xinv @ Xd)@ Vd

    xerrmat = mr.MatrixLog6(xinv @ Xd)

    xerr = mr.se3ToVec(xerrmat)

    secon = Kp @ xerr

    integxerr += xerr * dt

    final = firs + secon + Ki @ integxerr

    return final, xerr


# testing the function

r = 0.0475
l = 0.47 / 2
w = 0.15
Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2167, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T
M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])
Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])
F = r / 4 * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                      [1, 1, 1, 1],
                      [-1, 1, -1, 1]])
F6 = np.concatenate((np.zeros((2, np.shape(F)[0] + 1)), F, np.zeros((1, np.shape(F)[0] + 1))))

config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
Xd = np.array([[0, 0, 1, 0.5],
               [0, 1, 0, 0],
               [-1, 0, 0, 0.5],
               [0, 0, 0, 1]])
Xd_next = np.array([[0, 0, 1, 0.6],
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0.3],
                    [0, 0, 0, 1]])
X = np.array([[0.170, 0, 0.985, 0.387],
              [0, 1, 0, 0],
              [-0.985, 0, 0.170, 0.570],
              [0, 0, 0, 1]])

Kp = np.identity(6)
Ki = np.zeros((6, 6))
dt = 0.01
integxerr = 0

Jacarm = mr.JacobianBody(Blist, config[3:])

T0e = mr.FKinBody(M0e, Blist, config[3:])
Tsb = np.array([[np.cos(config[0]), -np.sin(config[0]), 0, config[1]],
                [np.sin(config[0]), np.cos(config[0]), 0, config[2]],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]])
Tbe = Tb0 @ T0e
Tse = Tsb @ Tbe

Jacbase = mr.Adjoint(mr.TransInv(Tbe)) @ F6


Je = np.concatenate((Jacbase, Jacarm), axis=1)
Je_inv = np.linalg.pinv(Je)

# print(Je)


V , xerr = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)

vcon= Je_inv @ V

print(vcon)
print(xerr)