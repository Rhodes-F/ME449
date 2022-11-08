import numpy as np
import modern_robotics as mr

#####Example inputs below


# For New task

# integxerr = 0
#
# Tcon = np.array([[0, 0, 1, 0],
#                  [0, 1, 0, 0],
#                  [-1, 0, 0, .5],
#                  [0, 0, 0, 1]])
# Tic = np.array([[0, 0, 1, 1],
#                 [0, 1, 0, 0],
#                 [-1, 0, 0, .025],
#                 [0, 0, 0, 1]])
#
# Tf = np.array([[0, 1, 0, 1.5],
#                [0, 0, -1, -.5],
#                [-1, 0, 0, .025],
#                [0, 0, 0, 1]])
#
# acon = np.array([1, 0, .3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
#
# Kp = 2*np.identity(6)
# Ki = np.identity(6)*.00011
#
# Cap(Tcon, Tic, Tf, acon, Kp, Ki)


#For Best


# integxerr = 0
#
# Tcon = np.array([[0, 0, 1, 0],
#                  [0, 1, 0, 0],
#                  [-1, 0, 0, .5],
#                  [0, 0, 0, 1]])
# Tic = np.array([[0, 0, 1, 1],
#                 [0, 1, 0, 0],
#                 [-1, 0, 0, .025],
#                 [0, 0, 0, 1]])
#
# Tf = np.array([[0, 1, 0, 0],
#                [0, 0, -1, -1],
#                [-1, 0, 0, .025],
#                [0, 0, 0, 1]])
#
# acon = np.array([1, 0, .3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
#
# Kp = 2*np.identity(6)
# Ki = np.identity(6)*.00011
#
# Cap(Tcon, Tic, Tf, acon, Kp, Ki)



# For Overshoot

# integxerr = 0
#
# Tcon = np.array([[0, 0, 1, 0],
#                  [0, 1, 0, 0],
#                  [-1, 0, 0, .5],
#                  [0, 0, 0, 1]])
# Tic = np.array([[0, 0, 1, 1],
#                 [0, 1, 0, 0],
#                 [-1, 0, 0, .025],
#                 [0, 0, 0, 1]])
#
# Tf = np.array([[0, 1, 0, 0],
#                [0, 0, -1, -1],
#                [-1, 0, 0, .025],
#                [0, 0, 0, 1]])
#
# acon = np.array([1, 0, .3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
#
# Kp = 2*np.identity(6)
# Ki = np.identity(6)*300
#
# Cap(Tcon, Tic, Tf, acon, Kp, Ki)

def TrajectoryGenerator(Tcon, Tic, Tf, Pgrasp=None, Hstand=.5, k=1):
    if Pgrasp is None:
        Pgrasp = [[0, 0, 0]]
    method = 5
    # inputs :
    # The initial configuration of the end-effector in the reference trajectory: Tcon

    # The cube's initial configuration: Tic

    # The cube's desired final configuration: Tf

    # a 3 vector position of the ee relative to the cube: Pgrasp

    # The standoff height for the robot, this will be a z distance above the cube: Hstand

    # The number of trajectory reference configurations per 0.01 seconds: k.

    # The trajectory will follow 8 steps

    # 1.	A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
    # 2.	A trajectory to move the gripper down to the grasp position.
    # 3.	Closing of the gripper.
    # 4.	A trajectory to move the gripper back up to the "standoff" configuration.
    # 5.	A trajectory to move the gripper to a "standoff" configuration above the final configuration.
    # 6.	A trajectory to move the gripper to the final configuration of the object.
    # 7.	Opening of the gripper.
    # 8.	A trajectory to move the gripper back to the "standoff" configuration.

    # each one of the steps will take the same ammount of time

    # the number of steps for each movement
    steps = (32 / 8) * k / .01

    Tstand1 = np.array([[0, 0, 1, Tic[0][3]],
                        [0, 1, 0, Tic[1][3]],
                        [-1, 0, 0, Hstand],
                        [0, 0, 0, 1]])

    # The trajectory from the initial position to the standoff
    step_a = mr.CartesianTrajectory(Tcon, Tstand1, 4, steps, method)

    moves = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    for T in step_a:
        rat = np.array(
            [T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2], T[0][3], T[1][3],
             T[2][3], 0])
        moves = np.append(moves, [rat], axis=0)

    moves = moves[1:][:]

    # The trajectory from the standoff to the position of the block
    step_b = mr.CartesianTrajectory(Tstand1, Tic, 4, steps, method)

    for T in step_b:
        rat = np.array(
            [T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2], T[0][3], T[1][3],
             T[2][3], 0])
        moves = np.append(moves, [rat], axis=0)

    # Closing the gripper
    i = 0
    while i < 100:
        last_row = [moves[-1][0], moves[-1][1], moves[-1][2], moves[-1][3], moves[-1][4],
                    moves[-1][5], moves[-1][6], moves[-1][7], moves[-1][8], moves[-1][9], moves[-1][10], moves[-1][11],
                    1]
        moves = np.append(moves, [last_row], axis=0)
        i += 1

    # The trajecory from the standoff position to the grip position in the world frame

    step_c = mr.CartesianTrajectory(Tic, Tstand1, 4, steps, method)

    for T in step_c:
        rat = np.array(
            [T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2], T[0][3], T[1][3],
             T[2][3], 1])
        moves = np.append(moves, [rat], axis=0)

    # moving over to above where I would like to put the block

    Tstand2 = np.array([[0, 1, 0, Tf[0][3]],
                        [0, 0, -1, Tf[1][3]],
                        [-1, 0, 0, Hstand],
                        [0, 0, 0, 1]])
    step_d = mr.ScrewTrajectory(Tstand1, Tstand2, 4, steps, method)

    for T in step_d:
        rat = np.array(
            [T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2], T[0][3], T[1][3],
             T[2][3], 1])
        moves = np.append(moves, [rat], axis=0)

    # move from stand position to final position
    step_e = mr.ScrewTrajectory(Tstand2, Tf, 4, steps, method)

    for T in step_e:
        rat = np.array(
            [T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2], T[0][3], T[1][3],
             T[2][3], 1])
        moves = np.append(moves, [rat], axis=0)

    # open the gripper
    i = 0
    while i < 100:
        last_row = [moves[-1][0], moves[-1][1], moves[-1][2], moves[-1][3], moves[-1][4],
                    moves[-1][5], moves[-1][6], moves[-1][7], moves[-1][8], moves[-1][9], moves[-1][10], moves[-1][11],
                    0]
        moves = np.append(moves, [last_row], axis=0)
        i += 1

    # move back to the standing position

    step_f = mr.ScrewTrajectory(Tf, Tstand2, 4, steps, method)

    for T in step_f:
        rat = np.array(
            [T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2], T[0][3], T[1][3],
             T[2][3], 0])
        moves = np.append(moves, [rat], axis=0)

    np.savetxt("moves.csv", moves, delimiter=",")
    return (moves)


def NextState(curconfig, u, dt=.01, max=10000000):
    # Inputs:
    # curconfig: the current configuration of the robot
    # u: The joint speeds of the robot
    # dt: the timestep
    # max: the maximum joint speeds
    #
    # Output:
    # newconfig: a new configuration obtiained by following the joint speeds for dt

    wheelspeeds = np.array(u[5:])
    jointspeeds = np.array(u[:5])

    for i, w in enumerate(wheelspeeds):
        if w > max:
            wheelspeeds[i] = max

    for i, j in enumerate(jointspeeds):
        if j > max:
            jointspeeds[i] = max

    l = .235
    w = .15
    r = .0475
    odom = np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                     [1, 1, 1, 1],
                     [-1, 1, -1, 1]])

    twistbod = (r / 4) * odom @ (wheelspeeds * dt)

    omeg = twistbod[0]
    vbx = twistbod[1]
    vby = twistbod[2]

    if omeg != 0:
        dqb = np.array([omeg,
                        (vbx * np.sin(omeg) + (vby * (np.cos(omeg) - 1))) / omeg,
                        (vby * np.sin(omeg) + (vbx * (-np.cos(omeg) + 1))) / omeg])
    else:
        dqb = np.array([omeg, vbx, vby])

    dq = np.array([[1, 0, 0],
                   [0, np.cos(omeg), -np.sin(omeg)],
                   [0, np.sin(omeg), np.cos(omeg)]])
    posbod = [curconfig[0], curconfig[1], curconfig[2]] + dq @ dqb

    poswheels = [curconfig[8] + wheelspeeds[0] * dt, curconfig[9] + wheelspeeds[1] * dt,
                 curconfig[10] + wheelspeeds[2] * dt, curconfig[11] + wheelspeeds[3] * dt]

    posjoint = [curconfig[3] + jointspeeds[0] * dt, curconfig[4] + jointspeeds[1] * dt,
                curconfig[5] + jointspeeds[2] * dt, curconfig[6] + jointspeeds[3] * dt,
                curconfig[7] + jointspeeds[4] * dt]

    newconfig = [posbod[0], posbod[1], posbod[2], posjoint[0], posjoint[1], posjoint[2], posjoint[3], posjoint[4],
                 poswheels[0], poswheels[1], poswheels[2], poswheels[3]]

    return newconfig


def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt=.01):
    # Inputs:
    # X:The current actual end-effector configuration X (also written Tse).
    # Xd:The current end-effector reference configuration Xd (i.e., Tse,d).
    # Xdnext:The end-effector reference configuration at the next timestep in the reference trajectory, Xd,next (i.e., Tse,d,next), at a time Δt later.
    # The PI gain matrices Kp and Ki.
    # dt:The timestep Δt between reference trajectory configurations.
    #
    # Output:
    # V: The end effector twist

    global integxerr

    xdinv = mr.TransInv(Xd)

    Vd = (1 / dt) * mr.se3ToVec(mr.MatrixLog6(xdinv @ Xdnext))

    xinv = mr.TransInv(X)

    firs = mr.Adjoint(xinv @ Xd) @ Vd

    xerrmat = mr.MatrixLog6(xinv @ Xd)

    xerr = mr.se3ToVec(xerrmat)

    secon = Kp @ xerr

    integxerr += xerr * dt

    final = firs + secon + Ki @ integxerr

    return final, xerr


def Cap(Tcon, Tic, Tf, acon, Kp, Ki):
    # inputs :

    # Te initial reference end effector configuration

    # The cube's initial configuration: Tic

    # The cube's desired final configuration: Tf

    # acon : the robots actual configuration

    # The PI gain matrices Kp and Ki.

    #output:

    # two .csv files, move.csv whcih contains the moves for copelia sim and errors.csv whcih has the error

    r = 0.0475
    l = 0.235
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

    curconfigp = np.copy([acon])

    errlist = np.empty((0,6))

    ref = TrajectoryGenerator(Tcon, Tic, Tf)

    i = 0

    while i < 2599:
        Xd = np.array([[ref[i][0], ref[i][1], ref[i][2], ref[i][9]],
                       [ref[i][3], ref[i][4], ref[i][5], ref[i][10]],
                       [ref[i][6], ref[i][7], ref[i][8], ref[i][11]],
                       [0, 0, 0, 1]])
        Xdnext = np.array([[ref[i + 1][0], ref[i + 1][1], ref[i + 1][2], ref[i + 1][9]],
                           [ref[i + 1][3], ref[i + 1][4], ref[i + 1][5], ref[i + 1][10]],
                           [ref[i + 1][6], ref[i + 1][7], ref[i + 1][8], ref[i + 1][11]],
                           [0, 0, 0, 1]])

        des = acon[:8]

        Jacarm = mr.JacobianBody(Blist, des[3:])

        T0e = mr.FKinBody(M0e, Blist, des[3:])
        Tsb = np.array([[np.cos(acon[0]), -np.sin(acon[0]), 0, acon[1]],
                        [np.sin(acon[0]), np.cos(acon[0]), 0, acon[2]],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
        Tbe = Tb0 @ T0e
        Tse = Tsb @ Tbe

        Jacbase = mr.Adjoint(mr.TransInv(Tbe)) @ F6

        Je = np.concatenate((Jacbase, Jacarm), axis=1)
        Jei = np.linalg.pinv(Je)

        Twis, xerr = FeedbackControl(Tse, Xd, Xdnext, Kp, Ki, dt=.01)

        u = Jei @ Twis

        ugood= [u[4],u[5],u[6],u[7],u[8],u[0],u[1],u[2],u[3]]

        acon = NextState(acon, ugood)

        configl = np.append(acon, ref[i][12])

        curconfigp = np.append(curconfigp, [configl] , axis= 0 )

        errlist = np.append(errlist, [xerr], axis= 0 )


        i += 1

    np.savetxt('move.csv', curconfigp, delimiter=',')
    np.savetxt('errors.csv', errlist, delimiter=',' )

    return


