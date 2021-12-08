import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    # t_func = [0]
    # path = np.array(path)
    # for i in range(1, len(path)):
    #     t_func.append(t_func[i-1] + np.linalg.norm(path[i] - path[i-1]) / V_des)
    # t_func = np.array(t_func)
    # splx = scipy.interpolate.splrep(t_func, path[:,0], s = alpha)
    # sply = scipy.interpolate.splrep(t_func, path[:,1], s = alpha)

    # t_smoothed = np.linspace(t_func[0], t_func[-1], round((t_func[-1] - t_func[0] + dt)/dt))
    # x_d = scipy.interpolate.splev(t_smoothed, splx, der=0)
    # y_d = scipy.interpolate.splev(t_smoothed, sply, der=0)
    # theta_d = np.arctan2(y_d, x_d)
    # xd_d = scipy.interpolate.splev(t_smoothed, splx, der=1)
    # yd_d = scipy.interpolate.splev(t_smoothed, sply, der=1)

    # xdd_d = scipy.interpolate.splev(t_smoothed, splx, der=2)
    # ydd_d = scipy.interpolate.splev(t_smoothed, sply, der=2)

    # traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    nominal_time = np.zeros(len(path))
    path_x = np.zeros(len(path))
    path_y = np.zeros(len(path))
    path_x[0] = path[0][0]
    path_y[0] = path[0][1]
    for index in range(1, len(path)):
        nominal_time[index] = nominal_time[index-1] + np.sqrt((path[index][0] - path[index-1][0])**2+(path[index][1] - path[index-1][1])**2)/V_des
        path_x[index] = path[index][0]
        path_y[index] = path[index][1]
    t_smoothed = np.arange(0, nominal_time[len(nominal_time)-1], dt)
    #print(nominal_time)

    tck_x = scipy.interpolate.splrep(nominal_time, path_x, s= alpha)
    #print(tck_x)
    x_d = scipy.interpolate.splev(t_smoothed, tck_x, der=0)
    xd_d = scipy.interpolate.splev(t_smoothed, tck_x, der=1)
    xdd_d = scipy.interpolate.splev(t_smoothed, tck_x, der=2)
    tck_y = scipy.interpolate.splrep(nominal_time, path_y, s= alpha)
    y_d = scipy.interpolate.splev(t_smoothed, tck_y, der=0)
    yd_d = scipy.interpolate.splev(t_smoothed, tck_y, der=1)
    ydd_d = scipy.interpolate.splev(t_smoothed, tck_y, der=2)
    theta_d = np.arctan2(yd_d, xd_d)


    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()


    

    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
