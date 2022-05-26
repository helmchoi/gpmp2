import numpy as np
from gtsam import *
from gpmp2 import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches


from gpmp2_python.utils.plot_utils import *
from gpmp2_python.utils.signedDistanceField2D import signedDistanceField2D

from utility import generate2Ddataset, getLiDAR

import time

# groundtruth map
cols = 400
rows = 300
origin_x = -20
origin_y = -10
cell_size = 0.1
dataset_ground_truth = generate2Ddataset(
    "GroundTruth", cols, rows, cell_size=cell_size)

# observed map
dataset = generate2Ddataset("Empty", cols, rows, cell_size=cell_size)
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)

# point robot model
pR = PointRobot(2, 1)
spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
nr_body = spheres_data.shape[0]
sphere_vec = BodySphereVector()
sphere_vec.push_back(
    BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
)
pR_model = PointRobotModel(pR, sphere_vec)

# GP
Qc = np.identity(2)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

# Obstacle avoid settings
cost_sigma = 0.5
epsilon_dist = 4.0

# prior to start/goal
pose_fix = noiseModel_Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel_Isotropic.Sigma(2, 0.0001)
initial_fix = noiseModel_Isotropic.Sigma(2, 0.00000000001)


# start and end conf
start_conf = np.asarray([0, 0])
start_vel = np.asarray([0, 0])
end_conf = np.asarray([17, 14])
end_vel = np.asarray([0, 0])

# Signed Distance field
field = signedDistanceField2D(dataset.map, cell_size)
sdf = PlanarSDF(origin_point2, cell_size, field)

figure1 = plt.figure(0)
axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
plotSignedDistanceField2D(
    figure1, axis1, field, dataset.origin_x, dataset.origin_y, dataset.cell_size
)


# settings
total_time_sec = 10.0
total_time_step = 20
total_check_step = 50.0
delta_t = total_time_sec / total_time_step
check_inter = int(total_check_step / total_time_step - 1)

use_GP_inter = True

def inference(delay, conf_val_pre, vel_pre, start_vel_i, total_time_sec,
              total_time_step, total_check_step, figure_current, figure_robot,
              axis_current, axis_robot, use_trustregion_opt=True):
    # make start conf
    start_conf_val_i = conf_val_pre[0]
    start_conf_i = (start_conf_val_i)
    print "start conf val i: ", start_conf_val_i
    print "start vel i: ", start_vel_i

    ### Update map from LiDAR (this is finite resolution LiDAR)
    dataset.map = getLiDAR(start_conf_val_i[0] - origin_x,
                            start_conf_val_i[1] - origin_y,
                            dataset.map,
                            dataset_ground_truth.map,
                            cell_size=cell_size,
                            lidar_range=10.0)

    # plot get LiDARor fast computing, just remove these
    obs = np.where(dataset.map == 1)
    rect = [(obs[0][i], obs[1][i], 1, 1) for i in range(len(obs[0]))]
    for (x, y, w, h) in rect:
        axis_current.add_patch(
            patches.Rectangle((y, x), w, h, edgecolor='green', facecolor='palegreen',
                              linewidth=0.5, fill=True))
    axis_current.set_xlim(0, cols)
    axis_current.set_ylim(0, rows)

    # Signed Distance field
    field = signedDistanceField2D(dataset.map, cell_size)
    sdf = PlanarSDF(origin_point2, cell_size, field)

    # figure1 = plt.figure(0)
    # axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
    # plotSignedDistanceField2D(figure1, axis1, field, dataset.origin_x,
    #                           dataset.origin_y, dataset.cell_size)

    # time & interpolation parameters
    delta_t = total_time_sec / total_time_step
    check_inter = int(total_check_step / total_time_step - 1)

    # Factor graph generation ---------------------------------------------------------

    # initialize optimization
    graph = NonlinearFactorGraph()
    init_values = Values()

    # make factor graph (pose initialized with previously optimized results; if None, init. with linear traj.)
    for i in range(0, total_time_step + 1):

        key_pos = symbol(ord("x"), i)
        key_vel = symbol(ord("v"), i)

        # initialize as previous configuration optimized result
        pose = (conf_val_pre[i])
        vel = vel_pre[i]
        init_values.insert(key_pos, pose)
        init_values.insert(key_vel, vel)

        # start/end priors
        if i == 0:
            ### start_conf changed at each step
            graph.push_back(
                PriorFactorVector(key_pos, start_conf_i, initial_fix))
            graph.push_back(
                PriorFactorVector(key_vel, start_vel_i, initial_fix))
        elif i == total_time_step:
            ### end_conf & end_vel can be globally defined
            graph.push_back(
                PriorFactorVector(key_pos, end_conf, pose_fix))
            graph.push_back(
                PriorFactorVector(key_vel, end_vel, vel_fix))

        graph.push_back(
            ObstaclePlanarSDFFactorPointRobot(
                key_pos, pR_model, sdf, cost_sigma, epsilon_dist
            )
        )

        # GP priors and cost factor
        if i > 0:
            key_pos1 = symbol(ord("x"), i - 1)
            key_pos2 = symbol(ord("x"), i)
            key_vel1 = symbol(ord("v"), i - 1)
            key_vel2 = symbol(ord("v"), i)

            temp = GaussianProcessPriorLinear(
                key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model)
            graph.push_back(temp)

            #% cost factor
            graph.push_back(
                ObstaclePlanarSDFFactorPointRobot(
                    key_pos, pR_model, sdf, cost_sigma, epsilon_dist
                )
            )
            # GP cost factor
            if use_GP_inter and check_inter > 0:
                for j in range(1, check_inter + 1):
                    tau = j * (total_time_sec / total_check_step)
                    graph.add(
                        ObstaclePlanarSDFFactorGPPointRobot(
                            key_pos1,
                            key_vel1,
                            key_pos2,
                            key_vel2,
                            pR_model,
                            sdf,
                            cost_sigma,
                            epsilon_dist,
                            Qc_model,
                            delta_t,
                            tau,
                        ))

    if use_trustregion_opt:
        parameters = DoglegParams()
        # parameters.setVerbosity('ERROR')
        optimizer = DoglegOptimizer(graph, init_values, parameters)
    else:
        parameters = GaussNewtonParams()
        # parameters.setRelativeErrorTol(1e-5)
        # parameters.setMaxIterations(100)
        # parameters.setVerbosity('ERROR')
        optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

    print "Initial Error \t= ", graph.error(init_values)

    # Graph Optimization --------------------------------------------------------------
    optimizer.optimizeSafely()
    result = optimizer.values()

    print "Final Error \t= ", graph.error(result)

    # Visualization -------------------------------------------------------------------
    # plot param
    pause_time = total_time_sec / total_time_step

    conf_results = []

    for i in range(total_time_step + 1):
        conf = result.atVector(symbol(ord("x"), i))
        conf_vel = result.atVector(symbol(ord("v"), i))
        conf_results.append(
            [conf[0], conf[1], conf_vel[0], conf_vel[1]])

    traj_x = []
    traj_y = []
    for i in conf_results:
        traj_x.append(i[0])
    for i in conf_results:
        traj_y.append(i[1])
    axis_robot.plot(traj_x, traj_y)

    return conf_results

if __name__ == '__main__':

    use_trustregion_opt = True

    # parameters
    delay = 4
    total_time_sec = 10.0
    total_time_step = 100
    dt = total_time_sec / total_time_step
    total_check_step = 50.0

    # initialize as straight line in conf space
    start_vel_t = start_vel

    time_sec = total_time_sec
    time_step = total_time_step

    conf_val_pre = np.empty((total_time_step + 1, 2))
    vel_pre = np.empty((total_time_step + 1, 2))
    avg_vel = (end_conf - start_conf) / total_time_step / dt
    for i in range(0, total_time_step + 1):
        conf_val_pre[
            i, :] = start_conf * float(total_time_step - i) / float(
                total_time_step) + end_conf * i / float(total_time_step)
        vel_pre[i, :] = avg_vel

    figure_cur = plt.figure(1)
    axis1 = figure_cur.gca()
    figure_r = plt.figure(2)
    axis2 = figure_r.gca()

    plotEvidenceMap2D(
        figure_r, axis2, dataset_ground_truth.map, dataset_ground_truth.origin_x,
        dataset_ground_truth.origin_y, cell_size)

    while time_step > 0:
        print("current time step is : ", total_time_step - time_step)
        conf_t = inference(delay=delay,
                            conf_val_pre=conf_val_pre,
                            vel_pre=vel_pre,
                            start_vel_i=start_vel_t,
                            total_time_sec=time_sec,
                            total_time_step=time_step,
                            total_check_step=total_check_step,
                            figure_current=figure_cur,
                            figure_robot=figure_r,
                            axis_current=axis1,
                            axis_robot=axis2,
                            use_trustregion_opt=use_trustregion_opt)

        conf_val_pre = np.array(conf_t)[delay:, 0:2]
        vel_pre = np.array(conf_t)[delay:, 2:4]
        ### TODO: this does not seem to be correct - want to assign velocities of the optimized result
        start_vel_t = vel_pre[0, :]

        for i in range(delay):
            plotPointRobot2D(figure_r, axis2, pR_model, conf_t[i])
            plt.pause(dt)

        ### update time params
        time_sec = time_sec - dt * delay
        time_step = time_step - delay

    plt.show()
