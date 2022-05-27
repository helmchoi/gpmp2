import numpy as np
import time
import matplotlib.pyplot as plt

from gtsam import *
from gpmp2 import *

from utility import generate2Ddataset, generatePriorTrajectory, generatePrModel, logging
from plot import plotTrajectory, plot2DMap, plotRobot, plotSignedDistanceField2D
from typedef import MapConfig, NoiseConfig, TrajectoryConfig, OptimizationConfig, TrajectoryType
from inference import inference

if __name__ == '__main__':
    map_config = MapConfig(
        cols=100, rows=75, origin_x=-20, origin_y=-10, cell_size=0.4)
    noise_config = NoiseConfig(
        QC=np.identity(2), obstacle_sigma=0.08, prior_sigma=0.001,
        initial_sigma=0.0000001, epsilon_dist=3.0)
    trajectory_config = TrajectoryConfig(
        start_position=np.asarray([0, 0]), end_position=np.asarray([17, 14]), delay=4,
        total_time_second=10.0, total_time_step=100, total_check_step=50.0)
    optimization_config = OptimizationConfig(use_GP_inter=True, use_trustregion_opt=True)

    dataset_ground_truth = generate2Ddataset("GroundTruth", map_config)
    dataset = generate2Ddataset("Empty", map_config)
    
    pr_model = generatePrModel()

    figure_current = plt.figure(1)
    axis_current = figure_current.gca()
    figure_robot = plt.figure(2)
    axis_robot = figure_robot.gca()

        
    previous_trajectory = generatePriorTrajectory(trajectory_config)

    time_sec = trajectory_config.total_time_second
    time_step = trajectory_config.total_time_step

    while time_step > 0:
        logging("simulation loop", "current time step",
            trajectory_config.total_time_step - time_step)

        result = inference(
            previous_trajectory, map_config, noise_config, trajectory_config, 
            optimization_config, figure_current, figure_robot, axis_current, 
            axis_robot, dataset, dataset_ground_truth, time_step, time_sec, pr_model)
        trajectory = result[0]
        field = result[1]

        axis_current.cla()
        axis_robot.cla()
        plot2DMap(
            figure_robot, axis_robot,
            dataset_ground_truth.map, dataset_ground_truth.origin_x,
            dataset_ground_truth.origin_y, map_config.cell_size)
        plot2DMap(
            figure_current, axis_current,
            dataset.map, dataset.origin_x,
            dataset.origin_y, map_config.cell_size)
        plotTrajectory(axis_robot, trajectory)
        plotSignedDistanceField2D(
            figure_current, axis_current, field, map_config.origin_x,
            map_config.origin_y, map_config.cell_size, epsilon_dist=noise_config.epsilon_dist)
        for i in range(trajectory_config.delay):
            plotRobot(figure_robot, axis_robot, pr_model, trajectory[i][:])
            plotRobot(figure_robot, axis_current, pr_model, trajectory[i][:])
        plt.pause(0.0001)

        previous_trajectory = TrajectoryType(
            np.array(trajectory)[trajectory_config.delay:, 0:2],
            np.array(trajectory)[trajectory_config.delay:, 2:4])
        time_sec = time_sec - trajectory_config.dt * trajectory_config.delay
        time_step = time_step - trajectory_config.delay
    plt.show()
