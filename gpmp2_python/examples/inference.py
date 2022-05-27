import numpy as np
import time
import matplotlib.patches as patches
from copy import deepcopy

from gtsam import *
from gpmp2 import *

from utility import getLiDAR, logging, signedDistanceField2D

def inference(previous_trajectory, map_config, noise_config, trajectory_config, 
  optimization_config, figure_current, figure_robot, axis_current, axis_robot,
  dataset, dataset_ground_truth, time_step, time_sec, pr_model):
  start_position = previous_trajectory.position_array[0]
  start_velocity = previous_trajectory.velocity_array[0]
  logging("inference", "start position", start_position)
  logging("inference", "start velocity", start_velocity)

  dataset.map = getLiDAR(
    start_position[0] - map_config.origin_x, start_position[1] - map_config.origin_y,
    dataset.map, dataset_ground_truth.map, cell_size=map_config.cell_size, 
    lidar_range=10.0)

  origin_point2 = Point2(map_config.origin_x, map_config.origin_x)
  field = signedDistanceField2D(dataset.map, map_config.cell_size)
  sdf = PlanarSDF(origin_point2, map_config.cell_size, field)

  delta_t = time_sec / time_step
  
  check_inter = int(trajectory_config.total_check_step / time_step - 1)
  
  graph = NonlinearFactorGraph()
  init_values = Values()
  for i in range(0, time_step + 1):
    key_pos = symbol(ord("x"), i)
    key_vel = symbol(ord("v"), i)
    position_at_i = previous_trajectory.position_array[i]
    velocity_at_i = previous_trajectory.velocity_array[i]
    init_values.insert(key_pos, position_at_i)
    init_values.insert(key_vel, velocity_at_i)

    if i == 0:
      graph.push_back(
        PriorFactorVector(key_pos, start_position, noise_config.pose_fix))
      graph.push_back(
        PriorFactorVector(key_vel, start_velocity, noise_config.vel_fix))
    elif i == time_step:
      graph.push_back(
        PriorFactorVector(key_pos, trajectory_config.end_position,
          noise_config.pose_fix))
      graph.push_back(
        PriorFactorVector(key_vel, trajectory_config.end_velocity,
          noise_config.vel_fix))

    graph.push_back(
      ObstaclePlanarSDFFactorPointRobot(
        key_pos, pr_model, sdf, noise_config.obstacle_sigma, noise_config.epsilon_dist))

    if i > 0:
      key_pos1 = symbol(ord("x"), i - 1)
      key_pos2 = symbol(ord("x"), i)
      key_vel1 = symbol(ord("v"), i - 1)
      key_vel2 = symbol(ord("v"), i)

      temp = GaussianProcessPriorLinear(
        key_pos1, key_vel1, key_pos2, key_vel2,
        trajectory_config.dt, noise_config.Qc_model)
      graph.push_back(temp)

      graph.push_back(
        ObstaclePlanarSDFFactorPointRobot(
          key_pos, pr_model, sdf, noise_config.obstacle_sigma, noise_config.epsilon_dist))

      if optimization_config.use_GP_inter and check_inter > 0:
        for j in range(1, check_inter + 1):
          tau = j * (time_sec / trajectory_config.total_check_step)
          graph.add(
            ObstaclePlanarSDFFactorGPPointRobot(
              key_pos1, key_vel1, key_pos2, key_vel2, pr_model,
              sdf, noise_config.obstacle_sigma, noise_config.epsilon_dist,
              noise_config.Qc_model, delta_t, tau))

  if optimization_config.use_trustregion_opt:
      parameters = DoglegParams()
      optimizer = DoglegOptimizer(graph, init_values, parameters)
  else:
      parameters = GaussNewtonParams()
      optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

  initial_error = graph.error(init_values)

  current_time = time.time()
  optimizer.optimizeSafely()
  result = optimizer.values()

  logging("inference", "initial error", initial_error)
  logging("inference", "final error", graph.error(result))
  logging("infernece", "optimization time", time.time() - current_time)

  pause_time = time_sec / time_step

  trajectory = []

  for i in range(time_step + 1):
      optimized_position = result.atVector(symbol(ord("x"), i))
      optimized_velocity = result.atVector(symbol(ord("v"), i))
      trajectory.append(
          [optimized_position[0], optimized_position[1],
          optimized_velocity[0], optimized_velocity[1]])

  return [trajectory, field]