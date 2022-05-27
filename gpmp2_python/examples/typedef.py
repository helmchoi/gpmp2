import numpy as np
from gtsam import *
from gpmp2 import *

class Obstacle:
    def __init__(self):
        self.x = None
        self.y = None
        self.v_x = None
        self.v_y = None
        self.a_x = None
        self.a_y = None
        self.size = None

class Dataset:
    def __init__(self):
        self.cols = None
        self.rows = None
        self.origin_x = None
        self.origin_y = None
        self.cell_size = None
        self.map = None

class MapConfig:
  def __init__(self, cols, rows, origin_x, origin_y, cell_size):
    self.cols = cols
    self.rows = rows
    self.origin_x = origin_x
    self.origin_y = origin_y
    self.cell_size = cell_size

class NoiseConfig:
  def __init__(self, QC, obstacle_sigma, prior_sigma, initial_sigma, epsilon_dist):
    self.Qc_model = noiseModel_Gaussian.Covariance(QC)
    self.obstacle_sigma = obstacle_sigma # cost_sigma
    self.epsilon_dist = epsilon_dist
    self.pose_fix = noiseModel_Isotropic.Sigma(2, prior_sigma)
    self.vel_fix = noiseModel_Isotropic.Sigma(2, prior_sigma)
    self.initial_fix = noiseModel_Isotropic.Sigma(2, initial_sigma)

class TrajectoryConfig:
  def __init__(self, start_position, end_position, delay, 
    total_time_second, total_time_step, total_check_step, 
    start_velocity=np.asarray([0, 0]), end_velocity=np.asarray([0, 0])):
    self.start_position = start_position
    self.start_velocity = start_velocity
    self.end_position = end_position
    self.end_velocity = end_velocity
    self.delay = delay
    self.total_time_second = total_time_second
    self.total_time_step = total_time_step
    self.total_check_step = total_check_step
    self.check_inter = int(total_check_step / total_time_step - 1)
    self.dt = total_time_second / float(total_time_step)

class OptimizationConfig:
  def __init__(self, use_GP_inter, use_trustregion_opt):
    self.use_GP_inter = use_GP_inter
    self.use_trustregion_opt = use_trustregion_opt

class TrajectoryType:
  def __init__(self, position_array, velocity_array):
    self.position_array = position_array
    self.velocity_array = velocity_array
