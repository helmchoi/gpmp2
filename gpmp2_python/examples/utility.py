import numpy as np
from gtsam import *
from gpmp2 import *
import math
from random import Random
from copy import deepcopy
import time
from scipy import ndimage

from typedef import Obstacle, Dataset, TrajectoryType

dataset_random = Random()

def logging(module, name, value, newline=False):
    if(newline):
        print("[{}] {}:".format(module, name))
        print("\t{}".format(value))
    else:
        print("[{}] {}: {}".format(module, name, value))

def random_number(min_val, max_val):
    return min_val + (max_val - min_val) * dataset_random.random()

def get_center(x, y, dataset):

    center = (np.asarray([y - dataset.origin_y, x - dataset.origin_x]) /
              dataset.cell_size)
    return center.astype(int)

def get_dim(w, h, dataset):

    return np.asarray([h, w]) / dataset.cell_size

def add_obstacle(position, size, map, landmarks=None, origin_x=None,
                 origin_y=None, cell_size=None):

    half_size_row = int(math.floor((size[0] - 1) / 2))
    half_size_col = int(math.floor((size[1] - 1) / 2))

    temp = map[position[0] - half_size_row - 1:position[0] + half_size_row,
               position[1] - half_size_col - 1:position[1] + half_size_col, ]

    map[position[0] - half_size_row - 1:position[0] + half_size_row,
        position[1] - half_size_col - 1:position[1] +
        half_size_col, ] = np.ones(temp.shape)

    # landmarks <-- what is this for?
    if landmarks is not None and origin_x is not None and origin_y is not None:
        raise NotImplementedError

    return map

def generate2Ddataset(dataset_str, map_config):
    dataset = Dataset()

    if dataset_str is "Empty":
        dataset.cols = map_config.cols
        dataset.rows = map_config.rows
        dataset.origin_x = map_config.origin_x
        dataset.origin_y = map_config.origin_y
        dataset.cell_size = map_config.cell_size
        dataset.map = 0.5 * np.ones((dataset.rows, dataset.cols))

    elif dataset_str is "GroundTruth":
        dataset.cols = map_config.cols
        dataset.rows = map_config.rows
        dataset.origin_x = map_config.origin_x
        dataset.origin_y = map_config.origin_y
        dataset.cell_size = map_config.cell_size
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # scenario 1
        dataset.map = add_obstacle(get_center(12, 10, dataset),
                                   get_dim(5, 7, dataset), dataset.map)
        dataset.map = add_obstacle(get_center(-7, 10, dataset),
                                   get_dim(10, 7, dataset), dataset.map)
        dataset.map = add_obstacle(get_center(0, -5, dataset),
                                   get_dim(10, 5, dataset), dataset.map)
        # scenario 2
        # dataset.map = add_obstacle(get_center(8, 10, dataset),
        #                            get_dim(15, 5, dataset), dataset.map)
        # dataset.map = add_obstacle(get_center(-10, 2, dataset),
        #                            get_dim(20, 5, dataset), dataset.map)
        # scenario 3
        # dataset.map = add_obstacle(get_center(7, 4, dataset),
        #                            get_dim(5, 23, dataset), dataset.map)
        # dataset.map = add_obstacle(get_center(-7, 10, dataset),
        #                            get_dim(10, 7, dataset), dataset.map)
        # dataset.map = add_obstacle(get_center(12, 19, dataset),
        #                            get_dim(8, 4, dataset), dataset.map)
        # dataset.map = add_obstacle(get_center(0, -5, dataset),
        #                            get_dim(10, 5, dataset), dataset.map)
    else:
        raise NameError("No such dataset exist")
    return dataset

def getLiDAR(robot_x, robot_y, current_map, gt_map, cell_size=0.01,
        lidar_range=1.0):
    updated_map = deepcopy(current_map)
    n = 100
    dtheta = 2.0 * np.pi / n 
    stride = 0.09
    n_d = np.ceil(lidar_range / stride).astype(np.uint)
    current_time = time.time()
    for i in range(n):
        theta_k = dtheta * i
        xy = np.array([robot_y, robot_x])
        occluded = False
        count = 0
        for j in range(n_d):
            xy += stride * np.array([-np.sin(theta_k), np.cos(theta_k)])
            cell_idx = np.floor(xy / cell_size).astype(np.uint)
            if (cell_idx[0] < 0 or cell_idx[0] >= np.shape(gt_map)[0]
                or cell_idx[1] < 0 or cell_idx[1] >= np.shape(gt_map)[1]):
                break  

            if gt_map[cell_idx[0], cell_idx[1]] == 1:
                updated_map[cell_idx[0], cell_idx[1]] = 1
                count += 1
                if (count > 3):
                    break
            else :
                updated_map[cell_idx[0], cell_idx[1]] = 0
    logging("getLiDAR", "sensor processing time", time.time() - current_time)

    return updated_map

def _linear_interpolation(a, b, i, total):
    return a * float(total - i) / total + b * i / float(total)

def generatePriorTrajectory(trajectory_config):
    tc = trajectory_config
    position_array = np.empty((tc.total_time_step + 1, 2))
    velocity_array = np.empty((tc.total_time_step + 1, 2))
    average_velocity = (tc.end_position - tc.start_position) / tc.total_time_step / tc.dt
    for i in range(0, tc.total_time_step + 1):
        position_array[i, :] = _linear_interpolation(
            tc.start_position, tc.end_position, i, tc.total_time_step)
        velocity_array[i, :] = average_velocity
    return TrajectoryType(position_array, velocity_array)

def generatePrModel():
    pR = PointRobot(2, 1)
    spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
    nr_body = spheres_data.shape[0]
    sphere_vec = BodySphereVector()
    sphere_vec.push_back(
        BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4])))
    return PointRobotModel(pR, sphere_vec)

def signedDistanceField2D(ground_truth_map, cell_size):
    logging("signedDisntancdField2D",
        "input size", ground_truth_map.shape)

    cur_map = ground_truth_map > 0.75
    cur_map = cur_map.astype(int)

    if np.amax(cur_map) is 0:
        return np.ones(ground_truth_map.shape) * 1000

    inv_map = 1 - cur_map

    map_dist = ndimage.distance_transform_edt(inv_map)
    inv_map_dist = ndimage.distance_transform_edt(cur_map)

    field = map_dist - inv_map_dist

    field = field * cell_size
    field = field.astype(float)
    logging("signedDisntancdField2D",
        "output size", field.shape)

    return field

if __name__ == "__main__":
    current_map = np.zeros((5, 5))
    # current_map[3,2] = 1
    gt_map = np.random.randn(5, 5)
    gt_map = gt_map / max(gt_map.flatten())
    gt_map[abs(gt_map) > 0.5] = 1
    gt_map[abs(gt_map) <= 0.5] = 0
    # gt_map = np.array([[1., 0., 0., 0., 0.],
    #                     [0., 0., 1., 1., 0.],
    #                     [1., 0., 1., 0., 1.],
    #                     [1., 0., 0., 1., 0.],
    #                     [0., 1., 0., 0., 0.]])

    print "gt_map: \n", gt_map
    print "current_map: \n", current_map

    robot_x = 2.4
    robot_y = 3.7

    updated_map = getLiDAR(robot_x, robot_y, current_map, gt_map, 1.0, 3.0)
    print "updated map: \n", updated_map
