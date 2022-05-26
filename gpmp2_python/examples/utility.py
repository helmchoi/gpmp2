import numpy as np
from gtsam import *
from gpmp2 import *
import math
from random import Random
from copy import deepcopy
import time

dataset_random = Random()


class Obstacle:
    """docstring for Dataset"""

    def __init__(self):
        self.x = None
        self.y = None
        self.v_x = None
        self.v_y = None
        self.a_x = None
        self.a_y = None
        self.size = None


class Dataset:
    """docstring for Dataset"""

    def __init__(self):
        self.cols = None
        self.rows = None
        self.origin_x = None
        self.origin_y = None
        self.cell_size = None
        self.map = None


def random_number(min_val, max_val):
    return min_val + (max_val - min_val) * dataset_random.random()


def get_center(x, y, dataset):

    center = (np.asarray([y - dataset.origin_y, x - dataset.origin_x]) /
              dataset.cell_size)
    return center.astype(int)


def get_dim(w, h, dataset):

    return np.asarray([h, w]) / dataset.cell_size


def add_obstacle(position,
                 size,
                 map,
                 landmarks=None,
                 origin_x=None,
                 origin_y=None,
                 cell_size=None):

    half_size_row = int(math.floor((size[0] - 1) / 2))
    half_size_col = int(math.floor((size[1] - 1) / 2))

    # occupency grid. here map is assumed to be numpy array

    temp = map[position[0] - half_size_row - 1:position[0] + half_size_row,
               position[1] - half_size_col - 1:position[1] + half_size_col, ]

    map[position[0] - half_size_row - 1:position[0] + half_size_row,
        position[1] - half_size_col - 1:position[1] +
        half_size_col, ] = np.ones(temp.shape)

    # landmarks <-- what is this for?
    if landmarks is not None and origin_x is not None and origin_y is not None:

        raise NotImplementedError

    return map


## Dataset for PGM project ===============================================================
def generate2Ddataset(dataset_str,
                          cols,
                          rows,
                          origin_x=-20,
                          origin_y=-10,
                          cell_size=0.1):
    # GENERATE2DDATASET Generate 2D dataset evidence grid
    # FOR Empty initialization and GrounTruth map
    #
    #    Output Format:
    #    dataset.map        ground truth evidence grid
    #    dataset.rows       number of rows (y)
    #    dataset.cols       number of cols (x)
    #    dataset.origin_x   origin of map x
    #    dataset.origin_y   origin of map y
    #    dataset.cell_size  cell size

    dataset = Dataset()

    if dataset_str is "Empty":
        # params
        dataset.cols = cols
        dataset.rows = rows
        dataset.origin_x = origin_x
        dataset.origin_y = origin_y
        dataset.cell_size = cell_size
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))

    elif dataset_str is "GroundTruth":
        # params
        dataset.cols = cols  # x
        dataset.rows = rows  # y
        dataset.origin_x = origin_x
        dataset.origin_y = origin_y
        dataset.cell_size = cell_size
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle(get_center(12, 10, dataset),
                                   get_dim(5, 7, dataset), dataset.map)
        dataset.map = add_obstacle(get_center(-7, 10, dataset),
                                   get_dim(10, 7, dataset), dataset.map)
        dataset.map = add_obstacle(get_center(0, -5, dataset),
                                   get_dim(10, 5, dataset), dataset.map)

    # no such dataset
    else:
        raise NameError("No such dataset exist")

    return dataset


def getLiDAR(robot_x,
              robot_y,
              current_map,
              gt_map,
              cell_size=0.01,
              lidar_range=1.0):
    # Update current map based on the current LiDAR sensing
    #    Input Format:
    #    robot_x            robot position (x)
    #    robot_y            robot position (y)
    #    current_map        map constructed up to now
    #    gt_map             groundtruth map
    #    cell_size          grid cell size (default 0.01)   # Q1. Can this be different from map cell_size..?
    #    lidar_range        LiDAR range (default 1.0)
    #
    #    Output Format:
    #    updated_map        updated map
    # ------------------------------------------------------

    updated_map = deepcopy(current_map)
    updated = np.zeros(np.shape(current_map))  # 1 if updated; 0 if not updated
    eps = 1.2  # precision factor   -> Q2. what is this
    n = np.ceil(2.0 * np.pi * lidar_range / cell_size * eps).astype(
        np.uint)  # number of rays
    dtheta = 2.0 * np.pi / n  # angular interval of rays

    stride = cell_size / eps
    n_d = np.ceil(lidar_range / stride).astype(np.uint)  # number of strides

    current_time = time.time()
    for i in range(n):
        theta_k = dtheta * i
        xy = np.array([robot_y, robot_x])
        occluded = False
        count = 0
        for j in range(n_d):
            xy += stride * np.array([-np.sin(theta_k), np.cos(theta_k)])

            cell_idx = np.floor(xy / cell_size).astype(np.uint)
            if cell_idx[0] < 0 or cell_idx[0] >= np.shape(
                    gt_map)[0] or cell_idx[1] < 0 or cell_idx[1] >= np.shape(
                        gt_map)[1]:
                break  #exceeded the map border

            if gt_map[cell_idx[0], cell_idx[1]] == 1:
                updated_map[cell_idx[0], cell_idx[1]] = 1
                count += 1
                if (count > 3):
                    break
    print "[getLiDAR] grid search time: ", time.time() - current_time
    # previous map fusion (if previously determined to be 1, set to 1) <- based on (unknown area = 0) If previously 1 but now 0 , set to 0
    current_time = time.time()
    updated_map2 = deepcopy(np.minimum(updated_map + current_map, 1))
    check_map = updated_map2 - updated_map
    updated_map2 = (updated_map2 - check_map)
    current_time = time.time()

    return updated_map2


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
