import numpy as np
import matplotlib.pyplot as plt

from utility import logging

def plotTrajectory(axis_robot, trajectory):
  traj_x = []
  traj_y = []
  for i in trajectory:
      traj_x.append(i[0])
      traj_y.append(i[1])
  axis_robot.plot(traj_x, traj_y)

def plot2DMap(figure, axis, prob_grid, origin_x, origin_y, cell_size):
    grid_rows = prob_grid.shape[0]
    grid_cols = prob_grid.shape[1]
    grid_corner_x = origin_x + (grid_cols - 1) * cell_size
    grid_corner_y = origin_y + (grid_rows - 1) * cell_size

    grid_X = np.linspace(origin_x, grid_corner_x, num=grid_cols)
    grid_Y = np.linspace(origin_y, grid_corner_y, num=grid_rows)

    temp = prob_grid
    z_min = 0
    z_max = 1

    c = axis.pcolor(grid_X, grid_Y, temp, vmin=z_min, vmax=z_max, cmap=plt.cm.Greys)

    axis.invert_yaxis() 
    axis.axis("equal")
    axis.axis(
        [
            origin_x - cell_size / 2,
            grid_corner_x + cell_size / 2,
            origin_y - cell_size / 2,
            grid_corner_y + cell_size / 2,
        ]
    )
    

    return c

def plotRobot(figure, axis, robot, conf, color_rgb=[0.4, 0.4, 0.4]):
    body_points = conf
    r = robot.sphere_radius(0)

    theta = np.linspace(0, 2 * np.pi, num=40)
    x = r * np.cos(theta) + body_points[0]
    y = r * np.sin(theta) + body_points[1]
    axis.plot(x, y, color=color_rgb)

def plotSignedDistanceField2D(
    figure, axis, field, origin_x, origin_y, cell_size, epsilon_dist=0):

    grid_rows = field.shape[0]
    grid_cols = field.shape[1]
    grid_corner_x = origin_x + (grid_cols - 1) * cell_size
    grid_corner_y = origin_y + (grid_rows - 1) * cell_size

    grid_X = np.linspace(origin_x, grid_corner_x, num=grid_cols)
    grid_Y = np.linspace(origin_y, grid_corner_y, num=grid_rows)

    z_min = np.amin(field)
    z_max = np.amax(field)
    c = axis.pcolor(grid_X, grid_Y, field, cmap="RdBu", vmin=z_min, vmax=z_max, alpha=0.3)
    # figure.colorbar(c, ax=axis)

    axis.invert_yaxis()

    axis.axis("equal")
    axis.axis(
        [
            origin_x - cell_size / 2,
            grid_corner_x + cell_size / 2,
            origin_y - cell_size / 2,
            grid_corner_y + cell_size / 2,
        ]
    )
    axis.set_xlabel("X/m")
    axis.set_ylabel("Y/m")
    axis.set_title("Signed Distance Field")
