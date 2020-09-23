
# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
from arm import Arm
from maze import Maze
from search import *
from geometry import *
from const import *
from util import *

def transformToMaze(arm, goals, obstacles, window, granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            arm (Arm): arm instance
            goals (list): [(x, y, r)] of goals
            obstacles (list): [(x, y, r)] of obstacles
            window (tuple): (width, height) of the window
            granularity (int): unit of increasing/decreasing degree for angles

        Return:
            Maze: the maze instance generated based on input arguments.

    """
    # arm link -- (armBasePos, armLinkSpec)

    # rows/cols = int(  (max_angle-min_angle)/granularity + 1   )
    rows = int((arm.getArmLimit()[0][1]-arm.getArmLimit()[0][0])/granularity + 1)
    cols = int((arm.getArmLimit()[1][1] - arm.getArmLimit()[1][0]) / granularity + 1)
    # print(rows, cols)

    maze = [[SPACE_CHAR for i in range(cols)] for j in range(rows)]
    # maze_map[init_alpha][init_beta] = START_CHAR

    alpha_limits = arm.getArmLimit()[0]
    beta_limits = arm.getArmLimit()[1]
    # print(alpha_limits, beta_limits)
    offset = [alpha_limits[0], beta_limits[0]]
    # print(offset)
    alpha, beta = alpha_limits[0], beta_limits[0]  # min of alpha to start out

    alpha_max = alpha_limits[1]
    beta_max = beta_limits[1]

    init_angle = arm.getArmAngle()
    init_alpha = init_angle[0]
    init_beta = init_angle[1]

    #init_start = angleToIdx(init_angle, offset, granularity)
    start = [int(math.floor(init_angle[0]/granularity))*granularity,int(math.floor(init_angle[1]/granularity))*granularity]
    # print(start)
    #maze[init_start[0]][init_start[1]] = START_CHAR

    while alpha <= alpha_max:
        beta = beta_limits[0]
        while beta <= beta_max:

            arm.setArmAngle((alpha, beta))
            arm_pos = arm.getArmPos()
            tip = arm_pos[1][1]
            arm_dist = arm.getArmPosDist()   # [start,end,padding distance] for all arm links

            idx = angleToIdx([alpha, beta], offset, granularity)
            idx1 = idx[0]
            idx2 = idx[1]

            if alpha == start[0] and beta == start[1]:
                maze[idx1][idx2] = START_CHAR
                # print(alpha, beta)
                # print("start", (idx1, idx2))
            elif doesArmTouchObjects(arm_dist, obstacles):
                maze[idx1][idx2] = WALL_CHAR
            elif doesArmTipTouchGoals(tip, goals):
                maze[idx1][idx2] = OBJECTIVE_CHAR
            elif not isArmWithinWindow(arm_pos, window):
                maze[idx1][idx2] = WALL_CHAR
            else:
                maze[idx1][idx2] = SPACE_CHAR

            beta += granularity
        alpha += granularity
    #Maze --- def __init__(self, input_map, offsets, granularity)

    return Maze(maze, offset, granularity)











