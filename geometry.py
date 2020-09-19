# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains geometry functions that relate with Part1 in MP2.
"""

import math
import numpy as np
from const import *

def computeCoordinate(start, length, angle):
    """Compute the end coordinate based on the given start position, length and angle.

        Args:
            start (tuple): base of the arm link. (x-coordinate, y-coordinate)
            length (int): length of the arm link
            angle (int): degree of the arm link from x-axis to counter-clockwise

        Return:
            End position (int,int):of the arm link, (x-coordinate, y-coordinate)
    """

    endX = start[0] + int(length * math.cos(math.radians(angle)))  # math.cos() should be in radians
    endY = start[1] - int(length * math.sin(math.radians(angle)))

    # print(angle, math.cos(math.radians(angle)), endX, endY)
    return (endX, endY)


def doesArmTouchObjects(armPosDist, objects, isGoal=False):
    """Determine whether the given arm links touch any obstacle or goal

        Args:
            armPosDist (list): start and end position and padding distance of all arm links [(start, end, distance)]
            objects (list): x-, y- coordinate and radius of object (obstacles or goals) [(x, y, r)]
            isGoal (bool): True if the object is a goal and False if the object is an obstacle.
                           When the object is an obstacle, consider padding distance.
                           When the object is a goal, no need to consider padding distance.
        Return:
            True if touched. False if not.
    """

    for arm in armPosDist:
        start = arm[0]
        end = arm[1]
        pad = arm[2]

        for obj in objects:
            rad = obj[2]
            if not isGoal:
                rad = obj[2] + pad
            if intersect((obj[0], obj[1]), start, end) <= rad:
                print("TOUCHING TOUCHING TOUCHING TOUCHING TOUCHING TOUCHING TOUCHING")
                return True

    return False


def intersect(obj_pos, start, end):
    # Inspired by:
    # https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    x = end[0] - start[0]
    y = end[1] - start[1]
    objX = obj_pos[0] - start[0]
    objY = obj_pos[1] - start[1]

    dot = objX * x + objY * y       # dot product of difference between end and start and difference between object and start
    len_ = x ** 2 + y ** 2          # pythagorean

    a = 0
    b = 0

    if len_ != 0:
        if dot > len_:
            a = end[0]
            b = end[1]
        else:
            a = start[0] + dot/len_ * x
            b = start[1] + dot/len_ * y
    else:
        a = start[0]
        b = start[1]

    dx = obj_pos[0] - a
    dy = obj_pos[1] - b
    dist = np.sqrt(dx * dx + dy * dy)  # pythagorean

    return dist


def doesArmTipTouchGoals(armEnd, goals):
    """Determine whether the given arm tick touch goals

        Args:
            armEnd (tuple): the arm tick position, (x-coordinate, y-coordinate)
            goals (list): x-, y- coordinate and radius of goals [(x, y, r)]. There can be more than one goal.
        Return:
            True if arm tip touches any goal. False if not.
    """
    for goal in goals:
        dist = np.sqrt(((armEnd[0] - goal[0]) ** 2) + ((armEnd[1] - goal[1]) ** 2))
        if dist <= goal[2]:
            print("GOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOALGOAL")
            return True

    return False


def isArmWithinWindow(armPos, window):
    """Determine whether the given arm stays in the window

        Args:
            armPos (list): start and end positions of all arm links [(start, end)]
            window (tuple): (width, height) of the window

        Return:
            True if all parts are in the window. False if not.
    """
    w = window[0]
    h = window[1]
    for arm in armPos:
        start = arm[0]
        end = arm[1]
        startX = start[0]
        startY = start[1]
        endX = end[0]
        endY = end[1]
        if startX < 0 or startX > w or startY < 0 or startY > h or endX < 0 or endX > w or endY < 0 or endY > h:
            print("OUT OF WINDOW OUT OF WINDOW OUT OF WINDOW OUT OF WINDOW")
            return False

    return True


if __name__ == '__main__':
    computeCoordinateParameters = [((150, 190),100,20), ((150, 190),100,40), ((150, 190),100,60), ((150, 190),100,160)]
    resultComputeCoordinate = [(243, 156), (226, 126), (200, 104), (57, 156)]
    testRestuls = [computeCoordinate(start, length, angle) for start, length, angle in computeCoordinateParameters]
    assert testRestuls == resultComputeCoordinate

    testArmPosDists = [((100,100), (135, 110), 4), ((135, 110), (150, 150), 5)]
    testObstacles = [[(120, 100, 5)], [(110, 110, 20)], [(160, 160, 5)], [(130, 105, 10)]]
    resultDoesArmTouchObjects = [
        True, True, False, True, False, True, False, True,
        False, True, False, True, False, False, False, True
    ]

    testResults = []
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle))

    print("\n")
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))

    assert resultDoesArmTouchObjects == testResults

    testArmEnds = [(100, 100), (95, 95), (90, 90)]
    testGoal = [(100, 100, 10)]
    resultDoesArmTouchGoals = [True, True, False]

    testResults = [doesArmTickTouchGoals(testArmEnd, testGoal) for testArmEnd in testArmEnds]
    assert resultDoesArmTouchGoals == testResults

    testArmPoss = [((100,100), (135, 110)), ((135, 110), (150, 150))]
    testWindows = [(160, 130), (130, 170), (200, 200)]
    resultIsArmWithinWindow = [True, False, True, False, False, True]
    testResults = []
    for testArmPos in testArmPoss:
        for testWindow in testWindows:
            testResults.append(isArmWithinWindow([testArmPos], testWindow))
    assert resultIsArmWithinWindow == testResults

    print("Test passed\n")
