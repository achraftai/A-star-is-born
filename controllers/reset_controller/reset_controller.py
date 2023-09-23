"""reset_controller controller."""
import os
from audioop import reverse

import A_star
import math
from controller import Robot
from controller import Motor
from controller import PositionSensor

robot = Robot()
left_motor = robot.getDevice('motor_1')
right_motor = robot.getDevice('motor_2')

timeStep = 64
max_speed = 6.28
wheel_radius = 0.025
axle_length = 0.090
robot_radius = 0.045
tangential_speed = max_speed * wheel_radius
rotational_speed = tangential_speed / (math.pi*axle_length)

#
def motorRotateLeft():
    left_motor.setVelocity(-max_speed)
    right_motor.setVelocity(max_speed)

def motorRotateRight():
    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(-max_speed)


def main():
    rx, ry = A_star.driver()
    # # # # # print(rx)
    # # # # # print(ry)
    # # # # # print(" ")
    new_rx = rx[::-1]
    new_ry = ry[::-1]
    # # # # # print(len(new_rx))
    # # # # # print(len(new_ry))
    # # # #
    # # # # #Below is where we are converting the pycharm coordinates to the WeBots coordinates
    wb_rx = []
    wb_ry = []

    for i in range (len(new_rx)):
         wb_rx.append(convert_scale(new_rx[i]))
         wb_ry.append(convert_scale(new_ry[i]))

    print(wb_rx)
    print(wb_ry)


    dx, dy, movex, movey = directions(wb_rx, wb_ry)


    num_loops = len(dx)-1
    left_motor.setPosition(float('inf'))
    # left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    # right_motor.setVelocity(0.0)

    leftEncoder = left_motor.getPositionSensor()  # Step 1
    rightEncoder = right_motor.getPositionSensor()

    leftEncoder.enable(timeStep)  # Step 2
    rightEncoder.enable(timeStep)



    # -----------------------------------------------------------------------------------------------------------------
    #MOVEMENT 1
    # -----------------------------------------------------------------------------------------------------------------
    dist = movement([dx[0], dx[1]], [dy[0], dy[1]])
    angle = 0
    arc = (angle * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = (dist) / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break
    # -----------------------------------------------------------------------------------------------------------------
    # MOVEMENT 2
    # -----------------------------------------------------------------------------------------------------------------
    dist = movement([dx[1], dx[2]], [dy[1], dy[2]])
    a1 = math.pi/2 - math.acos((dy[2]-dy[1])/dist)
    motorRotateRight()
    arc = (a1 * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = dist / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break
    # -----------------------------------------------------------------------------------------------------------------
    # MOVEMENT 3
    # -----------------------------------------------------------------------------------------------------------------

    dist = movement([dx[2], dx[3]], [dy[2], dy[3]])
    a2 = math.asin((dy[3]-dy[2])/dist) - a1
    motorRotateRight()
    arc = (a2 * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = dist / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break
    # -----------------------------------------------------------------------------------------------------------------
    # MOVEMENT 4
    # -----------------------------------------------------------------------------------------------------------------
    dist = movement([dx[3], dx[4]], [dy[3], dy[4]])
    a3 = a2 + a1 - math.pi/2
    motorRotateLeft()
    arc = (a3 * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = dist / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break
    # -----------------------------------------------------------------------------------------------------------------
    # MOVEMENT 5
    # -----------------------------------------------------------------------------------------------------------------

    dist = movement([dx[4], dx[5]], [dy[4], dy[5]])
    a4 = math.asin((dy[5]-dy[4])/dist)
    motorRotateRight()
    arc = (a4 * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = dist / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break
    # -----------------------------------------------------------------------------------------------------------------
    # MOVEMENT 6
    # -----------------------------------------------------------------------------------------------------------------
    dist = movement([dx[5], dx[6]], [dy[5], dy[6]])
    a5 = a4
    motorRotateLeft()
    arc = (a5 * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = dist / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break
    # -----------------------------------------------------------------------------------------------------------------
    # MOVEMENT 7
    # -----------------------------------------------------------------------------------------------------------------
    dist = movement([dx[6], dx[7]], [dy[6], dy[7]])
    a6 = math.asin((dy[7]-dy[6])/dist)
    motorRotateLeft()
    arc = (a6 * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = dist / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break
    # -----------------------------------------------------------------------------------------------------------------
    # MOVEMENT 8
    # -----------------------------------------------------------------------------------------------------------------

    dist = movement([dx[7], dx[8]], [dy[7], dy[8]])
    a7 = a6
    motorRotateLeft()
    arc = (a7 * robot_radius)

    start_time = robot.getTime()
    duration = abs(arc) / tangential_speed

    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break

    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

    start_time = robot.getTime()
    duration = (dist + 4*robot_radius) / tangential_speed
    while robot.step(timeStep) != -1:
        if robot.getTime() > start_time + duration:
            break



def convert_scale(pc_value):
    wb_startxy = 0.5
    pc_startxy = 10
    interval = -0.025     #1 pixel in pycharm is 0.025m in webots
    # webots origin in pycharm is 30,30
    if pc_value < 30 :
        wb_value = wb_startxy + ((pc_value - pc_startxy) * interval)
    elif pc_value > 30:
        wb_value = ((pc_value - 30) * interval)
    elif pc_value == 30:
        return 0


    return wb_value

def movement(wb_rx, wb_ry):
    dist = math.sqrt((wb_rx[1] - wb_rx[0])**2 + (wb_ry[1] - wb_ry[0])**2)

    # find angle
    angle = math.sin((wb_rx[1] - wb_rx[0]) / dist)

    if angle < 0:
        motorRotateRight()

    if angle > 0:
        motorRotateLeft()

    return dist

def directions(rx, ry):
    nx = []
    ny = []
    for i in range(len(rx)-1):
        x = rx[i + 1] - rx[i]
        if x == 0:
            nx.append(0)
        elif x > 0:
            nx.append(1)
        elif x < 0:
            nx.append(-1)

        y = ry[i + 1] - ry[i]
        if y == 0:
            ny.append(0)
        elif y > 0:
            ny.append(1)
        elif y < 0:
            ny.append(-1)

    xvals = [0]
    yvals = [0]
    for i in range(len(nx)-1):
        if nx[i+1] != nx[i]:
            xvals.append(i+1)
        if ny[i+1] != ny[i]:
            yvals.append(i+1)


    vals = sorted(set(xvals + yvals), reverse=False)
    vals.append(len(nx)-1)
    x = []
    xd = []
    y = []
    yd = []
    for i in range(len(vals)-1):
        x.append(rx[vals[i]])
        xd.append(nx[vals[i]])
        y.append(ry[vals[i]])
        yd.append(ny[vals[i]])
    x.append(rx[len(rx)-1])
    y.append(ry[len(ry) - 1])
    print(x)
    print(y)
    return x, y, xd, yd




if __name__== "__main__":
    main()



       

