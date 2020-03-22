# **************************************************************
# Project 08 - Disciplina de robótica Móvel UFC / IFCE / LAPISCO
#       Simulação 08 com Drone Mavic 2 Pro - Webots R2020a
#            Decolar e manter altitude fixa de 1 metro
#                  mostrando a imagem da câmera
#        Python 3.5 na IDE Pycharm - controller <extern>
#                By: Jefferson Silva Almeida
#                       Data: 22/03/2020
# **************************************************************

from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import Compass
from controller import Gyro
from controller import GPS
from controller import Camera
import numpy as np
import math
import cv2 as cv

def CLAMP(n, minn, maxn): # limitar numeros a uma faixa de valores
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

print("start the drone")
robot = Robot()

TIME_STEP = 32

# constantes
k_vertical_thrust = 68.5 # with this thrust, the drone lifts
k_vertical_offset = 0.6  # Vertical offset where the robot actually targets to stabilize itself
k_vertical_p = 3.0       # P constant of the vertical PID
k_roll_p = 50.0          # P constant of the roll PID
k_pitch_p = 30.0         # P constant of the pitch PID
M_PI = np.pi

#iniciar sensores
cameraRGB = robot.getCamera('camera')
Camera.enable(cameraRGB, TIME_STEP)
imu = InertialUnit("inertial unit")
imu.enable(TIME_STEP)
compass = Compass("compass")
compass.enable(TIME_STEP)
gyro = Gyro("gyro")
gyro.enable(TIME_STEP)
gps = GPS("gps")
gps.enable(TIME_STEP)

# iniciar motores
leftPropellerFront = robot.getMotor('front left propeller')
rightPropellerFront = robot.getMotor('front right propeller')
leftPropellerBack = robot.getMotor('rear left propeller')
rightPropellerBack = robot.getMotor('rear right propeller')

# permitir giro livre dos motores
leftPropellerFront.setPosition(float('inf'))
rightPropellerFront.setPosition(float('inf'))
leftPropellerBack.setPosition(float('inf'))
rightPropellerBack.setPosition(float('inf'))

# definir velocidade inicial dos motores
leftPropellerFront.setVelocity(1.0)
rightPropellerFront.setVelocity(1.0)
leftPropellerBack.setVelocity(1.0)
rightPropellerBack.setVelocity(1.0)

# camera gimbal
# camera_roll_motor = robot.getDevice('camera roll')
camera_roll_motor = robot.getMotor('camera roll')
# camera_pitch_motor = robot.getDevice('camera pitch')
camera_pitch_motor = robot.getMotor('camera pitch')

# esperar 1 segundo
while robot.step(TIME_STEP) != -1:
    if(robot.getTime() > 1.0):
        break

while robot.step(TIME_STEP) != -1:
    # ler os sensores
    roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
    pitch = imu.getRollPitchYaw()[1]
    altitude = gps.getValues()[1]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    print("roll %0.2f   pitch %0.2f   altitude %0.2fm   rollAcel %0.2f   pitchAcel %0.2f" % (roll, pitch, altitude, roll_acceleration, pitch_acceleration))

    # estabilizar camera de acordo com o giro do drone
    camera_roll_motor.setPosition(0.1 * roll_acceleration)
    camera_pitch_motor.setPosition(0.1 * pitch_acceleration)

    # variaveis de controle dos movimentos do drone
    roll_disturbance = 0.0      # mover para os lados -1 a +1
    pitch_disturbance = 0.0     # mover para frente e tras -1 a +1
    yaw_disturbance = 0.0       # rotacionar no eixo vertical -1 a +1
    target_altitude = 1.0       # manter a altitude (em metros)

    # calcular roll, pitch, yaw e vertical input
    roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
    pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * math.pow(clamped_difference_altitude, 3.0)

    # acionar os motores
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    # print('%0.2f, %0.2f, %0.2f, %0.2f' %(front_left_motor_input, front_right_motor_input, rear_left_motor_input, rear_right_motor_input))

    # set velocidades
    leftPropellerFront.setVelocity(front_left_motor_input)
    rightPropellerFront.setVelocity(-front_right_motor_input)
    leftPropellerBack.setVelocity(-rear_left_motor_input)
    rightPropellerBack.setVelocity(rear_right_motor_input)

    Camera.getImage(cameraRGB)
    Camera.saveImage(cameraRGB, 'color.png', 1)
    frameColor = cv.imread('color.png')
    cv.imshow("Color", frameColor)
    cv.waitKey(10)
