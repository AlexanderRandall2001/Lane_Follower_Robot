from controller import Robot
from Lane_Follower_Robot_PD_Controller import PDController
from Lane_Follower_Robot_CV import LaneDetector
import numpy as np

# Robot setup

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Camera

camera = robot.getDevice("camera")
camera.enable(timestep)

# Physical parameters

wheel_radius = 0.0205
L = 0.052

# Vision parameters

LANE_WIDTH_CM = 8.0
CHECK_OFFSET = 50
ROW_THICKNESS = 5

lane_detector = LaneDetector(
    lane_width_cm=LANE_WIDTH_CM,
    check_offset=CHECK_OFFSET,
    row_thickness=ROW_THICKNESS
)

# PD Controller parameters

Kp = 0.35
Kd = 0.12
ALPHA = 0.1
v = 0.1

pd_controller = PDController(Kp=Kp, Kd=Kd, alpha=ALPHA)

# Main loop

while robot.step(timestep) != -1:
    dt = timestep / 1000.0

    # Camera processing
 
    image_array = np.array(camera.getImageArray(), dtype=np.uint8)

 
    # Compute lane error
 
    error = lane_detector.compute_error(image_array)

    # PD control
 
    omega = pd_controller.update(error, dt)

    # Differential drive
    
    v_r = v + (L / 2) * omega
    v_l = v - (L / 2) * omega

    w_r = v_r / wheel_radius
    w_l = v_l / wheel_radius

    max_speed = 6.28
    w_r = max(min(w_r, max_speed), -max_speed)
    w_l = max(min(w_l, max_speed), -max_speed)

    right_motor.setVelocity(w_r)
    left_motor.setVelocity(w_l)