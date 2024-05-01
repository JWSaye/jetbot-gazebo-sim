#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pygame
import threading

# Initialize Pygame
pygame.init()
pygame.font.init()
font = pygame.font.Font(None, 36)
screen = pygame.display.set_mode((550, 300))
pygame.display.set_caption('Robot Control Panel')

# Define constants
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.05

# Initialize the velocity values
linear_velocity = 0.0
angular_velocity = 0.0
last_linear_velocity = 0.0
last_angular_velocity = 0.0
last_key_pressed = ''

# Publisher for data collection signal
data_pub = rospy.Publisher('data_collection_signal', String, queue_size=10)
velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Function to publish velocity if there's a change
def publish_velocity():
    global last_linear_velocity, last_angular_velocity
    rospy.init_node('teleop_keyboard', anonymous=True)
    rate = rospy.Rate(20)  # Adjusted rate
    while not rospy.is_shutdown():
        if linear_velocity != last_linear_velocity or angular_velocity != last_angular_velocity:
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            velocity_pub.publish(twist)
            last_linear_velocity = linear_velocity
            last_angular_velocity = angular_velocity
        rate.sleep()

# Function to handle Pygame events
def check_keys():
    global linear_velocity, angular_velocity, last_key_pressed
    running = True
    while running and not rospy.is_shutdown():
        screen.fill((0, 0, 0))
        display_control_instructions()
        display_text("Set Linear Vel: {:.2f}".format(linear_velocity), (50, 200))
        display_text("Set Angular Vel: {:.2f}".format(angular_velocity), (50, 230))

        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                data_pub.publish("shutdown")
                rospy.signal_shutdown('X clicked')
                running = False
            elif event.type == pygame.KEYDOWN:
                update_velocity_on_key_press(event.key)
                last_key_pressed = pygame.key.name(event.key)
                if event.key == pygame.K_c:
                    data_pub.publish("start")
                elif event.key == pygame.K_ESCAPE:
                    data_pub.publish("shutdown")
                    rospy.signal_shutdown('ESC key pressed')
                    running = False

            elif event.type == pygame.KEYUP:
                update_velocity_on_key_release(event.key)

        pygame.time.delay(100)  # Reduce the delay

# Function to display text on Pygame screen
def display_text(message, position, color=(255, 255, 255)):
    text = font.render(message, True, color)
    screen.blit(text, position)

# Display control instructions
def display_control_instructions():
    instructions = [
        "Control Your JetBot!",
        "w/s: Increase/Decrease linear velocity",
        "a/d: Increase/Decrease angular velocity",
        "space: Stop",
        "c: Capture image",
        "ESC: Quit and shutdown data collection"
    ]
    y_pos = 20
    for line in instructions:
        display_text(line, (50, y_pos))
        y_pos += 30

# Update velocities based on key press
def update_velocity_on_key_press(key):
    global linear_velocity, angular_velocity
    if key == pygame.K_w:
        linear_velocity = LIN_VEL_STEP_SIZE
    elif key == pygame.K_s:
        linear_velocity = -LIN_VEL_STEP_SIZE
    elif key == pygame.K_a:
        angular_velocity = ANG_VEL_STEP_SIZE
    elif key == pygame.K_d:
        angular_velocity = -ANG_VEL_STEP_SIZE
    elif key == pygame.K_SPACE:
        linear_velocity = 0.0
        angular_velocity = 0.0

# Update velocities on key release to zero them out
def update_velocity_on_key_release(key):
    global linear_velocity, angular_velocity
    if key in [pygame.K_w, pygame.K_s]:
        linear_velocity = 0.0
    if key in [pygame.K_a, pygame.K_d]:
        angular_velocity = 0.0

if __name__ == '__main__':
    key_thread = threading.Thread(target=check_keys)
    key_thread.start()
    try:
        publish_velocity()
    finally:
        pygame.quit()
        key_thread.join()