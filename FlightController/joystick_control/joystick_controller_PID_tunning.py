import pygame
from pymavlink import mavutil
import time

# Initialize joystick
pygame.init()
pygame.joystick.init()

# Check for joystick availability
if pygame.joystick.get_count() == 0:
    print("No joystick found!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Using joystick: {joystick.get_name()}")

# Connect to MAVLink (replace with correct connection string)
mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Wait for a heartbeat to confirm connection
print("Waiting for heartbeat...")
mavlink_connection.wait_heartbeat()
print("Heartbeat received! MAVLink connection established.")

# Function to scale joystick input to PWM values
def scale_joystick(value, min_out=1000, max_out=2000):
    return int((value + 1) / 2 * (max_out - min_out) + min_out)


# Button mappings for PlayStation controllers
BUTTON_TRIANGLE = 2
BUTTON_CIRCLE = 1
BUTTON_CROSS = 0
BUTTON_SQUARE = 3 

# Default neutral values
roll = 1500
pitch = 1500
throttle = 1500
yaw = 1500

# pre state
triangle_pre=0
circle_pre=0
cross_pre=0
square_pre=0

# Main loop to send RC commands
try:
    while True:        
        pygame.event.pump()  # Process joystick events

        # Read button presses
        triangle = joystick.get_button(BUTTON_TRIANGLE)
        circle = joystick.get_button(BUTTON_CIRCLE)
        cross = joystick.get_button(BUTTON_CROSS)
        square = joystick.get_button(BUTTON_SQUARE)
                   
                   
        if triangle or triangle_pre:  # Read all values from joystick
          roll = scale_joystick(joystick.get_axis(3))  # Left/Right
          pitch = scale_joystick(joystick.get_axis(4))  # Forward/Backward
          throttle = scale_joystick(-joystick.get_axis(1))  # Throttle (inverted)
          yaw = scale_joystick(joystick.get_axis(0))  # Yaw
        elif cross or cross_pre:  # Roll and Throttle vary, others static
            roll = scale_joystick(joystick.get_axis(3))
            throttle = scale_joystick(-joystick.get_axis(1))
        elif square or square_pre:  # Pitch and Throttle vary, others static
            pitch = scale_joystick(joystick.get_axis(4))
            throttle = scale_joystick(-joystick.get_axis(1))
        elif circle or circle_pre:  # Throttle and Yaw vary, others static
            throttle = scale_joystick(-joystick.get_axis(3))
            yaw = scale_joystick(joystick.get_axis(1))

        # Print values for debugging
        print(f"Roll: {roll}, Pitch: {pitch}, Throttle: {throttle}, Yaw: {yaw}")

        # Send RC command via MAVLink
        mavlink_connection.mav.rc_channels_override_send(
            mavlink_connection.target_system,  # Target system ID
            mavlink_connection.target_component,  # Target component ID
            roll, pitch, throttle, yaw,  # RC channels 1-4
            0, 0, 0, 0  # Unused channels
        )

        if(triangle or circle or cross or square):
          triangle_pre=triangle
          circle_pre=circle
          cross_pre=cross
          square_pre=square
          roll = 1500
          pitch = 1500
          throttle = 1500
          yaw = 1500

          
        time.sleep(0.04)  # 25 Hz update rate
                
except KeyboardInterrupt:
    print("Exiting...")
    pygame.quit()

