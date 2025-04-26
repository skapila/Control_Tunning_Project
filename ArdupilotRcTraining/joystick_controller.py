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

# Main loop to send RC commands
try:
    roll = pitch = yaw = 1500
    throttle=1000
    switch=0
    while True:
        pygame.event.pump()  # Process joystick events
        
        # Check if triangle button is pressed (usually index 3)
        triangle_pressed = joystick.get_button(3) 
        if(triangle_pressed):
           switch=not(switch) 
       

        if (switch==1):
            # Read joystick axes
            roll = scale_joystick(joystick.get_axis(3))     # Left/Right
            pitch = scale_joystick(joystick.get_axis(4))   # Forward/Backward
            throttle = scale_joystick(-joystick.get_axis(1))# Throttle (inverted)
            yaw = scale_joystick(joystick.get_axis(0))      # Yaw
        elif(switch==0):
            roll = pitch = yaw = 1500
            throttle=1000
           

        # Print values for debugging
        print(f"Roll: {roll}, Pitch: {pitch}, Throttle: {throttle}, Yaw: {yaw}")

        # Send RC command via MAVLink
        mavlink_connection.mav.rc_channels_override_send(
            mavlink_connection.target_system,  # Target system ID
            mavlink_connection.target_component,  # Target component ID
            roll, pitch, throttle, yaw,  # RC channels 1-4
            0, 0, 0, 0  # Unused channels
        )

        time.sleep(0.1)  # 10 Hz update rate

except KeyboardInterrupt:
    print("Exiting...")
    pygame.quit()

