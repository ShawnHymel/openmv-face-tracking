# Untitled - By: sgmustadio - Sun Jul 18 2021

import time
from pyb import UART, LED

# Generic servo settings
speed_limit = 100        # Speed limit (0.25 us)/(10 ms) of servos
accel_limit = 20        # Acceleration limit (0.25 us)/(10 ms)/(80 ms) of servos

# Pan servo settings
servo_pan_ch = 2        # Pan servo channel
pulse_pan_min = 1000    # Pan minimum pulse (microseconds)
pulse_pan_max = 2000    # Pan maximum pulse (microseconds)
speed_pan = 1.8         # How fast the servo moves to track face (X direction)

# Tilt servo settings
servo_tilt_ch = 0       # Tilt servo channel
pulse_tilt_min = 1000   # Tilt minimum pulse (microseconds)
pulse_tilt_max = 2000   # Tilt maximum pulse (microseconds)
speed_tilt = 1.8        # How fast the servo moves to track face (Y direction)

# Other settings
threshold_x = 20        # Num pixels BB center x can be from CENTER_X
threshold_y = 20        # Num pixels BB center y can be from CENTER_Y
dir_x = 1               # Direction of servo movement (1 or -1)
dir_y = -1              # Direction of servo movement (1 or -1)
baud_rate = 9600        # Baud rate of Mini Maestro servo controller
speed_limit_min = 0     # Speed limit minimum (0 is infinite)
speed_limit_max = 10000 # Speed limit maximum
accel_limit_min = 0     # Acceleration limit minimum (0 is infinite)
accel_limit_max = 255   # Acceleration limit maximum


# Commands (for talking to Maestro servo controller)
cmd_set_target = 0x84
cmd_set_speed = 0x87
cmd_set_accel = 0x89

###############################################################################
# Functions

def servo_send_cmd(cmd, ch, payload):
    """
    Send generic compact protocol command to servo controller:
    | cmd | ch | msg lsb | msg msb |
    """

    # Check that channel is in range
    if (ch < 0) or (ch > 11):
        return

    # Construct message
    msg = bytearray()
    msg.append(cmd)
    msg.append(ch)
    msg.append(payload & 0x7F)
    msg.append((payload >> 7) & 0x7F)

    # Send a message
    uart.write(msg)

def servo_set_target(ch, pulse):
    """
    Write pulse width (in microseconds) to given channel to control servo.
    """

    # Pulse number is 4x pulse width (in microseconds)
    p_num = 4 * int(pulse)

    # Send command to servo controller
    servo_send_cmd(cmd_set_target, ch, p_num)

def servo_set_speed_limit(ch, speed):
    """
    Set speed limit of servo on a given channel.
    """

    # Check to make sure speed is in range
    speed = max(speed, speed_limit_min)
    speed = min(speed, speed_limit_max)

    # Send command to servo controller
    servo_send_cmd(cmd_set_speed, ch, speed)

def servo_set_speed_limit(ch, accel):
    """
    Set accel limit of servo on a given channel.
    """

    # Check to make sure speed is in range
    speed = max(accel, accel_limit_min)
    speed = min(accel, accel_limit_max)

    # Send command to servo controller
    servo_send_cmd(cmd_set_accel, ch, accel)

###############################################################################
# Main

# Pour a bowl of serial
uart = UART(1, baud_rate)

servo_pos_x = 1000
servo_pos_y = 2000

# Set servo speed limits on both servos
servo_set_speed_limit(servo_pan_ch, speed_limit)
servo_set_speed_limit(servo_tilt_ch, speed_limit)

# Set servo accel limits on both servos
servo_set_speed_limit(servo_pan_ch, accel_limit)
servo_set_speed_limit(servo_tilt_ch, accel_limit)

# Constrain servo positions to range of servos
servo_pos_x = max(servo_pos_x, pulse_pan_min)
servo_pos_x = min(servo_pos_x, pulse_pan_max)
servo_pos_y = max(servo_pos_y, pulse_pan_min)
servo_pos_y = min(servo_pos_y, pulse_pan_max)

# Go back and forth...FOREVER
while True:

    # Go to one corner
    servo_set_target(servo_pan_ch, 1200)
    servo_set_target(servo_tilt_ch, 1800)

    # Sleep for a couple of seconds
    time.sleep(2)

    # go to the other corner
    servo_set_target(servo_pan_ch, 1800)
    servo_set_target(servo_tilt_ch, 1200)

    # Sleep for a couple of seconds
    time.sleep(2)
