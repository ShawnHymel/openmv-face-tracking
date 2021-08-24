"""
Random motion demo

Randomly move pan/tilt around.

License: 0BSD

Permission to use, copy, modify, and/or distribute this software for any purpose
with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
THIS SOFTWARE.

"""

import pyb, sensor, image, time, utime, ustruct, random

# Set this to true to see serial output and LED on face detection
DEBUG = True

# Generic servo settings
speed_limit = 100        # Speed limit (0.25 us)/(10 ms) of servos
accel_limit = 20        # Acceleration limit (0.25 us)/(10 ms)/(80 ms) of servos

# Pan servo settings
servo_pan_ch = 2        # Pan servo channel
pulse_pan_min = 1000    # Pan minimum pulse (microseconds)
pulse_pan_max = 2000    # Pan maximum pulse (microseconds)
speed_pan = 1.5         # How fast the servo moves to track face (X direction)

# Tilt servo settings
servo_tilt_ch = 0       # Tilt servo channel
pulse_tilt_min = 1000   # Tilt minimum pulse (microseconds)
pulse_tilt_max = 2000   # Tilt maximum pulse (microseconds)
speed_tilt = 1.5        # How fast the servo moves to track face (Y direction)

# Other settings
dir_x = 1               # Direction of servo movement (1 or -1)
dir_y = -1              # Direction of servo movement (1 or -1)
maestro_uart_ch = 1     # UART channel connected to Maestro board
baud_rate = 9600        # Baud rate of Mini Maestro servo controller
speed_limit_min = 0     # Speed limit minimum (0 is infinite)
speed_limit_max = 10000 # Speed limit maximum
accel_limit_min = 0     # Acceleration limit minimum (0 is infinite)
accel_limit_max = 255   # Acceleration limit maximum
wait_delay_min = 1000   # Minimum wait (ms) before moving around randomly
wait_delay_max = 3000   # Maximum wait (ms) before moving around randomly

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

# Configure camera
sensor.reset()
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.GRAYSCALE)

# Get center x, y of camera image
WIDTH = sensor.width()
HEIGHT = sensor.height()
CENTER_X = int(WIDTH / 2 + 0.5)
CENTER_Y = int(HEIGHT / 2 + 0.5)

# Pour a bowl of serial
uart = pyb.UART(maestro_uart_ch, baud_rate)

# Set servo speed limits on both servos
servo_set_speed_limit(servo_pan_ch, speed_limit)
servo_set_speed_limit(servo_tilt_ch, speed_limit)

# Set servo accel limits on both servos
servo_set_speed_limit(servo_pan_ch, accel_limit)
servo_set_speed_limit(servo_tilt_ch, accel_limit)

# Initial servo positions
servo_pos_x = int(((pulse_pan_max - pulse_pan_min) / 2) + pulse_pan_min)
servo_pos_y = int(((pulse_tilt_max - pulse_tilt_min) / 2) + pulse_tilt_min)

# Create random wait period before moving camera around
wait_timestamp = time.ticks_ms()
wait_delay = random.randrange(wait_delay_min, wait_delay_max + 1)

# Superloop
while(True):

    # Wait for some random amount of time before moving
    if (time.ticks_ms() - wait_timestamp) >= wait_delay:
        wait_timestamp = time.ticks_ms()
        wait_delay = random.randrange(wait_delay_min, wait_delay_max + 1)

        # Generate random x, y coordinates
        rnd_x = random.randrange(pulse_pan_min, pulse_pan_max + 1)
        rnd_y = random.randrange(pulse_tilt_min, pulse_tilt_max + 1)

        # Move to location
        print("Moving to: (" + str(rnd_x) + ", " + str(rnd_y) + ")")
        servo_set_target(servo_pan_ch, rnd_x)
        servo_set_target(servo_tilt_ch, rnd_y)
