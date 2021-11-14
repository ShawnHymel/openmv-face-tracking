"""
Pan/tilt face tracking demo

Connect OpenMV to Mini Maestro servo controller to control pan/tilt mechanism.
This program will have the servos move such that the largest face seen by the
camera will be centered.

Authors: Shawn Hymel (@ShawnHymel), Jorvon Moss (@Odd_Jayy)
Date: July 18, 2021

Hardware:
 - OpenMV H7
 - Pololu Mini Maestro (servo controller)

Connections:

 OpenMV | Maestro
--------|---------
   GND  |   GND
    P0  |    TX
    P1  |    RX

Optionally add LED(s) to pin P2 on the OpenMV. They will light up when a face
is detected.

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
DEBUG_FPS = False

# Set to true to enable pan and tilt motion
SERVO_PAN_EN = True
SERVO_TILT_EN = True

# Generic servo settings
speed_limit = 20        # Speed limit (0.25 us)/(10 ms) of servos
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

# Left ear servo settings
servo_ear_l_ch = 3      # Left ear servo channel
pulse_ear_l_min = 1000  # Left ear minimum pulse (microseconds)
pulse_ear_l_max = 2000  # Left ear maximum pulse (microseconds)
speed_ear_l = 1.5       # How fast the servo moves (left ear)

# Right ear servo settings
sero_ear_r_ch = 4       # Right ear servo channel
servo_ear_r_ch = 3      # Right ear servo channel
pulse_ear_r_min = 1000  # Right ear minimum pulse (microseconds)
pulse_ear_r_max = 2000  # Right ear maximum pulse (microseconds)
speed_ear_r = 1.5       # How fast the servo moves (right ear)

# Random motion settings
rnd_pan_min = 1350      # Minimum pan random motion
rnd_pan_max = 1650      # Maximum pan random motion
rnd_tilt_min = 1350     # Minimum pan random motion
rnd_tilt_max = 1650     # Maximum pan random motion
wait_delay_min = 1000   # Minimum wait (ms) before moving around randomly
wait_delay_max = 3000   # Maximum wait (ms) before moving around randomly

# GPIO pins
led = pyb.Pin("P2", pyb.Pin.OUT_PP) # LED that lights up on face detect
snd = pyb.Pin("P3", pyb.Pin.OUT_PP) # SND that lights up on face detect

# Other settings
threshold_x = 10        # Num pixels BB center x can be from CENTER_X
threshold_y = 10        # Num pixels BB center y can be from CENTER_Y
dir_x = 1               # Direction of servo movement (1 or -1)
dir_y = -1              # Direction of servo movement (1 or -1)
maestro_uart_ch = 1     # UART channel connected to Maestro board
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

def servo_set_accel_limit(ch, accel):
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

# Print out sensor stats
print("Width:", WIDTH, "Height:", HEIGHT)

# Create cascade for finding faces
face_cascade = image.HaarCascade("frontalface", stages=25)
print(face_cascade)

# Start clock
clock = time.clock()

# Set servo speed limits on servos
servo_set_speed_limit(servo_pan_ch, speed_limit)
servo_set_speed_limit(servo_tilt_ch, speed_limit)
servo_set_speed_limit(servo_ear_l_ch, speed_limit)
servo_set_speed_limit(servo_ear_r_ch, speed_limit)

# Set servo accel limits on servos
servo_set_accel_limit(servo_pan_ch, accel_limit)
servo_set_accel_limit(servo_tilt_ch, accel_limit)
servo_set_accel_limit(servo_ear_l_ch, accel_limit)
servo_set_accel_limit(servo_ear_r_ch, accel_limit)

# Initial servo positions
servo_pos_x = int(((pulse_pan_max - pulse_pan_min) / 2) + pulse_pan_min)
servo_pos_y = int(((pulse_tilt_max - pulse_tilt_min) / 2) + pulse_tilt_min)

# Create LED for debugging
if DEBUG:
    green_led = pyb.LED(2)

# Create random wait period before moving camera around
wait_timestamp_face = time.ticks_ms()
wait_timestamp_ears = time.ticks_ms()
wait_delay_face = random.randrange(wait_delay_min, wait_delay_max + 1)
wait_delay_ears = random.randrange(wait_delay_min, wait_delay_max + 1)

# Superloop
while(True):

    # Take timestamp (for calculating FPS)
    clock.tick()

    # Take photo
    img = sensor.snapshot()

    # Find faces in image
    objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)

    # Print out all faces in image
    largest_face_size = 0
    largest_face_bb = None
    for r in objects:

        # Find largest bounding box
        face_size = r[2] * r[3]
        if (face_size > largest_face_size):
            largest_face_size = face_size
            largest_face_bb = r

        # Draw bounding boxes and print out elements
        if DEBUG:
            img.draw_rectangle(r)
            #print("Face:", r)

    # Find distance from center of camera to largest face
    if largest_face_bb is not None:

        # Print largest face
        if DEBUG:
            print("Face:", largest_face_bb)
            print(sensor.width(), sensor.height())
            green_led.on()

        # Turn on LED and play sound
        led.high()
        snd.low()

        # Reset wait delay
        wait_timestamp = time.ticks_ms()

        # Find x, y of center of largest face in image
        face_x = largest_face_bb[0] + int((largest_face_bb[2]) / 2 + 0.5)
        face_y = largest_face_bb[1] + int((largest_face_bb[3]) / 2 + 0.5)

        # Demo: draw line from center of image to center of face
        if DEBUG:
            img.draw_line(CENTER_X, CENTER_Y, face_x, face_y)

        # Figure out how far away from center the face is (minus the dead zone)
        diff_x = face_x - CENTER_X
        if abs(diff_x) > threshold_x:
            if diff_x > 0:
                diff_x = diff_x - threshold_x
            else:
                diff_x = diff_x + threshold_x
        else:
            diff_x = 0
        diff_y = face_y - CENTER_Y
        if abs(diff_y) > threshold_y:
            if diff_y > 0:
                diff_y = diff_y - threshold_y
            else:
                diff_y = diff_y + threshold_y
        else:
            diff_y = 0

        # Calculate how fast the servo should move based on distance
        mov_x = dir_x * speed_pan * diff_x
        mov_y = dir_y * speed_tilt * diff_y

        # Print out relative movement
        if DEBUG:
            print("Move:", mov_x, ",", mov_y)

        # Adjust camera position left/right and up/down
        servo_pos_x = servo_pos_x + mov_x
        servo_pos_y = servo_pos_y + mov_y

        # Constrain servo positions to range of servos
        servo_pos_x = max(servo_pos_x, pulse_pan_min)
        servo_pos_x = min(servo_pos_x, pulse_pan_max)
        servo_pos_y = max(servo_pos_y, pulse_pan_min)
        servo_pos_y = min(servo_pos_y, pulse_pan_max)

        # Set pan/tilt
        if SERVO_PAN_EN:
            if DEBUG:
                print("Pulse X:", int(servo_pos_x))
            servo_set_target(servo_pan_ch, servo_pos_x)
        if SERVO_TILT_EN:
            if DEBUG:
                print("Pulse Y:", int(servo_pos_y))
            servo_set_target(servo_tilt_ch, servo_pos_y)

    # No face detected
    else:

        # Make sure LEDs are off and no sound is playing
        led.low()
        snd.high()
        if DEBUG:
            green_led.off()

        # Wait for some random amount of time before moving...randomly
        if (time.ticks_ms() - wait_timestamp_face) >= wait_delay_face:
            wait_timestamp_face = time.ticks_ms()
            wait_delay_face = random.randrange(wait_delay_min, wait_delay_max + 1)

            # Generate random x, y coordinates
            rnd_x = random.randrange(rnd_pan_min, rnd_pan_max + 1)
            rnd_y = random.randrange(rnd_tilt_min, rnd_tilt_max + 1)

            # Move to location
            if DEBUG:
                print("Moving to: (" + str(rnd_x) + ", " + str(rnd_y) + ")")
            servo_set_target(servo_pan_ch, rnd_x)
            servo_set_target(servo_tilt_ch, rnd_y)

        # ---------------------------------------------------------------------


    # Print FPS
    if DEBUG_FPS:
        print("FPS:", clock.fps())
