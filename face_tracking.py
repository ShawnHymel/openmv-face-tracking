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

import sensor, image, time, utime, ustruct
from pyb import UART, LED

# Set this to True to see output and visualization
DEBUG = True
SERVO_PAN_EN = True
SERVO_TILT_EN = True

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


# Commands (for talking to Maestro servo controller)
cmd_set_target = 0x84

###############################################################################
# Functions


def servo_set_target(ch, pulse):
    """
    Write pulse width (in microseconds) to given channel to control servo.
    """

    # Check that channel is in range
    if (ch < 0) or (ch > 11):
        return

    # Pulse number is 4x pulse width (in microseconds)
    p_num = 4 * int(pulse)

    # Construct message
    msg = bytearray()
    msg.append(cmd_set_target)
    msg.append(ch)
    msg.append(p_num & 0x7F)
    msg.append((p_num >> 7) & 0x7F)

    # Send a message
    uart.write(msg)

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
uart = UART(1, baud_rate)

# Print out sensor stats
print("Width:", WIDTH, "Height:", HEIGHT)

# Create cascade for finding faces
face_cascade = image.HaarCascade("frontalface", stages=25)
print(face_cascade)

# Start clock
clock = time.clock()

# Initial servo positions
servo_pos_x = int(((pulse_pan_max - pulse_pan_min) / 2) + pulse_pan_min)
servo_pos_y = int(((pulse_tilt_max - pulse_tilt_min) / 2) + pulse_tilt_min)

# Create LED for debugging
if DEBUG:
    green_led = LED(2)

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

    else:
        if DEBUG:
            green_led.off()

        # ---------------------------------------------------------------------


    # Print FPS
    #if DEBUG:
        #print("FPS:", clock.fps())
