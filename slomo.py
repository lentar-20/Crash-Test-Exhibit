#!/usr/bin/env python3

import numpy as np
import cv2
import picamera
import serial
import RPi.GPIO as GPIO
import time

# GLOBAL VARIABLES
GPIO.setmode(GPIO.BOARD)
# RASPI GPIO PINS FOR BUTTON AND BEAM BREAK
buttonPin = 16
beamBreakPin = 11
# number of FPS to record, duration to record
targetFPS, duration = 60, 1
# recording resolution
width, height = 1024, 768
# screen resolution
res = (1920, 1080)
# location of force reading text on screen
org = (100, 100)
# background image file path
imagePath = r'/home/pi/Downloads/gatNg9E.png'
# SETUP
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(beamBreakPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
ser = serial.Serial('/dev/arduino', 9600, timeout=1)
ser.reset_input_buffer()
img = cv2.imread(imagePath, 1)


# SPLIT FRAMES CLASS
class SplitFrames(object):
    def __init__(self):
        self.frame_num = 0
        # List of all the JPEG-encoded frames we receive
        self.frames = []
        # Total memory used
        self.memory = 0

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame
            l = len(buf)
            # make a copy of the original image
            self.frames.append(buf)
            self.frame_num += 1
            self.memory += l
        else:
            print(f'ERROR: partial frame of {len(buf)} bytes received belonging to previous frame')

# MAIN
def run():
    ser.write(b"T\n")

    # DEBUG
    # print(f'Recording frames of {width}x{height} at {targetFPS} fps for {duration} seconds')

    # RECORD FRAMES
    with picamera.PiCamera(framerate=targetFPS) as camera:
        camera.resolution = (width, height)
        camera.start_preview()
        output = SplitFrames()
        start = time.time()
        camera.start_recording(output, format='mjpeg')
        camera.wait_recording(duration)
        camera.stop_recording()
        finish = time.time()

    # DEBUG
    # fps = output.frame_num / (finish - start)
    # avg = int(output.memory/output.frame_num)
    # MB  = output.memory/(1024*1024)
    # print(f'Captured {output.frame_num} frames at {fps:.3f} fps, average bytes/frame={avg}, total RAM={MB:.3f} MB')

    # BUILD DISPLAY WINDOW
    cv2.namedWindow('Slow Motion Replay', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Slow Motion Replay', res[0], res[1])
    cv2.setWindowProperty('Slow Motion Replay', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # PLAYBACK LOOP
    for frame_num, frame in enumerate(output.frames):
        # print(f'DEBUG: Playing back frame {frame_num}')
        # cv2.WINDOW_NORMAL makes the output window resizealbe

        im = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
        # display the output image with text over it
        cv2.imshow('Slow Motion Replay', im)
        cv2.waitKey(32)

    # BACKGROUND IMAGE
    backImage = cv2.imread(imagePath, 1)

    # ADD FORCE READING
    imageText = backImage.copy()
    ser.reset_input_buffer()
    val = ser.read()
    text = "Max Force:" + str(int.from_bytes(val, "big", signed=False))
    # write the text on the input image
    cv2.putText(imageText, text, org, fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=3.0, color=(190, 150, 37),
                thickness=10)

    # DISPLAY BACKGROUND IMAGE WITH FORCE READING
    cv2.imshow('Slow Motion Replay', imageText)

    # CHECK FOR ESC KEY
    key = cv2.waitKey(2000)
    if key == 27:
        cv2.destroyAllWindows()

# RUN ON INIT
run()

# MAIN LOOP
while True:
    # CHECK FOR BUTTON PRESS
    buttonState = GPIO.input(buttonPin)
    if not buttonState:
        endTime = time.time() + 1.0
        # CHECK FOR BEAM BREAK
        while time.time() < endTime:
            beamState = GPIO.input(beamBreakPin)
            if not beamState:
                # RESET MAX FORCE
                ser.write(b"T\n")
                ser.write(b"T\n")
                ser.write(b"T\n")
                # RUN PROGRAM
                run()
                break
    else:
        # TELL ARDUINO NOT TO RESET MAX FORCE
        ser.write(b"F\n")
        # CHECK FOR ESC KEY
        key = cv2.waitKey(20)
        if key == 27:
            cv2.destroyAllWindows()
