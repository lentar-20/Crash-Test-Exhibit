#!/usr/bin/env python3

import numpy as np
import cv2
import time
import picamera
import serial
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
buttonPin = 16
buttonPin2 = 11
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buttonPin2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
ser = serial.Serial('/dev/arduino', 9600, timeout=1)
max_val = 0;
ser.reset_input_buffer()
path = r'/home/pi/Downloads/gatNg9E.png'
img = cv2.imread(path,1)
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
            #print(f'DEBUG: New frame of {l} bytes')
            # make a copy of the original image
            self.frames.append(buf)
            self.frame_num += 1
            self.memory    += l
        else:
            print(f'ERROR: partial frame of {len(buf)} bytes received belonging to previous frame')

################################################################################
# Main program - alter these variables according to needs
################################################################################
targetFPS, duration = 60, 1
width, height = 1024, 768
def run():

    ser.write(b"T\n")
    #print(f'Recording frames of {width}x{height} at {targetFPS} fps for {duration} seconds')

    ################################################################################
    # Recording loop
    ################################################################################
    with picamera.PiCamera(framerate=targetFPS) as camera:
        camera.resolution=(width,height)
        camera.start_preview()
        # Give the camera some warm-up time
        #time.sleep(0.2)
        output = SplitFrames()
        start = time.time()
        camera.start_recording(output, format='mjpeg')
        camera.wait_recording(duration)
        camera.stop_recording()
        finish = time.time()

    # Calculate a few statistics
    fps = output.frame_num / (finish - start)
    avg = int(output.memory/output.frame_num)
    MB  = output.memory/(1024*1024)
    print(f'Captured {output.frame_num} frames at {fps:.3f} fps, average bytes/frame={avg}, total RAM={MB:.3f} MB')


    cv2.namedWindow('Slow Motion Replay', cv2.WINDOW_NORMAL)
    #resize the window according to the screen resolution
    cv2.resizeWindow('Slow Motion Replay', 1920, 1080)
    cv2.setWindowProperty('Slow Motion Replay', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
################################################################################
    # Playback loop - grab frames from list and display with delay
    ################################################################################
    for frame_num, frame in enumerate(output.frames):
        #print(f'DEBUG: Playing back frame {frame_num}')
        #cv2.WINDOW_NORMAL makes the output window resizealbe
        
        im = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
        # display the output image with text over it
        cv2.imshow('Slow Motion Replay', im)
        cv2.waitKey(32)
    backImage = cv2.imread(path,1)
    imageText = backImage.copy()
    #let's write the text you want to put on the image
    ser.reset_input_buffer()
    val = ser.read()
    print(val)
    text = "Max Force:"+str(int.from_bytes(val,"big",signed = False))
    #org: Where you want to put the text
    print(text)
    org = (100,100)
    # write the text on the input image
    cv2.putText(imageText, text, org, fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 3.0, color = (190,150,37), thickness = 10)
    #cv2.destroyAllWindows()
    #cv2.WINDOW_NORMAL makes the output window resizealbe
    #cv2.namedWindow('Slow Motion Replay', cv2.WINDOW_NORMAL)
    #resize the window according to the screen resolution
    #cv2.setWindowProperty('Slow Motion Replay', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    #cv2.resizeWindow('Slow Motion Replay', 1920, 1080)
    cv2.imshow('Slow Motion Replay', imageText)
    key = cv2.waitKey(2000)
    if key==27:
        cv2.destroyAllWindows()
run()
while True:
    buttonState = GPIO.input(buttonPin)
    if buttonState == False:
        endTime = time.time()+1.0
        print("b1 pressed")
        while time.time()<endTime:
            buttonState2 = GPIO.input(buttonPin2)
            if buttonState2 == False:
                print("b2 pressed")
                ser.write(b"T\n")
                ser.write(b"T\n")
                ser.write(b"T\n")
                run()
                break
    else:
        ser.write(b"F\n")
        key = cv2.waitKey(20)
        if key==27:
            cv2.destroyAllWindows()

