import numpy as np
import cv2 as cv
import argparse
from datetime import datetime
import os
# Local modules
from videostream import openStream, StreamType, KinectVideoStream
from circledetector import CircleDetector
from calibrationhandler import CalibrationHandler
from utils import stop



def main():
    # streams = [kinectStream, flirStream] = [KinectVideoStream(), openStream(targetHeights=[768])]
    streams = [kinectStream] = [KinectVideoStream()]
    # Main capture and calibrate loop
    while True:
        frames = kinectStream.read()#, flirStream.read()]
        col = frames.color
        depth = frames.depth
        image_8bit = cv.convertScaleAbs(depth, alpha=0.03)

        # print(type(depth[0,0]))
        cv.imshow("color", image_8bit)

        if cv.waitKey(1) == ord('q'):
            break

if __name__ == '__main__':
    main()