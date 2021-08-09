import cv2 as cv
import numpy as np
import json
import sys
import time
import argparse
from datetime import datetime
import os
# Local modules
from videostream import openStreams, StreamType
from utils import scaleForHconcat, stop, openCalibrationFile
from circledetector import CircleDetector
from circlegridfinder import CircleGridFinder

def main(cameraTypes, calibFile1, calibFile2):

    # Open calibration files and read contents
    matrix1, dist1 = openCalibrationFile(calibFile1)
    matrix2, dist2 = openCalibrationFile(calibFile2)

    # If empty, quit
    if not dist1.any():
        sys.exit("Cannot read matrices from Kinect calibration file.")
    if not dist2.any():
        sys.exit("Cannot read matrices from FLIR calibration file.")


    if "F" in cameraTypes and "K" in cameraTypes: # FLIR + Kinect
        streams = openStreams(targetHeights=[720, 768])
        types = [StreamType.rgb, StreamType.ir]
    if "W" in cameraTypes and "K" in cameraTypes: # Webcam + Kinect
        streams = openStreams(targetCameras=[1, 0])
        types = [StreamType.rgb, StreamType.rgb]
    else:
        raise Exception("Invalid camera types selected.")

    # Number of images to take for rectification
    rectImageCount = 15
    # termination criteria for iterative functions
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 0.0001)

    # Initialize blob detector with circle parameters
    circleDetector = CircleDetector()
    # Initialize grid finding object with both streams simultaneously
    gridFinder = CircleGridFinder(cameraType, streams, types, [circleDetector, circleDetector], rectImageCount)

    # Initial message to user
    print("Press q to exit program.")

    gridFinder.start()
    while not gridFinder.finished:
        # Get next frame from both cameras
        frames = []
        for stream in streams:
            if stream.stopped:
                stop("Camera disconnnected, not enough frames taken for calibration.", streams)
            else:
                frame = stream.read().copy()
                gridFinder.drawOutlines([frame]) # Small performance impact, displays previously found grids as green lines
                frames.append(frame)
        scaleForHconcat(frames[1], frames[0], 0.75)
        concatFrame = cv.hconcat([frames[0], frames[1]])
        # Indicates how many images remain to take in upper left
        cv.putText(concatFrame, 'Remaining images: ' + str(rectImageCount - gridFinder.len()), (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
        # Shows frame
        cv.imshow('Rectify', concatFrame)

    # Stereo calibration in OpenCV, using keypoints identified above by gridfinder
    # This obtains the R and T matrices to transform between 2 CAMERA POSITIONS, plus improved camera matrices and distortion coefficients
    # Uses previous calibration parameters for instrinsics (cv.CALIB_FIX_INTRINSIC)
    frameSize1 = frames[0, :, :, 0].shape[::-1]
    frameSize2 = frames[1, :, :, 0].shape[::-1]
    stereoSuccess, matrix1, dist1, matrix2, dist2, R, T, E, F = cv.stereoCalibrate( 
        gridFinder.objectPositions, gridFinder.allImagePositions[0], gridFinder.allImagePositions[1], 
        matrix1, dist1, matrix2, dist2, frameSize1, flags=cv.CALIB_FIX_INTRINSIC, criteria=criteria)
    
    if stereoSuccess:
        print("Stereo calibration successful.")
        # Alpha can be varied from 0 (no black pixels) to 1 (all pixels) in undistorted image
        # This obtains precise R and T matrices to transforms PIXELS between images
        R1, R2, P1, P2, Q, roi1, roi2 = cv.stereoRectify(matrix1, dist1, matrix2, dist2, frameSize1, R, T, alpha=1)
        # Calculates maps for undistorting each image into a rectified state
        mapX1, mapY1 = cv.initUndistortRectifyMap(matrix1, dist1, R1, P1, frameSize1, cv.CV_32FC1) #try CV_32FC2
        mapX2, mapY2 = cv.initUndistortRectifyMap(matrix2, dist2, R2, P2, frameSize2, cv.CV_32FC1) #try CV_32FC2

    # Displayed undistorted image for debugging, should be in another file!
    while True:
        frame1 = streams[0].read()
        frame2 = streams[1].read()
        # Maps pixels from FLIR to Kinect image
        frame2Remap = cv.remap(frame2, mapX2, mapY2, cv.INTER_LINEAR)

        # Scales images for display side-by-side
        scaleForHconcat(frame2Remap, frame1, 0.75)

        # Side-by-side display, requires both frames have same height
        concatFrame = cv.hconcat([frame1, frame2Remap])
        cv.imshow('Undistorted', concatFrame)

        # Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stop("User exited program.")
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', type=str, dest='cameraTypes', help='List of 2 cameras to open', required=True)
    parser.add_argument('--calib1', type=str, dest='calibFile1', help='First file to open', required=True)
    parser.add_argument('--calib2', type=str, dest='calibFile2', help='Second file to open', required=True)
    args = parser.parse_args().__dict__
    cameraType, file1, file2 = args["cameraTypes"], args["calibFile1"], args["calibFile2"]
    #Initialise circle detector
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H-%M")
    saveDirectory = os.path.join("Results", cameraType + "_" + dt_string)
    main(cameraType, saveDirectory)