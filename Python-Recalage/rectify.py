import cv2 as cv
import numpy as np
import sys
import time
import argparse
from datetime import datetime
import os
# Local modules
from videostream import openStream, StreamType
from utils import scaleForHconcat, stop, openCalibrationFile, calculateOptimalMatrix
from circledetector import CircleDetector
from circlegridfinder import CircleGridFinder


def main(cameraTypes, calibFileLeft, calibFileRight):
    # Open calibration files and read contents
    matrixLeft, distLeft = openCalibrationFile(calibFileLeft)
    matrixRight, distRight = openCalibrationFile(calibFileRight)

    # If empty, quit
    if not distLeft.any():
        sys.exit("Cannot read matrices from Kinect calibration file.")
    if not distRight.any():
        sys.exit("Cannot read matrices from FLIR calibration file.")

    streams = []
    types = []
    try:
        for letter in cameraTypes:
            if letter == "K":
                streams.append(openStream(targetHeight = 720))
                types.append(StreamType.rgb)
            elif letter == "F":
                streams.append(openStream(targetHeight = 768))
                types.append(StreamType.ir)
            elif letter == "W":
                streams.append(openStream(targetCamera = 0, flag = "W"))
                types.append(StreamType.rgb)
            else:
                raise Exception("Invalid camera types selected.")
    except Exception as err:
        stop(err, streams)

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
                frames.append(stream.read())

        frames = [x.copy() for x in frames]
        gridFinder.drawCircles(frames)
        gridFinder.drawOutlines(frames) # Small performance impact, displays previously found grids as green lines
        # Resizing and concatenating for imshow
        resizedFrameLeft, resizedFrameRight = scaleForHconcat(frames[0], frames[1], 0.6)
        concatFrame = cv.hconcat([resizedFrameLeft, resizedFrameRight])
        # Indicates how many images remain to take in upper left
        cv.putText(concatFrame, 'Remaining images: ' + str(rectImageCount - gridFinder.len()), (10, 50),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
        # Shows frame
        cv.imshow('Rectify', concatFrame)
        if cv.waitKey(1) == ord('q'):
            stop("User exited program.", streams)

    cv.destroyAllWindows()
    # Find frame size using .shape[::-1] of blue channel of individual frames
    frameSizeLeft = frames[0][:, :, 0].shape[::-1]
    frameSizeRight = frames[1][:, :, 0].shape[::-1]

    # Stereo calibration in OpenCV, using keypoints identified above by gridfinder
    # This obtains the R and T matrices to transform between 2 CAMERA POSITIONS, plus improved camera matrices and distortion coefficients
    # Uses previous calibration parameters for instrinsics (cv.CALIB_FIX_INTRINSIC)
    retError, matrixLeft, distLeft, matrixRight, distRight, R, T, E, F = cv.stereoCalibrate(
        gridFinder.objectPositions, gridFinder.allImagePositions[0], gridFinder.allImagePositions[1],
        matrixLeft, distLeft, matrixRight, distRight, frameSizeLeft, flags= cv.CALIB_FIX_INTRINSIC + cv.CALIB_RATIONAL_MODEL, criteria=criteria)

    distLeft = distLeft[:,0:8]
    distRight = distRight[:,0:8]

    # Calibration successful:
    if retError:
        print(retError)
        print("E: ", E)
        print("F: ", F)
        print("R: ", R)
        print("T: ", T)
        # Alpha can be varied from 0 (no black pixels) to 1 (all pixels) in undistorted image
        # This obtains precise R and T matrices to transforms PIXELS between images
        RLeft, RRight, PLeft, PRight, Q, roiLeft, roiRight = cv.stereoRectify(matrixLeft, distLeft, matrixRight, distRight, frameSizeLeft, R, T, alpha=1)
        # Calculates maps for undistorting each image into a rectified state
        mapXLeft, mapYLeft = cv.initUndistortRectifyMap(matrixLeft, distLeft, RLeft, PLeft, frameSizeLeft, cv.CV_32FC1)
        mapXRight, mapYRight = cv.initUndistortRectifyMap(matrixRight, distRight, RRight, PRight, frameSizeRight, cv.CV_32FC1)
        print("MAPXLEFT")
        print(mapXLeft)
        print("MAPYXRIGHT")
        print(mapXRight)

    else:
        raise Exception("Stereo calibration failed")

    # Displayed undistorted image for debugging, should be in another file!
    while not streams[0].stopped and not streams[1].stopped:
        frameLeft = streams[0].read()
        frameRight = streams[1].read()

        # Remap both views
        frameLeftRemap = cv.remap(frameLeft, mapXLeft, mapYLeft, cv.INTER_LINEAR, cv.BORDER_CONSTANT, 0)
        frameRightRemap = cv.remap(frameRight, mapXRight, mapYRight, cv.INTER_LINEAR, cv.BORDER_CONSTANT, 0)

        # Scales images for display side-by-side
        resizedFrameLeftRemap, resizedFrameRightRemap = scaleForHconcat(frameLeftRemap, frameRightRemap, 0.5)

        # Side-by-side display, requires both frames have same height
        concatFrame = cv.hconcat([resizedFrameLeftRemap, resizedFrameRightRemap])
        cv.imshow('Undistorted', concatFrame)

        # Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stop("User exited program.", streams)
    else:
        stop("Camera disconnected", streams)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', type=str, dest='cameraTypes', help='List of 2 cameras to open', required=True)
    parser.add_argument('--calibL', type=str, dest='calibFileLeft', help='First file to open', required=True)
    parser.add_argument('--calibR', type=str, dest='calibFileRight', help='Second file to open', required=True)
    args = parser.parse_args().__dict__
    cameraType, fileLeft, fileRight = args["cameraTypes"], args["calibFileLeft"], args["calibFileRight"]
    # Initialise circle detector
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H-%M")
    saveDirectory = os.path.join("Results", cameraType + "_" + dt_string)
    main(cameraType, fileLeft, fileRight)
