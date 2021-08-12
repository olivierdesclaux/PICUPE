import cv2 as cv
import numpy as np
import sys
import argparse
from datetime import datetime
import os
import json
# Local modules
from videostream import StreamType, selectStreams
from utils import scaleForHconcat, stop, openCalibrationFile, NumpyEncoder
from circledetector import CircleDetector
from circlegridfinder import CircleGridFinder


def main(camerasToOpen, calibFileLeft, calibFileRight, saveDirectory):
    if len(camerasToOpen) != 2:
        stop("Only 2 cameras must be specified (K, F or W).")

    # Open calibration files and read contents
    try:
        matrixLeft, distLeft = openCalibrationFile(calibFileLeft)
        matrixRight, distRight = openCalibrationFile(calibFileRight)
    except:
        stop("Unable to open calibration files.")

    # If empty, quit
    if not distLeft.any():
        sys.exit("Cannot read matrices from Kinect calibration file.")
    if not distRight.any():
        sys.exit("Cannot read matrices from FLIR calibration file.")

    # Opens 2 streams, depending on cameras specificed in args
    try:
        streams, types = selectStreams(camerasToOpen)
    except Exception as err:
        stop(err)

    # Number of images to take for rectification
    rectImageCount = 20
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
        for (index, stream) in enumerate(streams):
            if stream.stopped:
                stop("Camera" + str(index) + "disconnnected, not enough frames taken for calibration.", streams)
            else:
                frames.append(stream.read())

        frames = [x.copy() for x in frames]
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
    # This obtains the R and T matrices to transform between 2 CAMERA POSITIONS
    # Uses previous calibration parameters for instrinsics (cv.CALIB_FIX_INTRINSIC) and does not modify them
    retError, matrixLeft, distLeft, matrixRight, distRight, R, T, E, F = cv.stereoCalibrate(
        gridFinder.objectPositions, gridFinder.allImagePositions[0], gridFinder.allImagePositions[1],
        matrixLeft, distLeft, matrixRight, distRight, frameSizeLeft, flags= cv.CALIB_FIX_INTRINSIC + cv.CALIB_RATIONAL_MODEL, criteria=criteria)
    # Crops to first 8 values, per Rational model
    distLeft = distLeft[:,0:8]
    distRight = distRight[:,0:8]

    # Calibration successful:
    if not retError:
        raise Exception("Stereo calibration failed")
    else:
        print(retError)
        print("E: ", E)
        print("F: ", F)
        print("R: ", R)
        print("T: ", T)
        # Alpha can be varied from 0 (no black pixels) to 1 (all pixels) in undistorted image
        # This obtains precise R and T matrices to transforms PIXELS between images
        # RLeft and RRight rotate the cameras to be coplanar and row-aligned
        # Pleft and Pright are "new" camera matrices that also project points onto the left image plane
        # flags = 0 sets the converging point for the cameras at a finite distance (which?) but flags = cv.CALIB_ZERO_DISPARITY sets the converging point at infinity
        RLeft, RRight, PLeft, PRight, Q, roiLeft, roiRight = cv.stereoRectify(matrixLeft, distLeft, matrixRight, distRight, frameSizeLeft, R, T, alpha=1, flags = cv.CALIB_ZERO_DISPARITY)
        print("PLeft", PLeft)        
        print("PRight", PRight)        
        print("roiLeft")
        print(roiLeft)
        print("roiRight")
        print(roiRight)

        # Creates filepath for results of calibration
        if not os.path.isdir(saveDirectory):
            os.mkdir(saveDirectory)
        filename = os.path.join(saveDirectory, "Rectify" + camerasToOpen + ".json")
        # Outputs calibration matrices to file
        with open(filename, 'w') as file:
            # Assigns labels to values to make JSON readable
            dumpDictionary = {'Format' : 'OpenCV', 'Matrix1':matrixLeft, 'Matrix2':matrixRight, 'Dist1': distLeft, 'Dist2': distRight, 
                                'R1' : RLeft, 'R2': RRight, 'P1': PLeft, 'P2': PRight, 'ROI1': roiLeft, 'ROI2': roiRight}
            # Uses NumpyEncoder to convert numpy values to regular arrays for json.dump
            json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)
            print("Succesfully wrote calibration to file.")
        
    stop("", streams)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Asks for cameras and both calibration files
    parser.add_argument('-c', type=str, dest='camerasToOpen', help='String of 2 cameras (K, F or W) to open', required=True)
    parser.add_argument('--calibL', type=str, dest='calibFileLeft', help='First file to open', required=True)
    parser.add_argument('--calibR', type=str, dest='calibFileRight', help='Second file to open', required=True)
    args = parser.parse_args().__dict__
    cameraType, fileLeft, fileRight = args["camerasToOpen"], args["calibFileLeft"], args["calibFileRight"]
    # Get current time and create Results path from time
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H-%M")
    saveDirectory = os.path.join("Results", cameraType + "_" + dt_string)
    main(cameraType, fileLeft, fileRight, saveDirectory)
