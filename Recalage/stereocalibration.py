import cv2.cv2 as cv2
import sys
import argparse
from datetime import datetime
import os
import json
# Local modules
from videostream import selectStreams
from utils import scaleForHconcat, stop, openCalibrationFile, NumpyEncoder, openStereoCalibrationFile
from circledetector import CircleDetector
from circlegridfinder import CircleGridFinder
from calibrate import calibrateCamera


def main(saveDirectory):
    # Start by calibrating the FLIR
    # Calibration results will be stored in the saveDirectory under CalibF.json
    flirCalib = calibrateCamera("F", saveDirectory)

    # Compute stereocalibration and extract stereocalibration file
    kinectCalib = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\CalibKinectFactory5Dist.json"
    stereoCalibrationFile = stereoCalibrate(cameraType, flirCalib, kinectCalib, saveDirectory)

    # Compute rectification matrices
    rectify(stereoCalibrationFile)

    pass


def stereoCalibrate(camerasToOpen, calibFile1, calibFile2, saveDirectory, initialNumGrids=20):
    """Rectify.py obtains extrinsic parameters between 2 cameras 

    Rectify.py uses a circle grid, just like calibrate.py. The grid must be
    presented to 2 cameras simulatenously, and the points obtained from
    each camera are used to calculate extrinsics. Also needed are previously 
    calculated intrinsic parameters for each camera, contained in calibFile1
    and calibFile2 in .json calibrate format. Results are saved in a .json 
    file.

    Parameters
    ----------
    camerasToOpen : {'KF', 'FK', 'WK', 'KW'}
        Type and order of cameras to rectify
        K = Kinect, F = Flir, W = Webcam
    calibFile1, calibFile2 : string
        Intrinsic camera parameters to use for first and second cameras
        Must have same order as camerasToOpen (1 first, 2 second)
        Must be .json files using CalibrationHandler.py format
        Directory in which to save the calibration matrices and error image
    initialNumGrids : int, default=20
        Number of grid images to obtain  before attempting rectification

    Returns
    -------
    None.
    """
    if len(camerasToOpen) != 2:
        stop("Only 2 cameras must be specified (K, F or W).")

    # Open calibration files and read contents
    try:
        matrixLeft, distLeft = openCalibrationFile(calibFile1)
        matrixRight, distRight = openCalibrationFile(calibFile2)
    except:
        stop("Unable to open calibration files.")

    # If matrices return empty, quit as well
    if not distLeft.any() or not distRight.any():
        sys.exit("Cannot read matrices from calibration files. \
                  Format may be incorrect.")

    # Opens 2 streams, depending on cameras specificed in args
    try:
        streams, types = selectStreams(camerasToOpen)
    # If invalid camera types
    except LookupError as err:
        sys.exit(err)

    # Termination criteria for iterative functions
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001)
    # Initialize blob detector with circle parameters
    circleDetector = CircleDetector()
    # Initialize grid finding object with both streams simultaneously
    gridFinder = CircleGridFinder(streams, types,
                                  [circleDetector, circleDetector], initialNumGrids)

    # Initial message to user
    print("Press q to exit program.")

    # Starts grid-finding in second thread, while this thread handles GUI
    gridFinder.start()
    while not gridFinder.finished:
        # Get next frame from both cameras
        frames = []
        for (index, stream) in enumerate(streams):
            if stream.stopped:
                stop("Camera" + str(index) + "disconnnected, not enough calibration images taken.", streams)
            else:
                frames.append(stream.read())
        # Copies frame to prevent modifying grid-finding frames
        frames = [x.copy() for x in frames]
        # Small performance impact, displays previously found grids
        gridFinder.drawOutlines(frames)
        # Resizing and concatenating for imshow
        resizedFrameLeft, resizedFrameRight = scaleForHconcat(frames[0], frames[1], 0.6)
        concatFrame = cv2.hconcat([resizedFrameLeft, resizedFrameRight])
        # Indicates how many images remain to take in upper left
        cv2.putText(concatFrame, 'Remaining images: ' + str(initialNumGrids - gridFinder.len()), (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
        # Shows frame
        cv2.imshow('Rectify', concatFrame)
        if cv2.waitKey(1) == ord('q'):
            stop("User exited program.", streams)

    cv2.destroyAllWindows()
    # Find frame size using .shape[::-1] of blue channel of individual frames
    frameSizeLeft = frames[0][:, :, 0].shape[::-1]
    frameSizeRight = frames[1][:, :, 0].shape[::-1]

    # Defining stereocalibration flags
    flags = cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_RATIONAL_MODEL
    # Crops to first 8 values, per Rational model
    distLeft = distLeft[:, 0:8]
    distRight = distRight[:, 0:8]

    # Stereo calibration in OpenCV, using keypoints identified by gridfinder
    # This obtains the R and T matrices to transform between 2 CAMERA POSITIONS
    # Uses previous calibration parameters for instrinsics 
    # (cv2.CALIB_FIX_INTRINSIC) and does not modify them
    (retError, matrixLeft, distLeft, matrixRight, distRight, R, T, E, F) = cv2.stereoCalibrate(
        gridFinder.objectPositions, gridFinder.allImagePositions[0],
        gridFinder.allImagePositions[1], matrixLeft, distLeft, matrixRight, distRight, frameSizeLeft,
        flags=flags, criteria=criteria)

    # Calibration successful:
    if not retError:
        raise Exception("Stereo calibration failed")
    else:
        print("Stereo Calibration retroprojection error: ", retError)
        print("E: ", E)
        print("F: ", F)
        print("R: ", R)
        print("T: ", T)
        if not os.path.isdir(saveDirectory):
            os.mkdir(saveDirectory)
        filename = os.path.join(saveDirectory, "StereoCalibrate-" + camerasToOpen + ".json")
        with open(filename, 'w') as file:
            # Assigns labels to values to make JSON readable
            dumpDictionary = {'Format': 'OpenCV', 'Matrix1': matrixLeft,
                              'Matrix2': matrixRight, 'Dist1': distLeft, 'Dist2': distRight,
                              'R': R, 'T': T, 'E': E, 'F': F, 'frameSize': frameSizeLeft}
            # Uses NumpyEncoder to convert numpy values
            # to regular arrays for json.dump
            json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)

    # Stop streams
    for stream in streams:
        stream.stop()

    return filename


def rectify(stereoCalibrationFile):
    matrixLeft, distLeft, matrixRight, distRight, frameSizeLeft, R, T = openStereoCalibrationFile(stereoCalibrationFile)
    # Alpha can be varied from 0 (no black pixels)
    # to 1 (all pixels) in undistorted image
    # This returns R, T matrices to transforms PIXELS between images
    # RLeft and RRight rotate the cameras to be coplanar and row-aligned
    # Pleft and Pright are "new" camera matrices that also project points
    # onto the left image plane
    # flags = 0 sets the converging point for the cameras at a finite
    # distance (which?) but flags = cv2.CALIB_ZERO_DISPARITY
    # sets the converging point at infinity
    RLeft, RRight, PLeft, PRight, Q, roiLeft, roiRight = cv2.stereoRectify(
        matrixLeft, distLeft, matrixRight, distRight, frameSizeLeft,
        R, T, alpha=0, flags=cv2.CALIB_ZERO_DISPARITY)
    print("PLeft", PLeft)
    print("PRight", PRight)
    print("roiLeft", roiLeft)
    print("roiRight", roiRight)

    # Creates filepath for results of calibration
    if not os.path.isdir(saveDirectory):
        os.mkdir(saveDirectory)
    filename = os.path.join(
        saveDirectory, "Rectify" + camerasToOpen + ".json")
    # Outputs calibration matrices to file
    with open(filename, 'w') as file:
        # Assigns labels to values to make JSON readable
        dumpDictionary = {'Format': 'OpenCV', 'Matrix1': matrixLeft,
                          'Matrix2': matrixRight, 'Dist1': distLeft, 'Dist2': distRight,
                          'R1': RLeft, 'R2': RRight, 'P1': PLeft, 'P2': PRight,
                          'ROI1': roiLeft, 'ROI2': roiRight}
        # Uses NumpyEncoder to convert numpy values
        # to regular arrays for json.dump
        json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)
        print("Succesfully wrote calibration to file.")


# Standard argument parser implementation for command-line use
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Asks for cameras and both calibration files
    parser.add_argument('-c', type=str, dest='camerasToOpen',
                        help='String of 2 cameras (K, F or W) to open', required=True)
    parser.add_argument('--calib1', type=str, dest='calibFile1', default='Results/CalibFLIR.json',
                        help='First file to open', required=False)
    parser.add_argument('--calib2', type=str, dest='calibFile2', default='Results/CalibKinectFactory.json',
                        help='Second file to open', required=False)
    args = parser.parse_args().__dict__
    cameraType = args["camerasToOpen"]
    file1 = args["calibFile1"]
    file2 = args["calibFile2"]
    # Get current time and create Results path from time
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H-%M")
    saveDirectory = os.path.join("Results", dt_string + "_" + cameraType)
    main(cameraType, file1, file2, saveDirectory)