import cv2 as cv
import numpy as np
import json
import sys
import time
# Local modules
import videostream
from convenience import stop
from circledetector import CircleDetector

# Default filename
kFilename = str('CalibrationFileKinect.json')
fFilename = str('CalibrationFileFLIR.json')

if len(sys.argv) > 1:
    kFilename = str(sys.argv[1]) # First argv is undistort.py, second is kinect filename
    fFilename = str(sys.argv[2]) # Third is flir filename

def openCalibrationFile(filename):
    try:
        with open(filename, 'r') as file:
            calibrationData = json.load(file)
            cameraMatrix = np.asarray(calibrationData['CameraMatrix'])
            distortion = np.asarray(calibrationData['DistortionCoefficients'])
            return cameraMatrix, distortion
    except:
        stop("Unable to open calibration files.")

# Open calibration files and read contents
kMatrix, kDist = openCalibrationFile(kFilename)
fMatrix, fDist = openCalibrationFile(fFilename)

# If empty, quit
if not kDist.any():
    sys.exit("Cannot read matrices from Kinect calibration file.")
if not fDist.any():
    sys.exit("Cannot read matrices from FLIR calibration file.")

# Open video streams from cameras
streams = [flirStream, kinectStream] = videostream.openStream([480, 720]) # Only for webcam
#[flirStream, kinectStream] = videostream.openStream([768, 720])
if None in streams:
    stop("Could not open both cameras.")

## Program global variables
# Termination criteria : 1st parameter is flag, 2nd is maximum iterations, 3rd is maximum epsilon
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 0.0001)
# Corner finder variables
numberOfRows = 12
numberOfColumns = 12
totalCircles = numberOfRows*numberOfColumns
checkerboardSize = (numberOfRows, numberOfColumns)

# Array of float32 points
# Last axis (z) is always 0
# Y axis (rows) increments 0, 1, 2, ... numberOfRows-1
# X axis (columns) increments 0, 1, 2, ... numberOfColumns-1
objectCorners = np.array([[x % numberOfColumns, np.floor(x / numberOfColumns), 0] for x in range(totalCircles)], np.float32)
# Arrays to store object points and image points from all the images.
objectPositions = [] #3d points in real world space
imagePositionsBoth = [[],[]] # 2d points in image plane, one for each camera
# Initialize blob detectors with circle parameters for kinect and FLIR
kCircleDetector = CircleDetector()
#fCircleDetector = CircleDetector(threshold = (40, 180, 20))
fCircleDetector = CircleDetector(minArea = 15)

# Skips X seconds between each checkerboard to increase variability of calibration images
secondsToSkip = 2
lastCheckerboardTime = time.time()
# Number of image to take for rectification
rectImageCount = 25

# Initial message to user
print("Press q to exit program.")
while len(objectPositions) < rectImageCount:
    # Get next frame from both cameras
    if kinectStream.stopped or flirStream.stopped:
        stop([flirStream, kinectStream], "Camera disconnnected, not enough frames taken for calibration.")
    else:
        kinectFrame = kinectStream.read()
        flirFrame = flirStream.read()
# Stereo calibration in OpenCV, using keypoints identified previously
# This obtains the R and T matrices to transform between 2 CAMERA POSITIONS, plus improved camera matrices and distortion coefficients
# Uses previous calibration parameters as a starting point (CALIB_USE_INSTRINSIC_GUESS)
stereoSuccess, kMatrix, kDist, fMatrix, fDist, R, T, E, F = cv.stereoCalibrate( 
    objectPositions, imagePositionsBoth[0], imagePositionsBoth[1], 
    kMatrix, kDist, fMatrix, fDist,
    kSize, flags=cv.CALIB_FIX_INTRINSIC, criteria=criteria)
if stereoSuccess:
    print("Stereo calibration successful")
    ### TEMP CODE ###
    h, w = kinectFrame.shape[:2]
    newKMatrix, croppingValues = cv.getOptimalNewCameraMatrix(kMatrix, kDist, (w,h), 1, (w,h))
    undistortedK = cv.undistort(kinectFrame, kMatrix, kDist, None, newKMatrix)
    cv.imshow('UndistortedK', undistortedK)
    h, w = flirFrame.shape[:2]
    newFMatrix, croppingValues = cv.getOptimalNewCameraMatrix(fMatrix, fDist, (w,h), 1, (w,h))
    undistortedF = cv.undistort(flirFrame, fMatrix, fDist, None, newFMatrix)
    cv.imshow('UndistortedF', undistortedF)
    # Alpha can be varied from 0 (no black pixels) to 1 (all pixels) in undistorted image
    # This obtains precise R and T matrices to transforms PIXELS between images
    R1, R2, P1, P2, Q, roi1, roi2 = cv.stereoRectify(kMatrix, kDist, fMatrix, fDist, kSize, R, T, alpha=1)
    # Calculates maps for undistorting each image into a rectified state
    kMapx, kMapy = cv.initUndistortRectifyMap(kMatrix, kDist, R1, P1, kSize, cv.CV_32FC1) #try CV_32FC2
    fMapx, fMapy = cv.initUndistortRectifyMap(fMatrix, fDist, R1, P1, fSize, cv.CV_32FC1) #try CV_32FC2

# Displayed undistorted image for debugging, should be in another file!
if cv.waitKey(25000) == 'q':
    pass
while True:
    kinectFrame = kinectStream.read()
    flirFrame = flirStream.read()
    # Maps pixels from FLIR to Kinect image
    flirRemapped = cv.remap(flirFrame, fMapx, fMapy, cv.INTER_LINEAR)

    # Scales FLIR image to same height as Kinect for display side-by-side
    flirRatio =  flirRemapped.shape[1] / flirRemapped.shape[0]
    scalingFactor = 1.5 # Reduces overall window size to fit on monitor
    kHeight, kWidth  = kinectFrame.shape[0:2]
    fDim = (int(kHeight*flirRatio/scalingFactor), int(kHeight/scalingFactor))
    kDim = (int(kWidth/scalingFactor), int(kHeight/scalingFactor))
    flirRemapped = cv.resize(flirRemapped, fDim)
    kinectFrame = cv.resize(kinectFrame, kDim)
    # Side-by-side display, requires both frames have same height
    concatenatedFrames = cv.hconcat([kinectFrame, flirRemapped])
    cv.imshow('Undistorted', concatenatedFrames)

    # Waits for next frame or quits main loop if 'q' is pressed
    if cv.waitKey(1) == ord('q'):
        stop("User exited program.")
