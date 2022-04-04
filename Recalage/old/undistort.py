"""Script for quickly displaying results of a calibration

See remap.py for rectification display
"""

import sys
import cv2 as cv
# Local modules
from videostream import openStream
from cameraUtils import stop, openCalibrationFile

# File with calibration matrices
# Format should be identical to calibrate.py
filename = str('CalibrationFile.json')

# If there are arguments, use the first one as camera flag
# And the second one as a filename
if len(sys.argv) > 1:
    targetCamera = int(sys.argv[1])
if len(sys.argv) > 2:
    filename = str(sys.argv[2])

# Open calibration file and read contents
cameraMatrix, distortion = openCalibrationFile(filename)
if not distortion.any():
    stop("Cannot open complete calibration file.")

# Open camera
videoStream = openStream(targetCamera = targetCamera)

# Calculate optimal cropping for distortion using 1st frame dimensions
tempframe = videoStream.read()
h, w = tempframe.shape[:2]
newCameraMatrix, croppingValues = cv.getOptimalNewCameraMatrix(
    cameraMatrix, distortion, (w,h), 1, (w,h))

while not videoStream.stopped:
    # Capture frame-by-frame
    frame = videoStream.read()

    # Undo camera distorsion
    undistortedFrame = cv.undistort(
        frame, cameraMatrix, distortion, None, newCameraMatrix)

    # Crop image according to optimal new camera matrix
    # Should be disabled for very distorted images or nothing will appear
    x, y, w, h = croppingValues
    undistortedFrame = undistortedFrame[y:y+h, x:x+w] 

    # Show original and new images in 2 separate windows
    cv.imshow('Original', frame)
    cv.imshow('Undistorted', undistortedFrame)

    # Quits main loop if 'q' is pressed
    if cv.waitKey(1) == ord('q'):
        stop("User exit.", [videoStream])

# Release the camera and window
stop("Camera closed", [videoStream])