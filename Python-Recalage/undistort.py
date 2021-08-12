import sys
import numpy as np
import cv2 as cv
import json
# Local modules
from videostream import openStream
from utils import stop, openCalibrationFile

filename = str('CalibrationFile.json')

if len(sys.argv) > 1:
    filename = str(sys.argv[1]) # First argv is undistort.py, second is filename, third is targetcamera
    targetCamera = int(sys.argv[2])

# Open calibration file and read contents
cameraMatrix, distortion = openCalibrationFile(filename)
if not distortion.any():
    stop("Cannot open complete calibration file.")

# Open camera
videoStream = openStream(targetCamera = targetCamera)

# Calculate optimal cropping for distortion using 1st frame
tempframe = videoStream.read()
h, w = tempframe.shape[:2]
newCameraMatrix, croppingValues = cv.getOptimalNewCameraMatrix(cameraMatrix, distortion, (w,h), 1, (w,h))

while not videoStream.stopped:
    # Capture frame-by-frame
    frame = videoStream.read()

    # Undo camera distorsion
    undistortedFrame = cv.undistort(frame, cameraMatrix, distortion, None, newCameraMatrix)

    # Crop image according to optimal new camera matrix
    x, y, w, h = croppingValues
    undistortedFrame = undistortedFrame[y:y+h, x:x+w] # Should be disabled for very distorted images or nothing will appear

    cv.imshow('Original', frame)
    cv.imshow('Undistorted', undistortedFrame)
    # Quits main loop if 'q' is pressed
    if cv.waitKey(1) == ord('q'):
        stop("User exit.", [videoStream])

# Release the camera and window
stop("Camera closed", [videoStream])