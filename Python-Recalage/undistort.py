import sys
import numpy as np
import cv2 as cv
import json
# Local modules
from videostream import openStream
from utils import stop

filename = str('CalibrationFileWebcam.json')

if len(sys.argv) > 1:
    filename = str(sys.argv[1]) # First argv is undistort.py, second is filename

# Open calibration file and read contents
with open(filename, 'r') as file:
    calibrationData = json.load(file)
    cameraMatrix = np.asarray(calibrationData['CameraMatrix'])
    distortion = np.asarray(calibrationData['DistortionCoefficients'])

if not distortion.any():
    stop("Cannot open complete calibration file.")

# Open camera
[videoStream] = openStream(targetCameras=[0])

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