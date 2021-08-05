import sys
import numpy as np
import cv2 as cv
import json

filename = str('calibrationFile.json')

if len(sys.argv) > 1:
    filename = str(sys.argv[1]) # First argv is undistort.py, second is filename

# Open calibration file and read contents
with open(filename, 'r') as file:
    calibrationData = json.load(file)
    cameraMatrix = np.asarray(calibrationData['CameraMatrix'])
    distortion = np.asarray(calibrationData['DistortionCoefficients'])

if not distortion.any():
    sys.exit("Cannot open complete calibration file.")

# Open camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    sys.exit("Cannot open camera.")

# Calculate optimal cropping for distortion using 1st frame
frameIsRead, tempframe = cap.read()
h, w = tempframe.shape[:2]
newCameraMatrix, croppingValues = cv.getOptimalNewCameraMatrix(cameraMatrix, distortion, (w,h), 1, (w,h))

while True:
    # Capture frame-by-frame
    frameIsRead, frame = cap.read()
    if not frameIsRead:
        print("Stream end.")
        break

    # Undo camera distorsion
    undistortedFrame = cv.undistort(frame, cameraMatrix, distortion, None, newCameraMatrix)

    # Crop image according to optimal new camera matrix
    x, y, w, h = croppingValues
    undistortedFrame = undistortedFrame[y:y+h, x:x+w] # Should be disabled for very distorted images or nothing will appear

    cv.imshow('Original', frame)
    cv.imshow('Undistorted', undistortedFrame)
    # Quits main loop if 'q' is pressed
    if cv.waitKey(1) == ord('q'):
        break

# Release the camera and window
cap.release()
cv.destroyAllWindows()