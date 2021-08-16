import json
import cv2 as cv
import sys
import json
import numpy as np
import argparse

# Convenience function for killing open objects
def stop(message, streams = []):
    cv.destroyAllWindows()
    for stream in streams:
        try:
            stream.stop()
        except: 
            pass
    sys.exit(str(message))

def openCalibrationFile(filename):
    with open(filename, 'r') as file:
        calibrationData = json.load(file)
        cameraMatrix = np.asarray(calibrationData['CameraMatrix'])
        distortion = np.asarray(calibrationData['DistortionCoefficients'])
        return cameraMatrix, distortion

def openRectFile(filename):
    with open(filename, 'r') as file:
        rectData = json.load(file)
        dists = [np.asarray(rectData['Dist1']), np.asarray(rectData['Dist2'])]
        matrices = [np.asarray(rectData['Matrix1']), np.asarray(rectData['Matrix2'])]
        Rs = [np.asarray(rectData['R1']), np.asarray(rectData['R2'])]
        Ps = [np.asarray(rectData['P1']), np.asarray(rectData['P2'])]
        rois = [np.asarray(rectData['ROI1']), np.asarray(rectData['ROI2'])]
        return matrices, dists, Rs, Ps, rois

def findingBoundingRectangle(frame):
    grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(grey, 1, 255, cv.THRESH_BINARY)
    nonZeroPoints = cv.findNonZero(thresh)
    return cv.boundingRect(nonZeroPoints)

def calculateOptimalMatrix(frame, cameraMatrix, distortion):
    h, w = frame.shape[:2]
    return cv.getOptimalNewCameraMatrix(cameraMatrix, distortion, (w,h), 1, (w,h))

def scaleForHconcat(referenceImage, targetImage, scalingFactor = 1.0):
    # Scales images to have same height for hconcat side-by-side display
    # Preserves aspect ratio of target
    targetRatio = targetImage.shape[1] / targetImage.shape[0]

    # Calculates new dimensions based on ratio, referenceImage height and a potential scaling factor
    referenceH, referenceW = referenceImage.shape[0:2]
    targetDim = (int(referenceH * targetRatio * scalingFactor), int(referenceH * scalingFactor))
    referenceDim = (int(referenceW * scalingFactor), int(referenceH * scalingFactor))
    # Resizes images with calculated dimensions
    resizedTargetImage = cv.resize(targetImage, targetDim)
    resizedReferenceImage = cv.resize(referenceImage, referenceDim)
    # Returns but also modifies images in place anyways
    return resizedReferenceImage, resizedTargetImage

# Encoder for Numpy arrays to JSON
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

# Used to allow argpase to accept "extend" action from python 3.8, so that multiple strings can be used with a single arg
class ExtendAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        items = getattr(namespace, self.dest) or []
        items.extend(values)
        setattr(namespace, self.dest, items)
