import json
import cv2 as cv
import sys
import json
import numpy as np

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
    try:
        with open(filename, 'r') as file:
            calibrationData = json.load(file)
            cameraMatrix = np.asarray(calibrationData['CameraMatrix'])
            distortion = np.asarray(calibrationData['DistortionCoefficients'])
            return cameraMatrix, distortion
    except:
        stop("Unable to open calibration files.")

def scaleForHconcat(targetImage, referenceImage, scalingFactor = 1.0):
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
    return resizedTargetImage, resizedReferenceImage