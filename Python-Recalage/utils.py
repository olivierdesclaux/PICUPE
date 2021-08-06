import cv2 as cv
import sys

# Convenience function for killing open objects
def stop(message, streams = []):
    cv.destroyAllWindows()
    for stream in streams:
        try:
            stream.stop()
        except: 
            pass
    sys.exit(str(message))

def scaleForHconcat(targetImage, referenceImage, scalingFactor = 1.0):
    # Scales images to have same height for hconcat side-by-side display
    # Preserves aspect ratio of target
    targetRatio =  targetImage.shape[1] / targetImage.shape[0]

    # Calculates new dimensions based on ratio, referenceImage height and a potential scaling factor
    referenceH, referenceW  = referenceImage.shape[0:2]
    targetDim = (int(referenceH * targetRatio * scalingFactor), int(referenceH * scalingFactor))
    referenceDim = (int(referenceW * scalingFactor), int(referenceH * scalingFactor))
    # Resizes images with calculated dimensions
    targetImage = cv.resize(targetImage, targetDim)
    referenceImage = cv.resize(referenceImage, referenceDim)
    # Returns but also modifies images in place anyways
    return targetImage, referenceImage