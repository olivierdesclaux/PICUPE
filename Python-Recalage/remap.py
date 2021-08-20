import cv2 as cv
import argparse
# Local modules
from videostream import selectStreams
from utils import stop, openRectFile, scaleForHconcat, findBoundingRectangle

def main(cameras, rectFile):
    """Remap.py uses calculated intrinsic and extrinsic camera parameters to 
    rectify 2 cameras

    Function calculates map in-place because map files are very large (50 MB). 
    Images are displayed superposed and side-by-side, but superposition code 
    does not work very well.

    Parameters
    ----------
    cameras : string, length=2
        Cameras to open for remapping
        Must be in same order as matrices in rectFile
    rectFile : string
        Path to file with instrinsic and extrinsic matrices
        Must use format shown in rectify.py, or openRectFile() will fail.

    Returns
    -------
    None.
    """
    # Opens specified rect file
    matrices, dists, Rs, Ps, rois = openRectFile(rectFile)
    # Opens specified cameras (must be 2)
    streams, _ = selectStreams(cameras[0:2], useKVS=True)
    # Gets camera sizes from first frames
    frameSizes = []
    for stream in streams:
        frame = stream.read()
        frameSizes.append(frame[:, :, 0].shape[::-1])

    # Generates maps from rect informaiton
    maps = []
    for (matrix, dist, R, P, frameSize) in zip(
            matrices, dists, Rs, Ps, frameSizes):
        maps.append(cv.initUndistortRectifyMap(
            matrix, dist, R, P, frameSize, cv.CV_32FC1))

    # Needed for crop calculation later
    # Crops are used to remove black from frames
    crops = [None] * len(streams)
        
    # Displays rectified images
    while True:
        remappedFrames = []
        # Remaps each stream individually
        for (stream, map, crop) in zip(streams, maps, crops):
            if stream.stopped:
                stop("Camera disconnected", streams)
            frame = stream.read()
            # Remaps frame
            # cv.INTER_LANCZOS4 is good, can be changed for linear
            # borderMode does not seem to have an effect
            frame = cv.remap(
                frame, map[0], map[1], cv.INTER_LANCZOS4, 
                borderMode = cv.BORDER_TRANSPARENT)

            # Calculates cropping boxes using rectified frames 
            # Only performed once (with first remapped frame)
            if crop is None:
                crop = findBoundingRectangle(frame)

            # Crops frame to remove unnecessary black
            frame = frame[crop[1] : crop[1]+crop[3], crop[0] : crop[0]+crop[2]]
            remappedFrames.append(frame)

        # Superpostion display code
        # Values used to manually position thermal frame on Kinect frame
        # using x and y differences, alpha controls mixing
        alpha = 0.5
        xDiff = 320
        yDiff = 150
        crop = crops[1]
        overlayFrame = remappedFrames[0].copy()
        overlayFrame[yDiff : yDiff+crop[3], xDiff : xDiff+crop[2]] = \
            alpha * overlayFrame[yDiff : yDiff+crop[3], xDiff : xDiff+crop[2]]\
            + (1 - alpha) * remappedFrames[1]
        cv.imshow('Overlay', overlayFrame)

        # Side-by-side display code
        # Scales images for display side-by-side
        left, right = scaleForHconcat(
            remappedFrames[0], remappedFrames[1], 0.7)
        # Horizontal display, requires both frames have same height
        concatFrame = cv.hconcat([left, right])
        cv.imshow('Side-by-side', concatFrame)

        # Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stop("User exited program.", streams)

# Standard argument parser implementation for command-line use
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description = "Remap images from 2 cameras using stored .json maps.")
    # Used to select cameras to open, must be of length 2
    parser.add_argument(
        '-c', type=str, dest='cameras', 
        help='List of cameras to open', required=True)
    parser.add_argument(
        '--rectFile', type=str, dest='file', 
        help='Rectification file to open', required=True)
    args = parser.parse_args().__dict__
    cameras, file = args["cameras"], args["file"]

    main(cameras, file)