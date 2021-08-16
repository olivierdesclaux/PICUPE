import numpy as np
import cv2 as cv
import argparse
# Local modules
from videostream import selectStreams
from utils import stop,  openRectFile, scaleForHconcat, findingBoundingRectangle

def main(cameras, rectFile):
    # Opens specified rect file
    matrices, dists, Rs, Ps, rois = openRectFile(rectFile)
    # Opens specified cameras
    streams, _ = selectStreams(cameras)
    # Gets camera sizes from first frames
    frameSizes = []
    for stream in streams:
        frame = stream.read()
        frameSizes.append(frame[:, :, 0].shape[::-1])
    # Generates maps from rect informaiton
    maps = []
    for (matrix, dist, R, P, frameSize) in zip(matrices, dists, Rs, Ps, frameSizes):
        maps.append(cv.initUndistortRectifyMap(matrix, dist, R, P, frameSize, cv.CV_32FC1))

    # Calculates cropping boxes using rectified frames
    crops = []
    for (stream, map) in zip(streams, maps):
        if stream.stopped:
            stop("Camera disconnected", streams)
        frame = stream.read()
        frame = cv.remap(frame, map[0], map[1], cv.INTER_LANCZOS4, borderMode=cv.BORDER_TRANSPARENT)
        crops.append(findingBoundingRectangle(frame))
        
    # Displayed undistorted images
    while not streams[0].stopped and not streams[1].stopped:
        remappedFrames = []

        for (stream, map, crop) in zip(streams, maps, crops):
            if stream.stopped:
                stop("Camera disconnected", streams)
            frame = stream.read()
            # Remaps frame, using a transparent border for overlay
            frame = cv.remap(frame, map[0], map[1], cv.INTER_LANCZOS4, borderMode=cv.BORDER_TRANSPARENT)
            # Crops frame to remove unnecessary black
            frame = frame[crop[1]:crop[1]+crop[3], crop[0]:crop[0]+crop[2]]
            remappedFrames.append(frame)

        # Manually position thermal frame on Kinect frame using x and y differences
        alpha = 0.5
        xDiff = 320
        yDiff = 150
        crop = crops[1]
        overlayFrame = remappedFrames[0].copy()
        overlayFrame[yDiff:yDiff+crop[3], xDiff:xDiff+crop[2]] = alpha*overlayFrame[yDiff:yDiff+crop[3], xDiff:xDiff+crop[2]] + (1-alpha) * remappedFrames[1]
        cv.imshow('Overlay', overlayFrame)

        # Scales images for display side-by-side
        left, right = scaleForHconcat(remappedFrames[0], remappedFrames[1], 0.7)

        # Side-by-side display, requires both frames have same height
        concatFrame = cv.hconcat([left, right])
        cv.imshow('Side-by-side', concatFrame)

        # Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stop("User exited program.", streams)
    else:
        stop("Camera disconnected", streams)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Remap images from several cameras using stored .json maps.")
    parser.add_argument('-c', type=str, dest='cameras', help='List of cameras to open', required=True)
    parser.add_argument('--rectFile', type=str, dest='file', help='Rectification file to open', required=True)
    args = parser.parse_args().__dict__
    cameras, file = args["cameras"], args["file"]

    main(cameras, file)