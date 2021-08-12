import sys
import numpy as np
import cv2 as cv
import argparse
# Local modules
from videostream import selectStreams
from utils import stop,  openRectFile, scaleForHconcat, findingBoundingRectangle

def main(cameras, rectFile):
    # Opens specified rect file
    matrices, dists, Rs, Ps, rois = openRectFile(rectFile)
    mainRoi = rois[0]
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

    # Displayed undistorted images
    while not streams[0].stopped and not streams[1].stopped:
        remappedFrames = []
        for (stream, map) in zip(streams, maps):
            if stream.stopped:
                stop("Camera disconnected", streams)
            frame = stream.read()
            # Remaps frame, using a transparent border for overlay
            frame = cv.remap(frame, map[0], map[1], cv.INTER_LANCZOS4, borderMode=cv.BORDER_TRANSPARENT)
            print(findingBoundingRectangle(frame))
            # Crops frame using ROI
            remappedFrames.append(frame)

        remappedFrames[0] = remappedFrames[0][mainRoi[1]:mainRoi[1]+mainRoi[3], mainRoi[0]:mainRoi[0]+mainRoi[2]]
        cv.imshow('Kinect', remappedFrames[0])
        #remappedFrames[1] = remappedFrames[1][roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
        cv.imshow('FLIR', remappedFrames[1])

        overlayed = cv.addWeighted(remappedFrames[0], 0.5, remappedFrames[1], 0.5, 0)
        cv.imshow('Overlay', overlayed)

        # Scales images for display side-by-side
        #left, right = scaleForHconcat(remappedFrames[0], remappedFrames[1], 0.7)

        # Side-by-side display, requires both frames have same height
        #concatFrame = cv.hconcat([left, right])
        #cv.imshow('Remapped', concatFrame)

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