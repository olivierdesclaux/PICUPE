"""Quick script for testing camera opening/synchronisation issues
"""

import cv2 as cv
import videostream
from utils import scaleForHconcat, stop
import time
import numpy as np

def main():
    # Open camera by selection through letter
    cameras = "K"
    streams, _ = videostream.selectStreams(cameras, True)
    n = len(cameras)

    # Alternate code for opening a KinectVideoStream instead
    # stream = videostream.KinectVideoStream()
    t1 = time.time()
    f = []
    while True:
        frames = []
        for stream in streams:
            if stream.stopped:
                stop("Camera disconnected", streams)
            frame = stream.read()

            # frames.append(frame)
            frames.append(frame.color)
            # f.append(frame)
        if n == 2:
            left, right = frames
            right = right.color
            # Scales images for display side-by-side
            # left, right = scaleForHconcat(frames[0], frames[1], 0.7)
            left, right = scaleForHconcat(left, right, 0.7)

            # Side-by-side display, requires both frames have same height
            # concatFrame = cv.hconcat([left, right])
            cv.imshow('Side-by-side', left)
        elif n == 1:
            cv.imshow("Camera Test", frame)

        # Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            for stream in streams:
                stream.stop()
            break
    t2 = time.time()
    # print(len(gframes))
    t = t2 - t1
    print(t)
    print(len(frames))
    cntFlir = 0
    cntDepth = 0
    flirFrames = []
    for i, x in enumerate(frames):
        if i<=1:
            continue
        if type(x) == np.ndarray:
            if np.sum(x - f[i-n]) > 1:
                cntFlir += 1
                # flirFrames.append(x)
        else:
            d = x.transformed_depth
            if np.sum(d - f[i - n].transformed_depth) > 1:
                cntDepth += 1
                # flirDepth.append(x)

    print("FLIR ACQUISITON RATE: {} FPS".format(cntFlir/t))
    print("KINECT ACQUISITON RATE: {} FPS".format(cntDepth/t))


if __name__ == '__main__':
    main()
