"""Quick script for testing camera opening/synchronisation issues
"""

import cv2
import videostream
from cameraUtils import scaleForHconcat, stop
import time
import numpy as np

def main():
    # Open camera by selection through letter
    cameras = "FK"
    streams, _ = videostream.selectStreams(cameras, True)
    n = len(cameras)
    COMPUTE_FPS = False

    t1 = time.time()
    f = []
    while True:
        frames = []
        for stream in streams:
            if stream.stopped:
                stop("Camera disconnected", streams)
            frame = stream.read()

            if cameras == "K":
                frames.append(frame.color)
            else:
                frames.append(frame)

            if COMPUTE_FPS:
                f.append(frame)

        if n == 2:
            left, right = frames
            right = right.color[:,:,:3]
            # Scales images for display side-by-side
            left, right = scaleForHconcat(left, right, 0.7)
            # Side-by-side display, requires both frames have same height
            concatFrame = cv2.hconcat([left, right])

            cv2.imshow('Side-by-side', concatFrame)
        elif n == 1:
            cv2.imshow("Camera Test", frames[0])

        # Waits for next frame or quits main loop if 'q' is pressed
        if cv2.waitKey(1) == ord('q'):
            for stream in streams:
                stream.stop()
            break

    t = time.time() - t1
    if COMPUTE_FPS:
        print(len(f))
        cntFlir = 0
        cntDepth = 0
        for i, x in enumerate(f):
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
