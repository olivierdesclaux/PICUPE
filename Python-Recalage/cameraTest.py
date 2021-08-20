"""Quick script for testing camera opening/synchronisation issues
"""

import cv2 as cv
import videostream
from utils import scaleForHconcat, stop

# Open camera by selection through letter
cameras = 'K'
streams, _ = videostream.selectStreams(cameras)
n = len(cameras)

# Alternate code for opening a KinectVideoStream instead
#stream = videostream.KinectVideoStream()

while True:
    frames = []
    for stream in streams:
        # print(stream.stopped)
        if stream.stopped:
            stop("Camera disconnected", streams)
        frame = stream.read()
        frames.append(frame)

    if n == 2:
        # Scales images for display side-by-side
        left, right = scaleForHconcat(frames[0], frames[1], 0.7)

        # Side-by-side display, requires both frames have same height
        concatFrame = cv.hconcat([left, right])
        cv.imshow('Side-by-side', concatFrame)
    elif n == 1:
        cv.imshow("Camera Test", frames[0])

    # Waits for next frame or quits main loop if 'q' is pressed
    if cv.waitKey(1) == ord('q'):
        stop("User exited program.", streams)





