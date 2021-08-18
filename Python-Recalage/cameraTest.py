import sys
import cv2 as cv
import videostream
from utils import scaleForHconcat

cameras = 'K'
streams, _ = videostream.selectStreams(cameras)

n = len(cameras)
if n ==1:
    streams = [streams]
#stream = videostream.KinectVideoStream()
# fps = videostream.FPS()

while True:
    if not(True in [stream.stopped for stream in streams]):
        frames = []
        for stream in streams:
            # print(stream.stopped)
            if stream.stopped:
                videostream.stop("Camera disconnected", streams)
            frame = stream.read()
            frames.append(frame)
            # Scales images for display side-by-side

        if n == 2:
            left, right = scaleForHconcat(frames[0], frames[1], 0.7)

            # Side-by-side display, requires both frames have same height
            concatFrame = cv.hconcat([left, right])
            cv.imshow('Side-by-side', concatFrame)
        elif n == 1:
            cv.imshow("Camera Test", frames[0])
        # Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            videostream.stop("User exited program.", streams)

# videostream.stop("Camera disconnected", streams)




