import sys
import cv2 as cv
import videohelper

flir = cv.VideoCapture(0)
flirStream = videohelper.VideoStream(flir)
fps = videohelper.FPS()

while True:
    if not flirStream.stopped:
        frame = flirStream.read()

        cv.putText(frame, str(fps.fps), (20, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (30, 30, 250), 2)
        redFrame = cv.subtract(frame[:,:,2], frame[:,:,0])

        cv.imshow('frame', frame)
        fps.frames += 1

        ## Waits 20 ms for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            flirStream.stop()
            cv.destroyAllWindows()
            fps.stop()
            sys.exit("User exited program.")
    else:
        fps.stop()
        cv.destroyAllWindows()
        sys.exit("Camera disconnected")

