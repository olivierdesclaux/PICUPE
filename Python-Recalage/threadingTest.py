import sys
import cv2 as cv
import videohelper

camera = cv.VideoCapture(1, cv.CAP_DSHOW)
stream = videohelper.VideoStream(camera)
fps = videohelper.FPS()

while True:
    if not stream.stopped:
        frame = stream.read()
        
        cv.putText(frame, str(fps.fps), (20, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (20, 20, 250), 2)

        cv.imshow('frame', frame)
        fps.frames += 1

        ## Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stream.stop()
            cv.destroyAllWindows()
            fps.stop()
            sys.exit("User exited program.")
    else:
        fps.stop()
        cv.destroyAllWindows()
        sys.exit("Camera disconnected")

