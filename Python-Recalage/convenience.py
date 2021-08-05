import cv2 as cv
import sys

# Convenience function for killing open objects
def stop(streams, message):
    cv.destroyAllWindows()
    for stream in streams:
        try:
            stream.stop()
        except: 
            pass
    sys.exit(str(message))