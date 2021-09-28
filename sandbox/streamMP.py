import sys
import os
import time
import cv2
from threading import Thread, Timer
import multiprocess as mp


# os.add_dll_directory(r"C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin")
# os.add_dll_directory(r"C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin")
import pyk4a as k4a
# from pyk4a import PyK4A

class KVS:
    def __init__(self):
        # Open Kinect with pyK4a for more options and depth mode
        self.kinect = k4a.PyK4A(k4a.Config(
            color_resolution=k4a.ColorResolution.RES_720P,
            depth_mode=k4a.DepthMode.NFOV_UNBINNED,
            camera_fps=k4a.FPS.FPS_30,
            synchronized_images_only=True,
        ))
        # Starts kinect and obtains first frame to fill attributes
        self.kinect.start()
        self.frame = self.kinect.get_capture()
        # Used to indicate if the thread should be stopped or not
        self.stopped = False
        self.process = None

    def run(self):
        p = mp.Process(target=self.update, args=())
        p.start()
        self.process = p

    def read(self):
        """ Returns latest frame taken in the thread
        """
        return self.frame

    def stop(self):
        """Stops frame-capturing thread and closes camera
        """
        # Stops thread
        self.stopped = True
        self.process.join()
        # Closes Kinect camera
        self.kinect.stop()

    def update(self):
        while True:
            if self.stopped:
                return
            else:
                # Grab next frame
                try:
                    self.frame = self.kinect.get_capture()
                except:
                    # Ends thread if get_capture() fails
                    self.stopped = True


def checker(oldImage, newImage, q):
    # if oldImage == newImage:
    #     return False
    # else:
    #     q.put(newImage)
    pass


def main():
    kinectStream = KVS()
    kinectStream.run()
    t1 = time.time()
    cnt = 0
    print("Recording...")

    while True:
        kinectFrame = kinectStream.read()
        if kinectFrame.color is not None:
            cv2.imshow("Color", kinectFrame.color)

        key = cv2.waitKey(1)
        if key == ord("q"):
            cv2.destroyAllWindows()
            break

        cnt += 1

    t2 = time.time()
    print("Acquisition time: ", round(t2 - t1, 4), " s")
    print("Number of frames: ", cnt)
    kinectStream.stop()
    cv2.destroyAllWindows()
    return True


if __name__ == '__main__':
    main()
