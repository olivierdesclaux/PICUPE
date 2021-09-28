import sys
import os
import cv2
import threading
import time
from datetime import datetime
from PIL import Image
import msvcrt
import xsensdeviceapi as xda

sys.path.append('./Python-Recalage')
sys.path.append('./XSens')

import videostream
import vsmp
import extractDepth
import MTwReceiveData as mtw


class ImageSaver:
    def __init__(self, mode, savePath):
        self.mode = mode
        self.savePath = savePath
        threading.Thread(target=self.save, args=(), daemon=True).start()

    def save(self, image, name):
        if self.mode == "ir":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif self.mode == 'depth':
            image = extractDepth.colorize(image, (None, 5000), cv2.COLORMAP_BONE)
        else:
            pass
        im = Image.fromarray(image)
        im.save(os.path.join(self.savePath, name))


def imageSaver(image, name, mode, savePath):
    if mode == "ir" or mode == 'rgb':
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif mode == 'depth':
        image = extractDepth.colorize(image, (None, 5000), cv2.COLORMAP_BONE)
    else:
        raise NameError("Wrong argument")
    im = Image.fromarray(image)
    im.save(os.path.join(savePath, name + ".png"))


def main():
    SAVE = False
    VIZ = False
    cameras = 'FK'
    controlDev = xda.XsControl_construct()
    if SAVE:
        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y_%H-%M")
        savePath = r"C:\Users\Olivier Desclaux\OneDrive\Recherche\PICUPE\Python-Recalage\ims"
        savePathIR = os.path.join(savePath, dt_string, "ir")
        savePathDepth = os.path.join(savePath, dt_string, "depth")
        savePathRGB = os.path.join(savePath, dt_string, "rgb")
        os.mkdir(os.path.join(savePath, dt_string))
        os.mkdir(savePathIR)
        os.mkdir(savePathDepth)
        os.mkdir(savePathRGB)

    # streams, _ = videostream.selectStreams(cameras, True)
    streams, _ = vsmp.selectStreams(cameras, True)
    # flirStream, kinectStream = streams
    kinectStream = streams[0]

    print("Recording starts in 3 seconds")
    time.sleep(3)
    t1 = time.time()
    cnt = 0
    print("Recording...")
    while True:
        # t = time.time() - t1
        kinectFrame = kinectStream.read()
        # flirFrame = flirStream.read()

        if SAVE:
            imnum = str(cnt)
            threadIR = threading.Thread(target=imageSaver, args=(flirFrame, imnum, "ir", savePathIR)).start()
            threadDepth = threading.Thread(target=imageSaver,
                                           args=(kinectFrame.depth, imnum, "depth", savePathDepth)).start()
            threadRGB = threading.Thread(target=imageSaver, args=(kinectFrame.color, imnum, "rgb", savePathRGB)).start()
            if msvcrt.kbhit():
                if msvcrt.getwche() == '\r':
                    break
        if VIZ:
            if kinectFrame.color is not None:
                cv2.imshow("Color", kinectFrame.color)
            if kinectFrame.transformed_depth is not None:
                cv2.imshow("Transformed Depth",
                           extractDepth.colorize(kinectFrame.transformed_depth, (None, 5000), cv2.COLORMAP_BONE))
            # cv2.imshow("FLIR", flirFrame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                cv2.destroyAllWindows()
                break

        if not VIZ and not SAVE:
            if msvcrt.kbhit():
                if msvcrt.getwche() == "\r":
                    break
            # print("No breaking condition")

        cnt += 1
        # if cnt == 30:
        #     break
    t2 = time.time()
    print("Acquisition time: ", round(t2 - t1, 4), " s")
    print("Number of frames: ", cnt)
    kinectStream.stop()
    # flirStream.stop()
    # cv2.destroyAllWindows()
    return True


if __name__ == '__main__':
    main()
