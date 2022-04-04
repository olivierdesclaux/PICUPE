import os
import cv2.cv2 as cv2
import numpy as np
import argparse
from extractDepth import colorize

def visualizeResults(path, pause):
    savePathFlir = os.path.join(path, "flir")
    savePathDepth = os.path.join(path, "depth")
    savePathRGB = os.path.join(path, "rgb")

    imagesRGB = os.listdir(savePathRGB)
    imagesRGB = [os.path.join(savePathRGB, x) for x in imagesRGB if len(x.split('.')) <= 1]
    imagesDepth = os.listdir(savePathDepth)
    imagesDepth = [os.path.join(savePathDepth, x) for x in imagesDepth if len(x.split('.')) <= 1]
    imagesFlir = os.listdir(savePathFlir)
    imagesFlir = [os.path.join(savePathFlir, x) for x in imagesFlir if len(x.split('.')) <= 1]

    topLeftCorner = (10, 50)
    timestampsKinect = open(os.path.join(savePathRGB, "timestamps.txt"), "r")
    timestampsFlir = open(os.path.join(savePathFlir, "timestamps.txt"), "r")
    cv2.namedWindow("Kinect")  # Create a named window
    cv2.moveWindow("Kinect", 30, 30)
    for i, (rgb, depth, flir) in enumerate(zip(imagesRGB, imagesDepth, imagesFlir)):
        # print(i)
        with open(rgb, 'rb') as f:
            imRGB = np.load(f)
            imRGB = cv2.resize(imRGB, dsize=None, fx=0.7, fy=0.7)
            imRGB = imRGB[:, :, :3]
        with open(depth, 'rb') as f:
            imDepth = np.load(f)
            imDepth = colorize(imDepth, (None, 5000), cv2.COLORMAP_BONE)
            imDepth = cv2.resize(imDepth, dsize=None, fx=0.7, fy=0.7)

        with open(flir, 'rb') as f:
            imFlir = np.load(f)
            imFlir = cv2.resize(imFlir, dsize=None, fx=0.7, fy=0.7)

        tsKinect = timestampsKinect.readline().strip()
        image = cv2.hconcat((imRGB, imDepth))
        cv2.putText(image, tsKinect, topLeftCorner, cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.putText(image, str(i), (60, 60), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.imshow("Kinect", image)

        tsFlir = timestampsFlir.readline().strip()
        # cv2.putText(imFlir, tsFlir, topLeftCorner, cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.putText(imFlir, tsFlir, topLeftCorner, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0))
        cv2.putText(imFlir, str(i), (60, 60), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.imshow("Flir", imFlir)
        if not pause:
            if cv2.waitKey(10) == ord("q"):
                break
        else:
            while True:
                key = cv2.waitKey(10)
                if key == ord("n"):
                    break
                elif key == ord("q"):
                    return
if __name__ == "__main__":
    allResultsDir = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\sandbox\results"
    latestDir = os.path.join(allResultsDir, os.listdir(allResultsDir)[-1])

    parser = argparse.ArgumentParser()

    parser.add_argument('-d', '--directory', type=str, dest='directory', default=latestDir,
                        help='Select a results directory',
                        required=False)
    parser.add_argument('-p', '--pause', dest='pause', default=False, action="store_true",
                        help='If pause, will work frame by frame',
                        required=False)

    parser.add_argument('--mode', type=str, dest='mode', default="rgb",
                        help='Select a modality: rgb, depth or webcam',
                        required=False)
    args = parser.parse_args().__dict__

    directory = args['directory']
    mode = args['mode']
    pause = args["pause"]
    visualizeResults(directory, pause)
