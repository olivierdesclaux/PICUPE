import os
import cv2.cv2 as cv2
import multiprocessing
import numpy as np
import argparse


def visualizeResults(dataPath):
    # newSavePath = os.path.join(os.path.join(r"/sandbox/results", dataPath))
    # savePathDepth = os.path.join(newSavePath, "depth")
    # savePathRGB = os.path.join(newSavePath, "rgb")
    savePathWebcam = os.path.join(dataPath, "flir")
    rgb = multiprocessing.Process(target=vizKinect, args=(dataPath,))
    # depth = multiprocessing.Process(target=viz, args=(savePathDepth, "Depth"))
    webcam = multiprocessing.Process(target=vizFlir, args=(savePathWebcam,))

    processes = [rgb, webcam]
    for proc in processes:
        proc.start()

    for proc in processes:
        proc.join()
    cv2.destroyAllWindows()


def vizKinect(path):
    savePathDepth = os.path.join(path, "depth")
    savePathRGB = os.path.join(path, "rgb")
    imagesRGB = os.listdir(savePathRGB)
    imagesRGB = [os.path.join(savePathRGB, x) for x in imagesRGB if len(x.split('.')) <= 1]
    imagesDepth = os.listdir(savePathDepth)
    imagesDepth = [os.path.join(savePathDepth, x) for x in imagesDepth if len(x.split('.')) <= 1]

    topLeftCorner = (100, 50)
    timestamps = open(os.path.join(savePathRGB, "timestamps.txt"), "r")
    # print(images)
    cv2.namedWindow("Kinect")  # Create a named window
    cv2.moveWindow("Kinect", 30, 30)
    for rgb, depth in zip(imagesRGB, imagesDepth):
        with open(rgb, 'rb') as f:
            imRGB = np.load(f)
            imRGB = cv2.resize(imRGB, dsize=None, fx=0.5, fy=0.5)
            imRGB = imRGB[:, :, :3]
        with open(depth, 'rb') as f:
            imDepth = np.load(f)
            imDepth = cv2.resize(imDepth, dsize=None, fx=0.5, fy=0.5)
        # print(imDepth.shape)
        # print(imRGB.shape)
        ts = timestamps.readline().strip()
        # image = cv2.hconcat((imRGB, np.stack((imDepth, imDepth, imDepth))))
        image = cv2.hconcat((imRGB, imDepth))
        cv2.putText(image, ts, topLeftCorner, cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.imshow("Kinect", image)
        if cv2.waitKey(15) == ord("q"):
            break


def vizFlir(path):
    images = os.listdir(path)
    images = [os.path.join(path, x) for x in images if len(x.split('.')) <= 1]
    topLeftCorner = (100, 50)
    timestamps = open(os.path.join(path, "timestamps.txt"), "r")
    # print(images)
    cv2.namedWindow("FLIR")  # Create a named window
    cv2.moveWindow("FLIR", 30, 500)
    for im in images:
        with open(im, 'rb') as f:
            image = np.load(f)
        image = cv2.resize(image, dsize=None, fx=0.5, fy=0.5)
        ts = timestamps.readline().strip()
        cv2.putText(image, ts, topLeftCorner, cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.imshow("FLIR", image)
        if cv2.waitKey(15) == ord("q"):
            break

if __name__ == "__main__":
    allResultsDir = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\sandbox\results"
    latestDir = os.path.join(allResultsDir, os.listdir(allResultsDir)[-1])

    parser = argparse.ArgumentParser()

    parser.add_argument('-d', '--directory', type=str, dest='directory', default=latestDir,
                        help='Select a results directory',
                        required=False)
    parser.add_argument('--mode', type=str, dest='mode', default="rgb",
                        help='Select a modality: rgb, depth or webcam',
                        required=False)
    args = parser.parse_args().__dict__

    directory = args['directory']
    mode = args['mode']
    visualizeResults(directory)
