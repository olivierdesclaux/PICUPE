import os
import cv2.cv2 as cv2
import multiprocessing
import numpy as np
import argparse


def visualizeResults(dataPath):
    newSavePath = os.path.join(os.path.join(r"../sandbox/results", dataPath))
    savePathDepth = os.path.join(newSavePath, "depth")
    savePathRGB = os.path.join(newSavePath, "rgb")
    savePathWebcam = os.path.join(newSavePath, "flir")
    savePathXSens = os.path.join(newSavePath, "../XSens")
    rgb = multiprocessing.Process(target=viz, args=(savePathRGB, "RGB"))
    depth = multiprocessing.Process(target=viz, args=(savePathDepth, "Depth"))
    webcam = multiprocessing.Process(target=viz, args=(savePathWebcam, "Webcam"))

    processes = [rgb, depth, webcam]
    for proc in processes:
        proc.start()

    for proc in processes:
        proc.join()
    cv2.destroyAllWindows()


def viz(path, t):
    images = os.listdir(path)
    images = [os.path.join(path, x) for x in images if len(x.split('.')) <= 1]
    topLeftCorner = (100, 50)
    timestamps = open(os.path.join(path, "timestamps.txt"), "r")
    # print(images)
    cv2.namedWindow(t)  # Create a named window
    if t == "RGB":
        cv2.moveWindow(t, 30, 30)
    elif t == "Depth":
        cv2.moveWindow(t, 840, 30)
    elif t == "Webcam":
        cv2.moveWindow(t, 30, 500)
    for im in images:
        with open(im, 'rb') as f:
            image = np.load(f)
        image = cv2.resize(image, dsize=None, fx=0.5, fy=0.5)
        ts = timestamps.readline().strip()
        cv2.putText(image, ts, topLeftCorner, cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.imshow(t, image)
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
