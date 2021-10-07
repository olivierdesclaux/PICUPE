import os
import numpy as np
import cv2
import argparse

def main(directory, mode):
    folder = os.path.join(r"/sandbox/results", directory, mode)
    topLeftCorner = (100, 50)

    ims = os.listdir(folder)
    ims = [os.path.join(folder, x) for x in ims if len(x.split('.')) <= 1]
    timestamps = open(os.path.join(folder, "timestamps.txt"), "r")
    for im in ims:
        with open(im, 'rb') as f:
            image = np.load(f)
        ts = timestamps.readline().strip()
        cv2.putText(image, ts, topLeftCorner, cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0))
        cv2.imshow("Image", image)
        if cv2.waitKey(10) == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    allResultsDir = r"C:\Users\picup\Desktop\PICUPE\sandbox\results"
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
    main(directory, mode)