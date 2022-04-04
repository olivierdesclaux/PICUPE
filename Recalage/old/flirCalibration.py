import os
import sys
import threading

sys.path.append("../..")
from Recalage.calibrate import Calibrator
import cv2.cv2 as cv2
from utils.myLogger import Logger



def main():
    savePath = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\kinect Calib"
    if not os.path.isdir(savePath):
        os.mkdir(savePath)

    logger = Logger(savePath, "customLog.log")
    listener = threading.Thread(target=logger.logWriter, daemon=True)
    listener.start()

    flags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_ASPECT_RATIO
    initialGuess = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\kinect " \
                   r"Calib\kinectInitialGuess.json"

    cv2.destroyAllWindows()
    for i in range(5, 8):
        logger.log("Set" + str(i))
        savePathi = os.path.join(savePath, "Set" + str(i))
        if not os.path.isdir(savePathi):
            os.mkdir(savePathi)
        calibrator = Calibrator(savePathi, "kinect", flags, initialGuess, logger=logger, initialGrids=30, minGrids=20)
        calibrator.performCalibration()

    listener.join()


if __name__ == "__main__":
    main()
