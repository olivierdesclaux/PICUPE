import logging
import multiprocessing
import threading
import time
import sys
import traceback
import random
import os
import shutil

class Logger:
    def __init__(self, savePath, name):
        self.logger = self.initLogger(savePath, name)
        self.loggerQueue = multiprocessing.Queue()

    def initLogger(self, savePath, name):
        """
        Parameters
        ----------
        savePath: str, directory to save the log file
        name: str, log file name, e.g. "myLogFile.log"

        Returns
        -------
        logger: object of class logging.
        """
        # Initialise logging object
        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)
        # create file handler which logs even debug messages
        fh = logging.FileHandler(os.path.join(savePath, name))
        fh.setLevel(logging.DEBUG)
        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.ERROR)
        # create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        # add the handlers to the logger
        logger.addHandler(fh)
        logger.addHandler(ch)

        return logger

    def logWriter(self):
        """
        Function that will actually write to file. Will be threaded in main process. Reads data from the loggerQueue, and outputs it to
        Returns
        -------

        """
        while True:
            try:
                record = self.loggerQueue.get()
                if record is None:
                    break
                print(record)
                self.logger.info(record)
            except Exception:
                print('Whoops! Problem:', file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                sys.exit()

    def log(self, msg):
        currentProcess = str(multiprocessing.current_process().name) + ": "
        self.loggerQueue.put(currentProcess + msg)

    def stop(self):
        self.loggerQueue.put_nowait(None)


def createSaveDirectories(resultsDir, saveName):
    newSavePath = os.path.join(resultsDir, saveName)
    savePathDepth = os.path.join(newSavePath, "depth")
    savePathRGB = os.path.join(newSavePath, "rgb")
    savePathFlir = os.path.join(newSavePath, "flir")
    savePathXsens = os.path.join(newSavePath, "XSens")
    savePathCalib = os.path.join(newSavePath, "calib")

    if not os.path.isdir(newSavePath):
        os.mkdir(newSavePath)
        os.mkdir(savePathDepth)
        os.mkdir(savePathRGB)
        os.mkdir(savePathFlir)
        os.mkdir(savePathXsens)
        os.mkdir(savePathCalib)
    else:
        raise Exception("Specified saving parameters lead to an already existing directory.")

    return newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens, savePathCalib

def copyCalibrationFiles(calibDir, savePathCalib):
    """
    Copies calibration parameters from a previous experiment to the directory of the specified new experiment.
    Checks that we are only copying the calibration files, and not the entire experiment data.
    Parameters
    ----------
    calibDir: string, path to the experiment directory whose calibration parameters we want to copy
    savePathCalib: string, path to the "calib" directory where we want to store the copied calibration parameters

    Returns
    -------
    True, if succeeds. If calibration data is missing in the calibDir, raises an exception.

    """
    files = os.listdir(calibDir)
    if ("calib" in files) and os.path.isdir(os.path.join(calibDir, "calib")):
        calibDir = os.path.join(calibDir, "calib")
    calibrationFiles = ['ErrorCharts.png', 'FCalib.json', 'Rectify.json', 'stereo.json', "calibParameters.json"]
    if not set(calibrationFiles) == set(os.listdir(calibDir)):
        for x in calibrationFiles:
            if x not in os.listdir(calibDir):
                raise Exception("Missing {} in specified calibration directory".format(x))
        raise Exception("Specified calibration directory must contain exclusively {}".format(calibrationFiles))
    else:
        shutil.copytree(calibDir, savePathCalib, dirs_exist_ok=True)

#
# def main():
#     logger = initLogger("./")
#     loggerQueue = multiprocessing.Queue()
#     keepGoing = multiprocessing.Value('i', 1)
#     procs = []
#     for i in range(5):
#         p = multiprocessing.Process(target=foo2, args=(loggerQueue,))
#         p.name = "Bambou " + str(i)
#         procs.append(p)
#     listener = threading.Thread(target=logWriter, args=(logger, loggerQueue), daemon=True)
#     listener.start()
#     for proc in procs:
#         proc.start()
#     time.sleep(1)
#     with keepGoing.get_lock():
#         keepGoing.value = 0
#     for proc in procs:
#         proc.join()
#
#     loggerQueue.put_nowait(None)
#     # listener.join()
#     # for _ in range(5):
#     #     time.sleep(0.5)
#     #
#
#
# def logWriter(logger, loggerQueue):
#     while True:
#         try:
#             record = loggerQueue.get()
#             if record is None:
#                 break
#             print(record)
#             logger.info(record)
#         except Exception:
#             print('Whoops! Problem:', file=sys.stderr)
#             traceback.print_exc(file=sys.stderr)
#             sys.exit()
#
#
# def log(loggerQueue, msg):
#     currentProcess = str(multiprocessing.current_process().name) + ": "
#     loggerQueue.put(currentProcess + msg)
#
#
# def foo(loggerQueue):
#     # loggerQueue.put(str(time.time()))
#     currentProcess = str(multiprocessing.current_process().name) + ": "
#     loggerQueue.put(currentProcess + "Bla")
#
#
# def foo2(loggerQueue):
#     x = random.random()
#     if x < 0.5:
#         log(loggerQueue, "x below 0.5")
#
#     try:
#         x = 1 / 0
#     except ZeroDivisionError as e:
#         log(loggerQueue, str(e))
#         # print(e)
#
#
# if __name__ == '__main__':
#     main()
