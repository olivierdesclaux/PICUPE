import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
import os
import cv2.cv2 as cv2
import json
import threading

from Recalage.calibrate import Calibrator
from Recalage.stereocalibration import StereoCalibrator
from utils.myLogger import Logger


class calibFrame(ttk.LabelFrame):
    def __init__(self, parent, row, column, rowspan, columnspan):
        super(calibFrame, self).__init__(text="Calibration")
        self.parent = parent
        self.grid(column=column, row=row, rowspan=rowspan, columnspan=columnspan)
        self.button = None
        self.calibrationSaveDirectory = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Calibration " \
                                        r"Directories"
        self.directory = tk.StringVar()
        self.label = ttk.Label(self, textvariable=self.directory)
        self.label.grid(column=0, row=3, columnspan=3)
        self.browseButton = ttk.Button(self, text="Browse A Directory", command=self.calibDirDialog)
        self.browseButton.grid(column=0, row=0)
        self.calibButton = ttk.Button(self, text="Perform Calibration", command=self.openCalibWindow)
        self.calibButton.grid(column=1, row=0)
        self.calibConfig = {}
        self.kinectCalibrationWindow = None

    def openCalibWindow(self):
        self.parent.checkExperimentName()
        self.kinectCalibrationWindow = calibWindow(self)

    def calibDirDialog(self):
        newDir = filedialog.askdirectory(initialdir=self.calibrationSaveDirectory,
                                         title="Select A Calibration Directory")
        self.directory.set(newDir)

    def updateCalibDirectory(self):
        self.label = ttk.Label(self, text=self.directory)
        self.label.grid(column=0, row=3, columnspan=3)
        self.label.configure(text=self.directory)


class calibWindow(tk.Frame):
    def __init__(self, parent):
        super(calibWindow, self).__init__()
        self.parent = parent
        ## Import calibration module
        # Dialog window to choose calibration parameters
        # Save results into a dir. If no experiment name, use default name. This dir becomes the calib dir
        # Must be careful: if enduser changes the experiment name after calibration, must change the directory
        # location
        self.window = tk.Toplevel(self.parent)
        self.window.wm_title("Calibration Parameters")
        self.window.grab_set()

        self.makeFlirCalibFrame(row=0, column=0)
        self.makeStereoCalibFrame(row=1, column=0)
        self.makeKinectCalibFrame(row=2, column=0, columnspan=2)
        self.makeLaunchCalibButton(row=0, column=1)
        self.makeAbortButton(row=1, column=1)
        self.config = {}

    def getConfigData(self):
        self.parent.calibConfig = self.config
        return self.config

    def makeStereoCalibFrame(self, row, column):
        self.stereoCalibFrame = ttk.LabelFrame(self.window, text="Stereo Calibration Parameters")
        self.stereoCalibFrame.grid(row=row, column=column, pady=10, sticky=tk.W)
        self.stereoInitialGrids = tk.IntVar()
        self.stereoInitialGrids.set(20)
        self.l3 = tk.Label(self.stereoCalibFrame, text="Number of initial Grids").grid(row=0, column=0)
        self.e3 = tk.Entry(self.stereoCalibFrame, textvariable=self.stereoInitialGrids).grid(row=0, column=1)

        self.stereoMinGrids = tk.IntVar()
        self.stereoMinGrids.set(15)
        self.l4 = tk.Label(self.stereoCalibFrame, text="Number of minimal Grids").grid(row=1, column=0)
        self.e4 = tk.Entry(self.stereoCalibFrame, textvariable=self.stereoMinGrids).grid(row=1, column=1)

    def makeFlirCalibFrame(self, row, column):
        self.flirCalibFrame = ttk.LabelFrame(self.window, text="FLIR Calibration Parameters")
        self.flirCalibFrame.grid(row=row, column=column, pady=10, sticky=tk.W)
        self.calibInitialGrids = tk.IntVar()
        self.calibInitialGrids.set(20)
        self.calibMinGrids = tk.IntVar()
        self.calibMinGrids.set(15)
        self.l1 = tk.Label(self.flirCalibFrame, text="Number of initial Grids").grid(row=0, column=0)
        self.e1 = tk.Entry(self.flirCalibFrame, textvariable=self.calibInitialGrids).grid(row=0, column=1)
        self.l2 = tk.Label(self.flirCalibFrame, text="Minimum number of grids \n for successful calibration").grid(
            row=1,
            column=0)
        self.e2 = tk.Entry(self.flirCalibFrame, textvariable=self.calibMinGrids).grid(row=1, column=1)

    def makeLaunchCalibButton(self, row, column):
        self.launchButton = tk.Button(self.window, text="Start Calibration", command=self.launchCalib)
        self.launchButton.grid(row=row, column=column)

    def makeAbortButton(self, row, column):
        self.launchButton = tk.Button(self.window, text="Abort", command=self.abortCalib)
        self.launchButton.grid(row=row, column=column)

    def abortCalib(self):
        self.config = {}
        self.window.destroy()

    def makeKinectCalibFrame(self, row, column, columnspan):
        self.kinectFrame = ttk.LabelFrame(self.window, text="Kinect Calibration Parameters")
        self.kinectFrame.grid(row=row, column=column, columnspan=columnspan, pady=10, sticky=tk.W)

        self.kinectCalibFile = tk.StringVar()
        self.kinectCalibFile.set(r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\kinect "
                                 r"Calib\kinectCalib.json")
        self.kinectLabel = ttk.Label(self.kinectFrame, textvariable=self.kinectCalibFile)
        self.kinectLabel.grid(column=0, row=0)
        self.kinectLabel.configure(text=self.kinectCalibFile)

        self.browseKinect = ttk.Button(self.kinectFrame, text="Browse Kinect Calib File",
                                       command=self.calibKinectDialog)
        self.browseKinect.grid(row=1, column=0)

    def calibKinectDialog(self):
        initialDir = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results"
        newKinectCalibFile = filedialog.askopenfilename(initialdir=initialDir, title="Select A Calibration Directory",
                                                        filetypes=[("json", ".json")])
        self.kinectCalibFile.set(newKinectCalibFile)

    def launchCalib(self):
        expName = self.parent.parent.experimentFrame.getExpName()
        savePathCalib = createCalibDirectory(self.parent.calibrationSaveDirectory, expName)
        calibConfig = {
            "calibInitialGrids": self.calibInitialGrids.get(),
            "calibMinGrids": self.calibMinGrids.get(),
            "kinectCalib": self.kinectCalibFile.get(),
            "stereoInitialGrids": self.stereoInitialGrids.get(),
            "stereoMinGrids": self.stereoMinGrids.get()}

        # Initialise logger
        logger = Logger(savePathCalib, "calib.log")
        listener = threading.Thread(target=logger.logWriter, daemon=True)
        listener.start()

        # FLIR CALIBRATION
        flirCalibFlags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_FIX_K3
        initialGuess = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\FLIRCalibration\flirCalib" \
                       r".json "

        logger.log("STARTING FLIR CALIBRATION")
        calibrator = Calibrator(savePathCalib, "flir", flirCalibFlags, initialGuess, logger,
                                initialGrids=calibConfig["calibInitialGrids"], minGrids=calibConfig["calibMinGrids"])
        calibConfig["flirCalib"] = calibrator.performCalibration()
        if calibrator.aborted:
            self.window.destroy()
            return

        # STEREOCALIBRATION
        logger.log("")
        logger.log("STARTING STEREOCALIBRATION")
        calibFiles = [calibConfig["flirCalib"], calibConfig["kinectCalib"]]
        cameras = ["flir", "kinect"]
        stereoCalibrator = StereoCalibrator(savePathCalib, cameras, calibFiles, logger,
                                            initialGrids=calibConfig["stereoInitialGrids"],
                                            minGrids=calibConfig["stereoMinGrids"])
        stereoCalibrator.performStereocalibration()

        if stereoCalibrator.aborted:
            self.window.destroy()
            return

        # SAVE PARAMETERS AND RESULTS
        self.calibConfig = calibConfig

        with open(os.path.join(savePathCalib, "calibParameters.json"), 'w') as file:
            json.dump(self.calibConfig, file, indent=4)
        self.getConfigData()
        self.parent.directory.set(savePathCalib)
        logger.stop()
        listener.join()
        self.window.destroy()


def createCalibDirectory(resultsDir, saveName):
    newSavePath = os.path.join(resultsDir, saveName)
    savePathCalib = os.path.join(newSavePath, "calib")
    if not os.path.isdir(newSavePath):
        os.mkdir(newSavePath)
        os.mkdir(savePathCalib)
    else:
        raise Exception("Specified saving parameters lead to an already existing directory.")

    return savePathCalib
