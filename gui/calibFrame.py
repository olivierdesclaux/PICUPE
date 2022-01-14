import sys
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk

import cv2.cv2 as cv2

sys.path.append("../Recalage")
sys.path.append("../utils")
from stereocalibration import stereoCalibrate, rectify
from myLogger import createSaveDirectories

class calibFrame(ttk.LabelFrame):
    def __init__(self, parent, row, column, rowspan, columnspan):
        super(calibFrame, self).__init__(text="Calibration")
        self.parent = parent
        self.grid(column=column, row=row, rowspan=rowspan, columnspan=columnspan)
        self.button = None
        self.directory = None
        self.label = ttk.Label(self, text="")
        self.label.grid(column=0, row=3, columnspan=3)
        self.browseButton = ttk.Button(self, text="Browse A Directory", command=self.calibDirDialog)
        self.browseButton.grid(column=0, row=0)
        self.calibButton = ttk.Button(self, text="Perform Calibration", command=self.openCalibWindow)
        self.calibButton.grid(column=1, row=0)
        self.calibConfig = {}
        self.kinectCalibrationWindow = None

    def openCalibWindow(self):
        self.kinectCalibrationWindow = calibWindow(self)

    def calibDirDialog(self):
        initialDir = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results"
        self.directory = filedialog.askdirectory(initialdir=initialDir, title="Select A Calibration Directory")
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
        self.stereoCalibGrids = tk.IntVar()
        self.stereoCalibGrids.set(20)
        self.l3 = tk.Label(self.stereoCalibFrame, text="Number of initial Grids").grid(row=0, column=0)
        self.e3 = tk.Entry(self.stereoCalibFrame, textvariable=self.stereoCalibGrids).grid(row=0, column=1)

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

    def launchCalib(self):
        print("Launching Calib")
        expName = self.parent.parent.experimentFrame.getExpName()
        saveDir = self.parent.parent.experimentFrame.getSaveDir()
        _, _, _, _, _, savePathCalib = createSaveDirectories(saveDir, expName)
        calibConfig = {
            "calibInitialGrids": self.calibInitialGrids.get(),
            "calibMinGrids": self.calibMinGrids.get(),
            "kinectCalib": self.kinectCalibFile.get(),
            "stereocalibInitialGrids": self.stereoCalibGrids.get()}
        self.config = calibConfig
        self.getConfigData()
        return True
        # Ne pas oublier de sauvegarder les résultats de calibration et d'updater le fichier dans calibFrame
        flirCalibFlags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K3
        stereoFlags = cv2.CALIB_FIX_INTRINSIC


        savePathCalib = ""
        calibConfig["flirCalib"] = calibrateCamera("F", savePathCalib, flirCalibFlags, initialNumGrids=calibConfig[
            "calibInitialGrids"], minNumGrids=calibConfig["calibMinGrids"])

        calibConfig["stereoCalibrationFile"] = stereoCalibrate("FK", calibConfig["flirCalib"], calibConfig[
            "kinectCalib"], savePathCalib, stereoFlags, initialNumGrids=calibConfig["stereocalibInitialGrids"])

        # stereoCalibrationFile = stereoCalibrate("KF", kinectCalib, flirCalib, savePathCalib, stereoFlags,
        #                                         initialNumGrids=20)

        # Compute rectification matrices
        rectify(savePathCalib)
        self.calibConfig = calibConfig


    def makeKinectCalibFrame(self, row, column, columnspan):
        self.kinectFrame = ttk.LabelFrame(self.window, text="Kinect Calibration Parameters")
        self.kinectFrame.grid(row=row, column=column, columnspan=columnspan, pady=10, sticky=tk.W)

        self.kinectCalibFile = tk.StringVar()
        self.kinectCalibFile.set(r"C:\Users\Recherche\OneDrive - "
                                 r"polymtl.ca\PICUPE\Recalage\Results\CalibKinectFactory5Dist.json")
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
