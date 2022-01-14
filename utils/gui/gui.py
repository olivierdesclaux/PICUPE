import sys
import os
import threading
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import cv2.cv2 as cv2
from queue import Queue
from PIL import Image
from PIL import ImageTk
import numpy as np
import pyk4a as k4a
import datetime
from readerWriterClass import findCameraPort
import extractDepth

sys.path.append("../Recalage")
from calibrate import calibrateCamera
from stereocalibration import stereoCalibrate, rectify


class Root(tk.Tk):
    def __init__(self):
        super(Root, self).__init__()
        self.title("PICUPE Configuration")
        self.minsize(640, 400)
        # self.wm_iconbitmap('icon.ico')
        self.experimentFrame = experimentFrame(row=0, column=0, rowspan=1, columnspan=1)
        self.calibFrame = calibFrame(row=8, column=0, rowspan=1, columnspan=1)
        self.imuFrame = IMUFrame(row=1, column=0, rowspan=5, columnspan=1)
        self.cameraFrame = cameraFrame(row=0, column=1, rowspan=7, columnspan=2)
        self.makeSummaryFrame(row=8, column=1, rowspan=1, columnspan=1)
        self.config = {}

    def makeSummaryFrame(self, row, column, rowspan, columnspan):
        self.summaryFrame = ttk.Frame(self)
        self.summaryFrame.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan)
        self.saveButton = ttk.Button(self.summaryFrame, text="Save Config", command=self.saveConfig)
        self.saveButton.grid(row=1, column=1)

        self.saveAndExitButton = ttk.Button(self.summaryFrame, text="Save and Exit", command=self.saveAndExit)
        self.saveAndExitButton.grid(row=1, column=2)

        self.abortButton = ttk.Button(self.summaryFrame, text="Abort", command=self.abortConfig)
        self.abortButton.grid(row=1, column=3)

    def abortConfig(self):
        sys.exit()

    def saveAndExit(self):
        self.saveConfig()
        self.destroy()

    def saveConfig(self):
        self.config["Experiment Name"] = self.experimentFrame.getExpName()
        self.config["Results Directory"] = self.experimentFrame.saveDirVar.get()
        self.config["Calibration Directory"] = self.calibFrame.directory
        self.config["IMUs"] = [imu for imu in self.imuFrame.IMUs if self.imuFrame.IMUs[imu].get() == 1]
        print(self.config)


class experimentFrame(ttk.LabelFrame):
    def __init__(self, row, column, rowspan, columnspan):
        super(experimentFrame, self).__init__(text="Experiment Information")
        self.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan)
        self.l1 = ttk.Label(self)
        self.l1.grid(column=0, row=1)
        self.l1.configure(text="Patient Name")
        self.expNameVar = tk.StringVar()
        self.experimentName = None
        self.experimentNameEntry = tk.Entry(self, textvariable=self.expNameVar)
        self.experimentNameEntry.grid(row=1, column=1)
        # self.expNameButton = ttk.Button(self, text="Ok", command=self.getExpName)
        # self.expNameButton.grid(row=1, column=2)
        self.defaultNameButton = ttk.Button(self, text="Default", command=self.getDefaultName)
        self.defaultNameButton.grid(row=1, column=2)

        self.l2 = ttk.Label(self)
        self.l2.grid(column=0, row=2)
        self.l2.configure(text="Save Directory")
        self.saveDirVar = tk.StringVar()
        self.saveDirVar.set(r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results")
        self.saveDir = self.saveDirVar.get()
        self.saveDirEntry = tk.Entry(self, textvariable=self.saveDirVar)
        self.saveDirEntry.grid(row=2, column=1)
        self.saveDirButton = ttk.Button(self, text="Browse", command=self.browseSaveDir)
        self.saveDirButton.grid(row=2, column=2)

    def browseSaveDir(self):
        initialDir = r"C:\Users\Recherche\OneDrive - polymtl.ca"
        self.saveDir = filedialog.askdirectory(initialdir=initialDir, title="Select A Save Directory")
        # self.label = ttk.Label(self, text="")
        self.saveDirVar.set(self.saveDir)

    def getDefaultName(self):
        self.experimentName = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
        self.expNameVar.set(self.experimentName)

    def getExpName(self):
        self.experimentName = self.expNameVar.get()
        return self.experimentName


class cameraFrame(ttk.LabelFrame):
    """
    https://stackoverflow.com/questions/52583911/create-a-gui-that-can-turn-on-off-camera-images-using-python-3-and
    -tkinter
    """

    def __init__(self, row, column, rowspan, columnspan):
        super(cameraFrame, self).__init__(text="Camera Testing")
        self.grid(column=column, row=row, rowspan=rowspan, columnspan=columnspan, padx=10, pady=10)
        self.is_running = False
        self.thread = None
        self.queue = Queue()
        self.imageHeight = 400
        self.imageWidth = 300
        self.basicPhoto = ImageTk.PhotoImage(Image.new("RGB", (self.imageHeight, self.imageWidth), (240, 240, 237)))
        self.photo = self.basicPhoto
        self.webcamPortNum = None
        self.flirPortNum = None
        self.browseCamerasButton = ttk.Button(self, text="Browse Cameras", command=self.browseCameras)
        self.browseCamerasButton.grid(column=0, row=2)
        self.create_ui()
        self.cap = None
        self.bind('<<MessageGenerated>>', self.on_next_frame)
        self.radioButtonFrame = ttk.LabelFrame(self, text="Camera")
        self.radioButtonFrame.grid(row=0, column=0, rowspan=2, columnspan=1)
        self.camVar = tk.StringVar()
        self.rgbRadioButton = ttk.Radiobutton(self.radioButtonFrame,
                                              text="Kinect RGB",
                                              variable=self.camVar,
                                              value="kinect rgb").grid(column=1, row=1)
        self.depthRadioButton = ttk.Radiobutton(self.radioButtonFrame,
                                                text="Kinect Depth",
                                                variable=self.camVar,
                                                value="kinect depth").grid(column=1, row=2)
        self.flirRadioButton = ttk.Radiobutton(self.radioButtonFrame,
                                               text="FLIR",
                                               variable=self.camVar,
                                               value="flir").grid(column=1, row=3)
        self.webcamRadioButton = ttk.Radiobutton(self.radioButtonFrame,
                                                 text="Webcam",
                                                 variable=self.camVar,
                                                 value="webcam").grid(column=1, row=4)
        self.camVar.set("kinect rgb")  # Set the default value of the radioButton to the kinect rgb

    def create_ui(self):
        self.button_frame = ttk.LabelFrame(self, text="On/Off")
        self.stop_button = ttk.Button(self.button_frame, text="Stop", command=self.stop)
        self.stop_button.grid(row=1, column=2)
        self.start_button = ttk.Button(self.button_frame, text="Start", command=self.start)
        self.start_button.grid(row=1, column=1)
        self.view = ttk.Label(self, image=self.photo)
        self.view.grid(row=0, column=1, rowspan=4, columnspan=3)
        self.button_frame.grid(row=3, column=0)

    def start(self):
        if self.is_running:
            self.stop()
        self.is_running = True
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.queue.put(np.array(Image.new("RGB", (self.imageHeight, self.imageWidth), "white")))
        if self.cap is not None:
            self.cap.stop()
            self.cap = None
        self.is_running = False

    def videoLoop(self):
        self.cap = camera(self.camVar.get(), self.webcamPortNum, self.flirPortNum)
        if not self.cap.open():
            self.stop()
            return
        while self.is_running:
            image = self.cap.read()
            if image is not None:
                image = cv2.cvtColor(cv2.resize(image, (self.imageHeight, self.imageWidth)), cv2.COLOR_BGR2RGB)
                self.queue.put(image)
                self.event_generate('<<MessageGenerated>>')
            else:
                self.stop()
                break

    def on_next_frame(self, eventargs):
        if (not self.queue.empty()) and self.is_running:
            image = self.queue.get()
            image = Image.fromarray(image)
            self.photo = ImageTk.PhotoImage(image)
            self.view.configure(image=self.photo)
        else:
            self.photo = self.basicPhoto
            self.view.configure(image=self.photo)

    def browseCameras(self):
        self.webcamPortNum = findCameraPort("webcam")
        self.flirPortNum = findCameraPort("flir")
        try:
            kinect = k4a.PyK4A(k4a.Config(
                color_resolution=k4a.ColorResolution.RES_720P,
                depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                camera_fps=k4a.FPS.FPS_30,
                synchronized_images_only=True))
            kinect.start()
            kinect.stop()
            print("Found Kinect")
        except:
            print("Failed to find Kinect")


class IMUFrame(ttk.LabelFrame):
    def __init__(self, row, column, rowspan, columnspan):
        super(IMUFrame, self).__init__(text="Selected IMUs")
        self.grid(column=column, row=row, rowspan=rowspan, columnspan=columnspan)
        self.IMUs = {"Head": tk.IntVar(),
                     "Torso": tk.IntVar(),
                     "Hips": tk.IntVar(),
                     "Left Upper Arm": tk.IntVar(),
                     "Right Upper Arm": tk.IntVar(),
                     "Left Forearm": tk.IntVar(),
                     "Right Forearm": tk.IntVar(),
                     "Left Hand": tk.IntVar(),
                     "Right Hand": tk.IntVar(),
                     "Left Upper Leg": tk.IntVar(),
                     "Right Upper Leg": tk.IntVar(),
                     "Left Lower Leg": tk.IntVar(),
                     "Right Lower Leg": tk.IntVar(),
                     "Left Foot": tk.IntVar(),
                     "Right Foot": tk.IntVar(),
                     }

        self.individualsFrame = ttk.LabelFrame(self, text="Individual IMUs")
        self.individualsFrame.grid(row=0, column=0)
        for k, imu in enumerate(["Head", "Torso", "Hips"]):
            self.addCheckBox(self.individualsFrame, imu, self.IMUs[imu], row=k // 3, col=k % 3)

        self.armsFrame = ttk.LabelFrame(self, text="Arm IMUs")
        self.armsFrame.grid(row=1, column=0)
        for k, imu in enumerate(["Left Upper Arm", "Left Forearm", "Left Hand", "Right Upper Arm", "Right Forearm",
                                 "Right Hand"]):
            self.addCheckBox(self.armsFrame, imu, self.IMUs[imu], row=k // 3, col=k % 3)

        self.legsFrame = ttk.LabelFrame(self, text="Leg IMUs")
        self.legsFrame.grid(row=2, column=0)
        for k, imu in enumerate(["Left Upper Leg", "Left Lower Leg", "Left Foot", "Right Upper Leg", "Right Lower Leg",
                                 "Right Foot"]):
            self.addCheckBox(self.legsFrame, imu, self.IMUs[imu], row=k // 3, col=k % 3)

    def addCheckBox(self, frame, IMUName, IMUVar, row, col):
        box = tk.Checkbutton(frame, text=IMUName,
                             variable=IMUVar,
                             onvalue=1,
                             offvalue=0)

        box.grid(column=col, row=row)


class calibFrame(ttk.LabelFrame):
    def __init__(self, row, column, rowspan, columnspan):
        super(calibFrame, self).__init__(text="Calibration")
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
        # self.label = ttk.Label(self, text="")
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
        self.config = {}

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

    def launchCalib(self):
        print("Launching Calib")
        calibConfig = {
            "calibInitialGrids": self.calibInitialGrids.get(),
            "calibMinGrids": self.calibMinGrids.get(),
            "kinectCalib": self.kinectCalibFile.get(),
            "stereocalibInitialGrids": self.stereoCalibGrids.get()}
        self.config = calibConfig
        return True
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


class camera:
    def __init__(self, cameraName, webcamPort, flirPort):
        self.name = cameraName
        try:
            if self.name == "webcam":
                if webcamPort > -1:
                    self.webcamPort = webcamPort
                else:
                    self.webcamPort = findCameraPort("webcam")
            elif self.name == "flir":
                if flirPort > -1:
                    self.flirPort = flirPort
                else:
                    self.flirPort = findCameraPort("flir")
        except TypeError:
            raise Exception("No camera found. Browse cameras")
        self.cam = None

    def open(self):
        try:
            if self.name == "webcam" and self.webcamPort > -1:
                self.cam = cv2.VideoCapture(self.webcamPort, cv2.CAP_DSHOW)
            elif self.name == "kinect rgb" or self.name == "kinect depth":
                self.cam = k4a.PyK4A(k4a.Config(
                    color_resolution=k4a.ColorResolution.RES_720P,
                    depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                    camera_fps=k4a.FPS.FPS_30,
                    synchronized_images_only=True))
                self.cam.start()
            elif self.name == "flir" and self.flirPort > -1:
                self.cam = cv2.VideoCapture(self.flirPort, cv2.CAP_DSHOW)
            else:
                print("Failed to open {}".format(self.name))
                return False
        except:
            print("Failed to open {}".format(self.name))
            return False
        return True

    def read(self):
        if self.name == "kinect rgb":
            return self.cam.get_capture().color
        elif self.name == "kinect depth":
            frame = self.cam.get_capture().transformed_depth
            return extractDepth.colorize(frame, (None, 5000), cv2.COLORMAP_BONE)
            # Have to convert from 16bit to 8bit for display
        elif self.name == "flir" or self.name == "webcam":
            _, frame = self.cam.read()
            return frame
        else:
            return None

    def stop(self):
        if self.cam is None:
            return
        if self.name == "kinect rgb" or self.name == "kinect depth":
            self.cam.stop()
        elif self.name == "flir" or self.name == "webcam":
            self.cam.release()
        else:
            return
        return


def main():
    os.environ["K4A_ENABLE_LOG_TO_STDOUT"] = "0"
    root = Root()
    root.mainloop()

    return root.config


if __name__ == "__main__":
    main()
