import sys
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
        self.experimentFrame.getExpName()
        self.config["Experiment Name"] = self.experimentFrame.experimentName
        self.config["Calibration Directory"] = self.calibFrame.directory
        self.config["IMUs"] = [imu for imu in self.imuFrame.IMUs if self.imuFrame.IMUs[imu].get() == 1]
        print(self.config)


class experimentFrame(ttk.LabelFrame):
    def __init__(self, row, column, rowspan, columnspan):
        super(experimentFrame, self).__init__(text="Experiment Name")
        self.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan)
        self.expNameVar = tk.StringVar()
        self.experimentName = None
        self.experimentNameEntry = tk.Entry(self, textvariable=self.expNameVar)
        self.experimentNameEntry.grid(row=1, column=1)
        self.expNameButton = ttk.Button(self, text="Ok", command=self.getExpName)
        self.expNameButton.grid(row=1, column=2)
        self.defaultNameButton = ttk.Button(self, text="Default", command=self.getDefaultName)
        self.defaultNameButton.grid(row=1, column=3)

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
        self.webcamPortNum = findCameraPort("webcam")
        self.create_ui()
        # self.grid(sticky=tk.NSEW)
        self.bind('<<MessageGenerated>>', self.on_next_frame)
        self.radioButtonFrame = ttk.LabelFrame(self, text="Camera")
        self.radioButtonFrame.grid(row=1, column=1)
        self.camVar = tk.StringVar()
        self.kinectRadioButton = ttk.Radiobutton(self.radioButtonFrame,
                                                 text="Kinect",
                                                 variable=self.camVar,
                                                 value="kinect").grid(column=1, row=1)
        self.webcamRadioButton = ttk.Radiobutton(self.radioButtonFrame,
                                                 text="Webcam",
                                                 variable=self.camVar,
                                                 value="webcam").grid(column=1, row=2)

        self.camVar.set("kinect")  # Set the default value of the radioButton to the kinect i.e. 1

    def create_ui(self):
        self.button_frame = ttk.LabelFrame(self, text="On/Off")
        self.stop_button = ttk.Button(self.button_frame, text="Stop", command=self.stop)
        self.stop_button.grid(row=1, column=2)
        self.start_button = ttk.Button(self.button_frame, text="Start", command=self.start)
        self.start_button.grid(row=1, column=1)
        self.view = ttk.Label(self, image=self.photo)
        self.view.grid(row=2, column=2)
        self.button_frame.grid(row=2, column=1)

    def start(self):
        self.is_running = True
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.queue.put(np.array(Image.new("RGB", (self.imageHeight, self.imageWidth), "white")))
        self.is_running = False

    def videoLoop(self):
        if self.camVar.get() == "kinect":
            cap = k4a.PyK4A(k4a.Config(
                color_resolution=k4a.ColorResolution.RES_720P,
                depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                camera_fps=k4a.FPS.FPS_30,
                synchronized_images_only=True))
            cap.start()
        elif self.camVar.get() == "webcam":
            cap = cv2.VideoCapture(self.webcamPortNum, cv2.CAP_DSHOW)
            # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.imageWidth)
            # cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.imageHeight)
        else:
            return

        while self.is_running:
            ret, frame = cap.read()
            image = cv2.cvtColor(cv2.resize(frame, (self.imageHeight, self.imageWidth)), cv2.COLOR_BGR2RGB)
            self.queue.put(image)
            self.event_generate('<<MessageGenerated>>')

    def on_next_frame(self, eventargs):
        if (not self.queue.empty()) and self.is_running:
            image = self.queue.get()
            image = Image.fromarray(image)
            self.photo = ImageTk.PhotoImage(image)
            self.view.configure(image=self.photo)
        else:
            self.photo = self.basicPhoto
            self.view.configure(image=self.photo)


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
        self.doCalib = tk.IntVar()
        self.checkBox = tk.Checkbutton(self, text='Use existing calibration directory',
                                       variable=self.doCalib,
                                       onvalue=1,
                                       offvalue=0,
                                       command=self.checkIfCalib)
        self.checkBox.grid(column=1, row=2, pady=10)

    def destroyCalibButton(self):
        self.button.destroy()
        self.label.destroy()
        self.directory = None

    def browseButton(self):
        self.button = ttk.Button(self, text="Browse A Directory", command=self.calibDirDialog)
        self.button.grid(column=1, row=4)

    def calibDirDialog(self):
        initialDir = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results"
        self.directory = filedialog.askdirectory(initialdir=initialDir, title="Select A Calibration Directory")
        # self.label = ttk.Label(self, text="")
        self.label = ttk.Label(self, text=self.directory)
        self.label.grid(column=1, row=5)
        # self.label.configure(text=self.directory)

    def checkIfCalib(self):
        if self.doCalib.get() == 1:
            self.browseButton()
        elif self.doCalib.get() == 0:
            self.destroyCalibButton()


def main():
    root = Root()
    root.mainloop()

    return root.config


if __name__ == "__main__":
    main()
