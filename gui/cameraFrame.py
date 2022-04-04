import threading
import sys
import tkinter as tk
from tkinter import ttk
import cv2.cv2 as cv2
from queue import Queue
from PIL import Image
from PIL import ImageTk
import numpy as np
import pyk4a as k4a
sys.path.append("../utils")
from utils.readerWriterClass import findCameraPort
from utils.extractDepth import colorize


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
        self.stop_button["state"] = "disabled"
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
        self.start_button["state"] = "disabled"
        self.stop_button["state"] = "normal"

    def stop(self):
        self.queue.put(np.array(Image.new("RGB", (self.imageHeight, self.imageWidth), "white")))
        if self.cap is not None:
            self.cap.stop()
            self.cap = None
        self.is_running = False
        self.start_button["state"] = "normal"
        self.stop_button["state"] = "disabled"

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
            elif self.name == "kinect rgb" or self.name == "kinect depth" or self.name == "kinect":
                self.cam = k4a.PyK4A(k4a.Config(
                    color_resolution=k4a.ColorResolution.RES_720P,
                    depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                    camera_fps=k4a.FPS.FPS_30,
                    synchronized_images_only=True))
                self.cam.start()
            elif self.name == "flir" and self.flirPort > -1:
                self.cam = cv2.VideoCapture(self.flirPort, cv2.CAP_DSHOW)
                self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
                self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
                self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
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
            return colorize(frame, (None, 5000), cv2.COLORMAP_BONE)
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
