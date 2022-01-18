import tkinter as tk
from tkinter import ttk


class IMUFrame(ttk.LabelFrame):
    def __init__(self, parent, row, column, rowspan, columnspan):
        super(IMUFrame, self).__init__(text="Selected IMUs")
        self.parent = parent
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

        self.checkButtonFrame = ttk.Frame(self)
        self.checkButtonFrame.grid(row=3, column=0)
        self.checkButton = ttk.Button(self.checkButtonFrame, text="Test IMU", command=self.testIMU)
        self.checkButton.grid(row=0, column=0)

    def testIMU(self):
        selectedIMUs = [imu for imu in self.IMUs if self.IMUs[imu].get() == 1]


    def addCheckBox(self, frame, IMUName, IMUVar, row, col):
        box = tk.Checkbutton(frame, text=IMUName,
                             variable=IMUVar,
                             onvalue=1,
                             offvalue=0)

        box.grid(column=col, row=row)
