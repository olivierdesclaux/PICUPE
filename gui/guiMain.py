import sys
import os
import tkinter as tk
from tkinter import ttk
# from experimentFrame import experimentFrame
# from calibFrame import calibFrame
# from IMUFrame import IMUFrame
# from cameraFrame import cameraFrame
from gui.experimentFrame import experimentFrame
from gui.calibFrame import calibFrame
from gui.IMUFrame import IMUFrame
from gui.cameraFrame import cameraFrame


class Root(tk.Tk):
    def __init__(self):
        super(Root, self).__init__()
        self.title("PICUPE Configuration")
        self.minsize(640, 400)
        # self.wm_iconbitmap('icon.ico')
        self.experimentFrame = experimentFrame(self, row=0, column=0, rowspan=1, columnspan=1)
        self.calibFrame = calibFrame(self, row=8, column=0, rowspan=1, columnspan=1)
        self.imuFrame = IMUFrame(self, row=1, column=0, rowspan=6, columnspan=1)
        self.cameraFrame = cameraFrame(row=0, column=1, rowspan=7, columnspan=2)
        self.makeSummaryFrame(row=8, column=1, rowspan=1, columnspan=1)
        self.config = {}
        self.protocol("WM_DELETE_WINDOW", self.abortConfig)

    def makeSummaryFrame(self, row, column, rowspan, columnspan):
        self.summaryFrame = ttk.Frame(self)
        self.summaryFrame.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan)
        self.saveButton = ttk.Button(self.summaryFrame, text="Save Config", command=self.saveConfig)
        self.saveButton.grid(row=1, column=1)

        self.saveAndExitButton = ttk.Button(self.summaryFrame, text="Save and Launch", command=self.saveAndLaunch)
        self.saveAndExitButton.grid(row=1, column=2)

        self.abortButton = ttk.Button(self.summaryFrame, text="Abort", command=self.abortConfig)
        self.abortButton.grid(row=1, column=3)

    def abortConfig(self):
        sys.exit()

    def saveAndLaunch(self):
        self.saveConfig()
        self.checkExperimentName()
        self.checkCalibrationDir()
        self.checkIMUs()
        self.destroy()

    def saveConfig(self):
        self.config["Experiment Name"] = self.experimentFrame.getExpName()
        self.config["Results Directory"] = self.experimentFrame.saveDirVar.get()
        self.config["Calibration Directory"] = self.calibFrame.directory.get()
        self.config["IMUs"] = [imu for imu in self.imuFrame.IMUs if self.imuFrame.IMUs[imu].get() == 1]
        try:
            # Look if stereocalibration was performed. If so, we save the used stereocalibration hyperparameters.
            calibConfig = self.calibFrame.kinectCalibrationWindow.getConfigData()
        except:
            calibConfig = None
        self.config["Calibration Parameters"] = calibConfig
        print(self.config)

    def checkExperimentName(self):
        self.saveConfig()
        # if self.config == {}:
        #     self.saveConfig()
        if self.config["Experiment Name"] == "":
            raise Exception("Experiment Name is blank.")
        if self.config["Experiment Name"] in os.listdir(self.config["Results Directory"]):
            raise Exception("Experiment already exists.")

    def checkCalibrationDir(self):
        self.saveConfig()
        if self.config["Calibration Directory"] == "":
            raise Exception("No calibration directory specified.")

    def checkIMUs(self):
        self.saveConfig()
        if len(self.config["IMUs"]) == 0:
            raise Exception("No IMUs were specified.")



def main():
    os.environ["K4A_ENABLE_LOG_TO_STDOUT"] = "0"
    root = Root()
    root.mainloop()

    return root.config


if __name__ == "__main__":
    main()
