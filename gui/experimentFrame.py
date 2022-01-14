import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import datetime

class experimentFrame(ttk.LabelFrame):
    def __init__(self, parent, row, column, rowspan, columnspan):
        super(experimentFrame, self).__init__(text="Experiment Information")
        self.parent = parent
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
        self.saveDir = self.getSaveDir()
        self.saveDirEntry = tk.Entry(self, textvariable=self.saveDirVar)
        self.saveDirEntry.grid(row=2, column=1)
        self.saveDirButton = ttk.Button(self, text="Browse", command=self.browseSaveDir)
        self.saveDirButton.grid(row=2, column=2)

    def getSaveDir(self):
        return self.saveDirVar.get()

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
