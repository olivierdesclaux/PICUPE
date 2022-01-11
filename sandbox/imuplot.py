import matplotlib.pyplot as plt
import opensim as osim
import numpy as np
import os
import matplotlib.animation as animation


def computeAccelerations(imuNum):
    IMUData = r"C:\Users\picup\Desktop\PICUPE\sandbox\results\2021-10-18_15-41\XSens\MTw data/"
    IMUNames = [x.split(".")[0] for x in os.listdir(IMUData)]
    IMUAdress = [name.split('_')[-1] for name in IMUNames]
    # Instantiate an XsensDataReader
    xsensSettings = osim.XsensDataReaderSettings()
    xsensSettings.set_data_folder(IMUData)
    # Append the IMU Names and their ID (name is the adress, e.g. 00B48765, and ID is just an int)
    for i, name in enumerate(IMUNames):
        xsensSettings.append_ExperimentalSensors(osim.ExperimentalSensor(name, str(i) + "bla"))
    xsens = osim.XsensDataReader(xsensSettings)

    # Read in separate tables of data from the specified IMU file(s)
    tables = xsens.read(IMUData)
    accelTable = xsens.getLinearAccelerationsTable(tables)

    # Extract acceleration data along the different axis.
    nRows = accelTable.getNumRows()
    XAccelData = np.zeros(nRows)
    YAccelData = np.zeros(nRows)
    ZAccelData = np.zeros(nRows)

    for i in range(nRows):
        row = accelTable.getRowAtIndex(i)
        r = row[imuNum]
        XAccelData[i] = r[0]
        YAccelData[i] = r[1]
        ZAccelData[i] = r[2]
    return IMUAdress[imuNum], XAccelData, YAccelData, ZAccelData


def initPlot(imuName):
    global plots
    global data
    global t
    f, ax = plt.subplots(1, figsize=(8, 4))
    plots = {}
    data = {}
    t = [0]  # Time variable

    # Set up figure
    ax.set_ylim(-8, 20)
    ax.set_xlim(0, nRows)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Acceleration")
    ax.title.set_text("IMU Address: {}".format(imuName))
    ax.grid(linestyle='--')
    for coord in ['X', 'Y', 'Z']:
        data[coord] = [0]
        plots[coord] = ax.plot(t, data[coord], label=coord)[0]
    ax.legend(loc="upper left")
    return f, t, plots, data


def updateData(self):
    global cnt
    global plots
    global data
    global t
    global XAccelData
    global YAccelData
    global ZAccelData

    for coord, accel in zip(["X", "Y", "Z"], [XAccelData, YAccelData, ZAccelData]):
        data[coord].append(accel[cnt])
    t.append(t[-1]+1)
    cnt += 1

    for tmp in plots.keys():
        plots[tmp].set_data(t, data[tmp])


    return plots

def dynamicPlot(imuNum):
    global cnt
    global t
    global XAccelData
    global YAccelData
    global ZAccelData
    global nRows
    global plots
    global data
    cnt = 0
    IMUAdress, XAccelData, YAccelData, ZAccelData = computeAccelerations(imuNum)
    nRows = XAccelData.shape[0]
    f, t, plots, data = initPlot(IMUAdress)
    simulation = animation.FuncAnimation(f, updateData, blit=False, frames=nRows-1, interval=1, repeat=False)
    file = os.path.join(r"C:\Users\picup\Desktop\PICUPE\sandbox\results\2021-10-18_15-41", IMUAdress + ".avi")
    writervideo = animation.FFMpegWriter(fps=60)
    simulation.save(file, writer=writervideo)
    # plt.show()

def staticPlot(imuNum):
    pass


def main():
    pass


if __name__ == '__main__':
    main(0)