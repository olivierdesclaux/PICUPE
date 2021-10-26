import matplotlib.pyplot as plt
import opensim as osim
import numpy as np
import os
import matplotlib.animation as animation


def computeAccelerations():
    IMUData = r"C:\Users\picup\Desktop\PICUPE\sandbox\results\2021-10-18_15-41\XSens\MTw data/"
    IMUNames = [x.split(".")[0] for x in os.listdir(IMUData)]
    IMUAdress = [name.split('_')[-1] for name in IMUNames]
    nIMUs = len(IMUNames)

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
    XAccelData = np.zeros((nIMUs, nRows))
    YAccelData = np.zeros((nIMUs, nRows))
    ZAccelData = np.zeros((nIMUs, nRows))

    for i in range(nRows):
        row = accelTable.getRowAtIndex(i)
        for imu in range(nIMUs):
            r = row[imu]
            XAccelData[imu, i] = r[0]
            YAccelData[imu, i] = r[1]
            ZAccelData[imu, i] = r[2]

    return IMUAdress, XAccelData, YAccelData, ZAccelData


def initPlots():
    IMUAdress, X, _, _ = computeAccelerations()
    (nIMUs, nRows) = X.shape
    f, axs = plt.subplots(2, 4, figsize=(15, 8))
    # x_data, y_data = [[], []] * nIMUs
    global plots
    global data
    global t
    plots = {}
    data = {}
    t = [0]  # Time variable
    for k in range(nIMUs):
        row, col = k // 4, k % 4
        ax = axs[row, col]
        ax.set_ylim(-8, 20)
        ax.set_xlim(0, nRows)
        ax.title.set_text(IMUAdress[k])
        ax.grid(linestyle='--')
        for coord in ['X', 'Y', 'Z']:
            data[str(k) + coord] = [0]
            plots[str(k) + coord] = ax.plot(t, data[str(k) + coord], label=coord)[0]
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

    for k in range(8):
        for coord, accel in zip(["X", "Y", "Z"], [XAccelData, YAccelData, ZAccelData]):
            data[str(k) + coord].append(accel[k, cnt])
    t.append(t[-1] + 1)
    cnt += 1

    for tmp in plots.keys():
        plots[tmp].set_data(t, data[tmp])
    nFrames = XAccelData.shape[1]
    if cnt % (nFrames//5) == 0:
        print("Executed {} %... \n".format(100 * cnt//nFrames))

    return plots


def main():
    global cnt
    global plots
    global data
    global t
    f, t, plots, data = initPlots()
    global XAccelData
    global YAccelData
    global ZAccelData
    cnt=0
    IMUAdress, XAccelData, YAccelData, ZAccelData = computeAccelerations()
    nFrames = XAccelData.shape[1]
    simulation = animation.FuncAnimation(f, updateData, blit=False, frames=500, interval=1, repeat=False)
    f = r"C:\Users\picup\Desktop\PICUPE\sandbox\results\2021-10-18_15-41\animation.avi"
    writervideo = animation.FFMpegWriter(fps=60)
    # simulation.save(f, writer=writervideo)
    plt.show()


if __name__ == "__main__":
    main()