import os
from utils.timestampMappers import makeIMUTimestamps, makeCameraTimestamps


def removeFlirFirstImage(experimentPath):
    """
    There is a slight issue with the flir. It acquires at the start an unnecessary image.
    This creates an offset of one image in the entire acquisition.
    So we remove the first FLIR image, and slide all the images "up".
    We also do the same thing for the timestamps
    Parameters
    ----------
    experimentPath: str, path to the experiment.
    Returns
    -------
    None
    """
    flirPath = os.path.join(experimentPath, "flir")
    # Remove first frame
    os.remove(os.path.join(flirPath, "00000"))
    # Slide all frames up i.e. frame i becomes frame i-1
    frames = [x for x in os.listdir(flirPath) if x.split('.')[-1] != "txt"]
    for i, frame in enumerate(frames):
        # i will start at 0, frames will start at 00001. So we just have to rename the frame with i.
        newName = str(i).zfill(5)
        os.rename(os.path.join(flirPath, frame), os.path.join(flirPath, newName))

    # Do the same thing for the timestamps
    timestamps = os.path.join(flirPath, "timestamps.txt")
    with open(timestamps, "r") as f:
        lines = f.readlines()
    lines = lines[1:]
    with open(timestamps, "w") as f:
        f.writelines(lines)
    return True


def removeFrames(savePath, n):
    """
    For each modality, removes the first n images (arbitrary value) to account for emptying camera buffers.
    Also, must slide the save FLIR images by one. i.e. Remove frame 00000, and frame 00001 becomes 00000 etc.
    Finally, must modify the associate timestamps.txt file
    Parameters
    ----------
    savePath: str, directory where data is stored
    n: int, number of frames to remove
    Returns
    -------
    None
    """

    # 1. First check if there is the same number of frames in each modality.
    if len(os.listdir(os.path.join(savePath, "flir"))) > len(os.listdir(os.path.join(savePath, "depth"))):
        removeFlirFirstImage(savePath)

    ims2remove = [str(i).zfill(5) for i in range(n)]
    for modality in ["depth", "rgb", "flir"]:
        modalityPath = os.path.join(savePath, modality)
        timestamps = os.path.join(modalityPath, "timestamps.txt")
        files2remove = [os.path.join(modalityPath, x) for x in ims2remove]
        # Remove images
        for x in files2remove:
            try:
                os.remove(x)
            except FileNotFoundError as e:
                print(e)
        # Rename existing images
        remainingImages = [x for x in os.listdir(modalityPath) if x.split(".")[-1] != "txt"]
        for i, frame in enumerate(remainingImages):
            # i will start at 0, frames will start at 00001. So we just have to rename the frame with i.
            newName = str(i).zfill(5)
            os.rename(os.path.join(modalityPath, frame), os.path.join(modalityPath, newName))

        # Update timestamps
        with open(timestamps, "r") as f:
            lines = f.readlines()
        lines = lines[n:]  # Remove first n lines (0 to n-1)
        with open(timestamps, "w") as f:
            f.writelines(lines)


def removeIMUTimestamps(path, n):
    """
    Removes the first n imu timestamps in the txt files stored in path.
    Parameters
    ----------
    path: str, path to the MTw data directory
    n: int, number of timestamps to remove

    Returns
    -------
    None
    """
    for imu in os.listdir(path):
        imuPath = os.path.join(path, imu)
        with open(imuPath, "r") as f:
            lines = f.readlines()
        # Careful the first 6 lines are important! They are the header
        header = lines[:6]
        lines2keep = lines[n + 6:]
        lines2keep = header + lines2keep
        with open(imuPath, "w") as f:
            f.writelines(lines2keep)
    return True


def setSameStart(experimentPath):
    """
    Make sure data from IMU acquisition and camera acquisition have the same start time. If not, iteratively removes
    the appropriate date.
    Parameters
    ----------
    experimentPath: str, path to experiment directory
    Returns
    -------
    None
    """
    imuPath = os.path.join(experimentPath, "XSens", "MTw data")
    imuTs = makeIMUTimestamps(imuPath)
    camTs, _ = makeCameraTimestamps(experimentPath)

    # Check which ts is bigger. If it is the IMUts that is bigger (i.e. later), we have to remove some images. If
    # it is the camTs which is bigger, we have to remove some imu data.
    if imuTs[0] > camTs[0]:
        i = 0
        while camTs[i] < imuTs[0]:
            i += 1
        print("Removing {} frames...".format(i-1))
        removeFrames(experimentPath, i-1)  # i necessarily greater than 1.
        print("Done")
    elif imuTs[0] < camTs[0]:
        i = 0
        while imuTs[i] < camTs[0]:
            i += 1
        print("Removing {} imu timestamps...".format(i-1))
        removeIMUTimestamps(imuPath, i - 1)  # i necessarily greater than 1.
        print("Done")
    else:
        print("Identical starting times, no cleanUp")


def cleanUp(experimentPath, n=5):
    """
    Clean Up the first n captures (i.e. frames and associated timestamps) of an acquisition, and the first 2*n imu
    acquisitions.
    This is done to remove all parasite data that can be held in buffers and stored.
    Parameters
    ----------
    experimentPath: str, path to experiment data
    n: int, number of datapoints we want to remove.

    Returns
    -------
    None
    """
    removeFrames(experimentPath, n)  # Remove 5 first frames to account for buffer cleaning
    imuPath = os.path.join(experimentPath, "XSens", "MTw data")
    removeIMUTimestamps(imuPath, 2*n)  # Remove 10 first imu datapoints to account for buffer cleaning


if __name__ == "__main__":
    path2exp = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\sandbox\results\Oliv mille - 22 mars"
    # experimentPath = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Acquisitions\OlivierNoHeadReset"
    # cleanUp(path2exp, 54)
    # setSameStart(path2exp)
