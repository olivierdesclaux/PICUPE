import os
import json
import numpy as np
import datetime
import pandas as pd


def mapImuTs2MotTs(path2Exp):
    """
    For linking absolute timestamps of the .txt file of an XSens and the relative timestamps of .sto file
    Parameters
    ----------
    path2Exp: str, path to the experiment data.

    Returns
    -------
    txt2sto: dict, keys are imu timestamps, values are associated .mot timestamps.
    Also writes reuslts to a .json file.
    """

    path2MTwData = os.path.join(path2Exp, "XSens", "MTw Data")
    path2mot = os.path.join(path2Exp, "openSim", "ik_MT_01200452_orientations.mot")
    imuTimestamps = makeIMUTimestamps(path2MTwData)

    # Extracting .sto timestamps
    with open(path2mot, "r") as f:
        lines = f.readlines()
        lines = lines[7:]
        lines = [x.split("\t") for x in lines]
        motTimestamps = [x[0] for x in lines]

    # the timestamps in the .mot file start at 0.0s and then are incremented linearly with the frame rate. Since we
    # do all our acquisitions at 60Hz, each timestamp is just it's index in the experiment /60.
    baseMotIndex = int(float(motTimestamps[0]) * 60)
    motIndexes = list(range(baseMotIndex, baseMotIndex + len(motTimestamps)))
    res = {}
    for index, ts in zip(motIndexes, motTimestamps):
        res[imuTimestamps[index].strftime("%Y:%m:%d:%H:%M:%S.%f")] = ts

    with open(os.path.join(path2Exp, "IMU2mot.json"), "w") as fp:
        json.dump(res, fp)

    return res


def makeIMUTimestamps(path2MTwData):
    """
    For generating a single list of timestamps fron the different imus. All imus have a slightly different timestamp,
    with a few milliseconds difference. We take the median timestamp as our reference.
    Parameters
    ----------
    path2MTwData: str, path to the "MTw data" directory

    Returns
    -------
    imuTimestamps: list, list of datetime objects.
    """

    print("Reading imu timestamps...")
    txtTimestamps = []
    imuList = os.listdir(path2MTwData)
    for imu in imuList:
        with open(os.path.join(path2MTwData, imu), "r") as f:
            lines = f.readlines()
            lines = lines[6:]  # Remove header
            lines = [x.split("\t") for x in lines]
            lines = [":".join(x[8:14]) for x in lines]  # Time info is stored there.
            lines = [datetime.datetime.strptime(x, '%Y:%m:%d:%H:%M:%S.%f') for x in lines]  # Convert to datetime obj.
            txtTimestamps.append(lines)
    # We now want to take the median timestamp between all the imus. However, we can't take the median of a list of
    # datetime objects. So we convert timestamps to pd.Timestamp objects.
    df = pd.DataFrame(txtTimestamps, index=imuList)
    imuTimestamps = [pd.Timestamp(df[col].astype(np.int64).median()) for col in df.columns]

    # And convert back to datetime.
    imuTimestamps = [x.to_pydatetime() for x in imuTimestamps]

    print("Done.")
    return imuTimestamps


def mapCameraTs2ImuTs(path2Exp):
    """
    Associates list of camera timestamps to list of imuTimestamps. For each frame (i.e. cameraTimestamp) we find the
    closest associated imuTimestamp. Let's say we map cameraTimestamp i to imuTimestamp j. Then for the
    cameraTimestamp (i+1), we only look in the range (j+1, j+10) for the closest timestamp. No need to look before j,
    because timestamps are in increasing order. No need to look way too far ahead in time, because the imus are
    sampled at 60 Hz, and the cameras work at 30 Hz.
    Parameters
    ----------
    path2Exp: str, path to experiment

    Returns
    -------
    mappings: dict, keys are camera timestamps, values are closest imu timestamp.
    Saves dict in a .json file.
    """
    # Generate both time stamps
    camTimestamps, _ = makeCameraTimestamps(path2Exp)
    path2MTwData = os.path.join(path2Exp, "XSens", "MTw data")
    imuTimestamps = makeIMUTimestamps(path2MTwData)

    # Initialise results
    # A neighbour is the closest imuTimestamp from a specific camera timestamp.
    mappings = {}
    neighbours = {}
    for i, camTs in enumerate(camTimestamps):
        # For the first frame, we look for the closest match across all imus.
        if i == 0:
            distances = np.array([abs(neighbour - camTs) for neighbour in imuTimestamps])
            closestNeighbourIndex = np.argmin(distances)
            closestNeighbour = imuTimestamps[closestNeighbourIndex]
            # Save neighbour coordinate.
            neighbours[i] = closestNeighbourIndex
            mappings[camTs.strftime("%Y:%m:%d:%H:%M:%S.%f")] = closestNeighbour.strftime("%Y:%m:%d:%H:%M:%S.%f")
        else:
            # Look after the argmin of mappings[i-1]
            previousNeighbour = neighbours[i - 1]
            distances = np.array([abs(neighbour - camTs) for neighbour in imuTimestamps[
                                                                          previousNeighbour + 1:previousNeighbour +
                                                                                                10]])
            closestNeighbourIndex = previousNeighbour + np.argmin(distances) + 1
            closestNeighbour = imuTimestamps[closestNeighbourIndex]
            neighbours[i] = closestNeighbourIndex
            mappings[camTs.strftime("%Y:%m:%d:%H:%M:%S.%f")] = closestNeighbour.strftime("%Y:%m:%d:%H:%M:%S.%f")

    # Save results in .json
    with open(os.path.join(path2Exp, "CameraTs2ImuTs.json"), "w") as fp:
        json.dump(mappings, fp)
    return mappings


def makeCameraTimestamps(path2Exp):
    """
    Both cameras have different, but very close, timestamps for each frame. We want to create a single timestamp
    representing each frame, so we take the mean timestamp between both cameras.
    Parameters
    ----------
    path2Exp: str, path to experiment data.

    Returns
    -------
    ts: list, the list of the created timestamps
    mapping: dict, keys are frame indices, values are the associated timestamps.
    Writes the mappings to a .json file.
    """
    print("Reading camera timestamps...")
    # Read timestamps
    rgbTime = txtToTime(os.path.join(path2Exp, "rgb", "timestamps.txt"))
    flirTime = txtToTime(os.path.join(path2Exp, "flir", "timestamps.txt"))
    # Initialise results
    ts = []
    mapping = {}
    for i, (rgb, flir) in enumerate(zip(rgbTime, flirTime)):
        minTime = min(rgb, flir)
        maxTime = max(rgb, flir)
        newT = minTime + (maxTime - minTime) / 2
        ts.append(newT)
        mapping[str(i).zfill(5)] = newT.strftime("%Y:%m:%d:%H:%M:%S.%f")

    # Save results
    with open(os.path.join(path2Exp, "Frame2Timestamp.json"), "w") as fp:
        json.dump(mapping, fp)
    print("Done.")
    return ts, mapping


def txtToTime(pathToFile):
    """
    Reads a timestamps.txt file and creates a list of datetime objects
    Parameters
    ----------
    pathToFile: str,path to the the timestampts.txt file

    Returns
    -------
    L: list, list of the datetime objects
    """
    L = []
    with open(pathToFile, 'r') as f:
        L = f.readlines()
    # In one single pass, we clean up the string by removing new lines and whitespaces
    # And we also convert it to a datetime object
    L = [datetime.datetime.strptime(x.lstrip().rstrip('\n').rstrip(), "%Y-%m-%d %H:%M:%S.%f") for x in L if x.rstrip()]

    return L


def mapMot2Frame(path2Exp):
    """
    Maps .mot timestamps to frame indices and inversely.
    Parameters
    ----------
    path2Exp: str, path to experiment.

    Returns
    -------
    frames2Mot: dict, keys are frame indices, values are .mot timestamps.

    """
    # Compute all mappings
    ImuTs2MotTs = mapImuTs2MotTs(path2Exp)
    camTs2ImuTs = mapCameraTs2ImuTs(path2Exp)
    _, camFrames2camTs = makeCameraTimestamps(path2Exp)
    frames2Mot = {}
    for frame in camFrames2camTs:
        # It's possible that the IK hasn't been performed on the entire experiment but just a subset. For ex,
        # between the seconds 20 and 30. So there might be no mapping for certain frames. In our ex, the first (20
        # *30)=600 frames approx. wouldn't have an associated motion. So we store a None for that key.
        try:
            camTs = camFrames2camTs[frame]
            imuTs = camTs2ImuTs[camTs]
            motTs = ImuTs2MotTs[imuTs]
            frames2Mot[frame] = motTs
        except KeyError:
            frames2Mot[frame] = None

    # Save results
    with open(os.path.join(path2Exp, "frames2mot.json"), "w") as f:
        json.dump(frames2Mot, f)

    # We then revert the dictionary, setting the mot timestamps as keys, and the associated frames as value. This
    # dict will be useful for visualising data.
    mot2frames = {}
    for key in frames2Mot.keys():
        if frames2Mot[key] is not None:
            mot2frames[frames2Mot[key]] = key

    # Save results
    with open(os.path.join(path2Exp, "mot2frames.json"), "w") as f:
        json.dump(mot2frames, f)

    return frames2Mot


if __name__ == "__main__":
    path2mot = r"C:\Users\Recherche\OneDrive - " \
               r"polymtl.ca\PICUPE\Results\Acquisitions\OlivierNoHeadReset\openSim\ik_MT_01200452_orientations.mot"
    path2txt = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Acquisitions\OlivierNoHeadReset\XSens\MTw data"
    path2Exp = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Acquisitions\OlivierNoHeadReset"
    # mapIMU2Mot(path2txt, path2mot)
    # makeCameraTimestamps(path2Exp)
    # mapCameraTs2ImuTs(path2Exp)
    mapMot2Frame(path2Exp)
