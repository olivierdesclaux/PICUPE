import matplotlib.pyplot as plt
import numpy as np
import os
import json
import cv2.cv2 as cv2
from matplotlib import cm
import argparse


def viz3D(path2Exp):
    """
    Main function for visualising rgb and flir alongside the 3D markers foundthanks to opensim.
    Parameters
    ----------
    path2Exp: str, path to experiment data.

    Returns
    -------
    None
    """
    global keepGoing  #Global variable set to false if user clicks on the close window button.
    keepGoing = True
    n_colors = len(list(links.keys()))
    colors = cm.rainbow(np.linspace(0, 1, n_colors))

    plotData = preprocessPos(path2Exp)
    frames = preprocessFrames(path2Exp)

    with open(os.path.join(path2Exp, "mot2frames.json"), "r") as f:
        mot2frames = json.load(f)

    # Matplotlib figure
    fig = plt.figure(figsize=(10, 8))
    fig.canvas.mpl_connect('close_event', on_close)
    mngr = plt.get_current_fig_manager()
    mngr.window.setGeometry(1000, 200, 840, 750)

    ax = fig.add_subplot(1, 1, 1, projection='3d')
    # Label each axis
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # Set each axis limits
    ax.set_xlim((-50, 50))
    ax.set_zlim((0, 170))
    ax.set_ylim((-50, 50))
    ax.view_init(azim=0, elev=10)

    # Set opencv window
    window = cv2.namedWindow("Cameras")
    cv2.moveWindow("Cameras", 50, 100)

    # Init variables:
    lines = []

    for i, mot in enumerate(mot2frames.keys()):
        if not keepGoing:
            break
        frame = frames[i, :, :, :]
        cv2.imshow("Cameras", frame)

        data = plotData[mot]
        for line in lines:
            ax.lines.remove(line)
        lines = []
        for i, segment in enumerate(data):
            [x1, z1, y1], [x2, z2, y2] = segment  # Switch y and z axis for better view
            line, = ax.plot([x1, x2], [y1, y2], [z1, z2], color=tuple(colors[i, :]))
            lines.append(line)

        plt.draw()
        plt.pause(0.0001)
        if cv2.waitKey(1) == ord("q") or cv2.getWindowProperty('Cameras', cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


def preprocessFrames(path2Exp):
    """
    Preprocesses frames that we want to visualise. Reads, stacks and stores all frames in a big array,
    of shape (nFrames, shape(frame))
    Parameters
    ----------
    path2Exp: str. path to experiment data.

    Returns
    -------
    res: np.array uint8, stacked rgb and flir frames.
    """
    print("Preprocessing frames")
    with open(os.path.join(path2Exp, "mot2frames.json"), "r") as f:
        mot2frames = json.load(f)
    #  We have to compute the first frame, so that we know it's size. Then we can initialise the final array.
    firstMot = list(mot2frames.keys())[0]  # Get first timestamp
    firstFrame = mot2frames[firstMot] # Get associated first frame
    stacked = readAndStackFrame(path2Exp, firstFrame, firstMot) # Read images , stack them and reshape

    # Initialise output array
    stacked_shape = stacked.shape
    nMots = len(list(mot2frames.keys()))  # Number of .mot timestamps.
    res = np.empty((nMots,) + stacked_shape)
    # Iterate over timestamps
    for i, mot in enumerate(mot2frames.keys()):
        frame = mot2frames[mot]
        # Store results
        res[i, :, :, :] = readAndStackFrame(path2Exp, frame, mot)

    res = res.astype(np.uint8) # Is tyoe float64. Want type np.uint8 to visualize using cv2.imshow

    return res


def readAndStackFrame(path2Exp, frame, mot):
    """
    Reads rgb and flir frames of a specific index, stacks them and writes the associated motion timestamp.
    Parameters
    ----------
    path2Exp: str, path to experiment data
    frame: str, index of the frame we want to see (e.g. "00023")
    mot: float, mot timestamp associated to the frame.

    Returns
    -------
    imstack: np.array, resized and stack rgb and flir images. 
    """

    rgb = np.load(os.path.join(path2Exp, "rgb", frame))
    rgb = rgb[:, :, :3]  # Remove transparency
    flir = np.load(os.path.join(path2Exp, "flir", frame))
    flir = cv2.resize(flir, (1280, 720))
    imstack = np.vstack((rgb, flir))
    imstack = cv2.resize(imstack, None, fx=0.7, fy=0.7)

    # Write associated timestamp. Keep only the first 4 digits.
    cv2.putText(imstack, str(mot)[:4], (50, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)

    return imstack


def preprocessPos(path2Exp):
    """
    Preprocesses the marker 3D coordinates so that visualisation goes faster.
    Parameters
    ----------
    path2Exp: str, path to experiment

    Returns
    -------
    res: dict, keys are .mot timestamps, values are pairs of keypoints that make up a joint.
    """
    print("Preprocessing 3D data.")
    # Read data.
    pos3DPath = os.path.join(path2Exp, "openSim", "markerPositions.json")
    with open(pos3DPath, "r") as f:
        pos3D = json.load(f)
    with open(os.path.join(path2Exp, "mot2frames.json"), "r") as f:
        mot2frames = json.load(f)

    # Initialise results.
    res = {}
    for mot in mot2frames.keys():
        pos = pos3D[mot]
        plotData = []
        for jointName in links.keys():
            jointCoords = pos[jointName]
            connectedJoints = links[jointName]
            for joint2Name in connectedJoints:
                joint2Coords = pos[joint2Name]
                plotData.append([jointCoords, joint2Coords])
        res[mot] = plotData
    return res


def on_close(evt):
    """
    Closes the windows if a user presses the close button of the matplotlib window.
    Parameters
    ----------
    evt: triggered if user presses the close button.

    Returns
    -------
    None
    """
    global keepGoing
    keepGoing = False


# Dictionary that contains the segments between different markers. Is used to visualise results: instead of
# scattering markers, we draw the body segments associated. We proceed in a top-down approach, from head to toes. We
# don't put marker links twice, so that we don't plot lines twice. Ex: R.Shoulder is connected to R.Elbow because it is
# in its values. So in R.Elbow's values, we don't put R.Shoulder
links = {'Forehead': ['Sternum.Upper'],
         'Sternum.Upper': ['R.Shoulder', 'Sternum.Lower', 'L.Shoulder'],
         'R.Shoulder': ['R.Elbow'],
         'L.Shoulder': ['L.Elbow'],
         'R.Elbow': ['R.Wrist'],
         'L.Elbow': ['L.Wrist'],
         'R.Wrist': [],
         'L.Wrist': [],
         'Sternum.Lower': ['Belt'],
         'Belt': ['R.Hip', 'L.Hip'],
         'R.Hip': ['R.Knee'],
         'L.Hip': ['L.Knee'],
         'R.Knee': ['R.Ankle'],
         'R.Ankle': [],
         'L.Knee': ['L.Ankle'],
         'L.Ankle': [],
         }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('-d', '--directory', type=str, dest='directory', help='Select a results directory',
                        required=True)
    args = parser.parse_args().__dict__

    directory = args['directory']
    viz3D(directory)
