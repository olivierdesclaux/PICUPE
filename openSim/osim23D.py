import opensim as osim
import numpy as np
import os
import pandas as pd
import json
from tqdm import tqdm
from json import JSONEncoder


class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)


def readMot(filePath):
    """
    Function for reading the orientation data stored in a .sto file as a pandas dataframe.
    Parameters
    ----------
    filePath: str, path to the orientations.sto file.

    Returns
    -------
    res: pd.DataFrame.
    """
    # Read file
    with open(filePath, "r") as f:
        lines = f.readlines()
    keys = lines[6]  # First 6 lines is just a header.
    keys = keys.split("\n")[0]  # Remove \n at the end of line
    keys = keys.split("\t")
    res = []
    for i in range(7, len(lines)):
        line = lines[i]
        line = line.split("\n")[0]  # Remove \n at the end of line
        line = line.split("\t")  # Remove tabs
        res.append(line)

    res = np.array(res)
    res = pd.DataFrame(res, columns=keys)
    res.set_index("time", inplace=True)

    # Data stored in .sto file is in degrees, but we want them in radians. EXCEPT for the pelvis translation vectors,
    # which aren't angles but distances.
    for col in res.columns:
        res[col] = res[col].astype("float")
        if col in ["pelvis_tx", "pelvis_ty", "pelvis_tz"]:
            continue
        else:
            res[col] = np.radians(res[col])
    return res


def computeMarker3D(model, experimentPath):
    """
    Computes the positions through time of markers on an .osim model that is animated by a .sto file. This is LONG. 
    Parameters
    ----------
    model: str, path to the .osim model used. Must contain a subset of markers.
    experimentPath: str, path to the experiment folder. Will automatically go fetch the joint angle data stored in
    the openSim/ik_orientations.mot file/

    Returns
    -------
    True when finished.
    """
    myModel = osim.Model(model)  # Load model
    motion = os.path.join(experimentPath, "openSim", "ik_MT_01200452_orientations.mot")
    if not os.path.isfile(motion):
        raise ValueError(".mot file not found. Have you computed Inverse Kinematics?")
    table = readMot(filePath=motion)  # Load motion data.
    state = myModel.initSystem()
    res = {}
    # We perform iteratively (i.e. row by row)
    for rowIndex in tqdm(table.index):
        row = table.loc[rowIndex]
        # Update .osim model
        for joint in myModel.getJointSet():
            for j in range(joint.numCoordinates()):
                coord = joint.get_coordinates(j)  # Get a joint
                coord.setValue(state, row[coord.toString()])  # Update it's value with the new joint angle.
        # Compute marker positions
        positions = {}
        for i in myModel.getMarkerSet():
            markerName = i.toString()
            markerPosition = i.getLocationInGround(state)
            positions[markerName] = markerPosition.to_numpy() * 100  # Convert to centimeters

        # Update results dict
        res[rowIndex] = positions

    # Store results.
    with open(os.path.join(experimentPath, "openSim", "markerPositions.json"), "w") as f:
        json.dump(res, f, cls=NumpyArrayEncoder)

    return True

def main():
    model = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\openSim\config\calibrated_Rajagopal_2015.osim"
    experiment = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Acquisitions\OlivierNoHeadReset"
    motion = os.path.join(experiment, "openSim", "ik_MT_01200452_orientations.mot")
    computeMarker3D(model, experiment)


if __name__ == "__main__":
    main()
