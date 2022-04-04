import os
import numpy as np
import json


def main():
    sets = os.listdir(".")
    sets = [x for x in sets if os.path.isdir(x)]
    mat = np.empty((len(sets), 3, 3))
    dist = np.empty((len(sets), 5))
    for i, set in enumerate(sets):
        jsonFile = os.path.join(".", set, "flirCalib.json")
        with open(jsonFile) as f:
            jsonDict = json.load(f)
        mat[i, :, :] = jsonDict["CameraMatrix"]
        dist[i, :] = jsonDict["DistortionCoefficients"][0]

    mat_mean = np.mean(mat, axis=0)
    dist_mean = np.mean(dist, axis=0)
    filename = "flirCalib.json"
    with open(filename, 'w') as file:
        # Assigns labels to values to make JSON readable
        dumpDictionary = {
            'Format': 'OpenCV', 'Model': 'Rational',
            'CameraMatrix': mat_mean,
            'DistortionCoefficients': dist_mean}
        json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)


class NumpyEncoder(json.JSONEncoder):
    """
    # Encodes numpy arrays as normal lists. Otherwise, they cannot be encode to JSON. Used when invoking json.dump as
    cls param
    """

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


if __name__ == "__main__":
    main()
