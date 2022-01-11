import os
import numpy as np
import datetime
import json
import argparse


def mapTimestamps(path):
    """
    Maps the timestamps between the rgb and flir cameras for a given experiment.
    """
    outputName = "rgb2flirMappings.json"
    outputPath = os.path.join(path, outputName)
    # Extract the rgb and flir data as datetime objects
    rgbTime = txtToTime(os.path.join(path, "rgb", "timestamps.txt"))
    flirTime = txtToTime(os.path.join(path, "flir", "timestamps.txt"))

    rgbToFlirMap = findClosestMatch(rgbTime, flirTime)

    with open(outputPath, "w") as outfile:
        json.dump(rgbToFlirMap, outfile)


def txtToTime(pathToFile):
    """
    Reads a timestamps.txt file and creates a liste of datetime objects
    :param pathToFile: path to the the timestampts.txt file
    :return L: list of the datetime objects
    """
    L = []
    with open(pathToFile, 'r') as f:
        L = f.readlines()
    # In one single pass, we clean up the string by removing new lines and whitespaces
    # And we also convert it to a datetime object
    L = [datetime.datetime.strptime(x.lstrip().rstrip('\n').rstrip(), "%Y-%m-%d %H:%M:%S.%f") for x in L if x.rstrip()]

    return L


def findClosestMatch(rgbTime, flirTime, windowSize=200):
    """"
    For each timestamp in rgbTime, we will find the closest timestamp in flirTime within a window of
    2*windowSize. We could of course look for the closest neighbour in all of flirTime, but that is inefficient,
    and takes roughly 10 seconds of processing time for a 1 minute acquisition.

    :param rgbTime: list of rgb timestamps, outputted from txtToTime() function
    :param flirTime: list of rgb timestamps, outputted from txtToTime() function
    :param windowSize: int, size of sliding window

    :return rgbToFlirMap: dict, keys are the rgb timestamps, values are the associated flir timestamps
    """
    nFlirImages = len(flirTime)
    rgbToFlirMap = {}  # Initialise output
    meanDelta = []  # For verbosity: will show average time difference between two mapped timestamps

    # Since the RGBD data interests us more, we will take the RGBD timestamps as  basis

    for i, t in enumerate(rgbTime):
        # We look for the closest timestamp in a sliding window having a maximum size of 2*windowSize
        # Find the window and make sure list indexing is not out of range.
        windowStart = min(nFlirImages - windowSize, max(0, i - windowSize))
        windowEnd = min(nFlirImages, i + windowSize)
        window = slice(windowStart, windowEnd)

        # Extract neighbourhood and compute distances
        neighbourhood = flirTime[window]
        distances = np.array([abs(neighbour - t) for neighbour in neighbourhood])

        # Find closest timestamp in neighbourhood
        posInWindow = np.argmin(distances)
        closestNeighbour = neighbourhood[posInWindow]

        # Find the associated image number
        absolutePos = windowStart + posInWindow

        meanDelta.append(abs(closestNeighbour - t))
        # Save result in dictionary
        rgbToFlirMap[t.strftime("%Y-%m-%d %H:%M:%S.%f")] = [str(i), str(absolutePos), closestNeighbour.strftime("%Y-%m-%d %H:%M:%S.%f")]

    keys = list(rgbToFlirMap)
    vals = list(rgbToFlirMap)
    for key in

    return rgbToFlirMap


def loadMappedImages(path):
    """

    Parameters
    ----------
    path: str, path to the directory storing the results from an experiment

    Returns
    -------

    """

if __name__ == "__main__":
    allResultsDir = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\sandbox\results"
    latestDir = os.path.join(allResultsDir, os.listdir(allResultsDir)[-1])

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--experimentDir', type=str, dest='experimentDir', default=latestDir,
                        help="Choose a directory storing the results from an experiment",
                        required=False)

    args = parser.parse_args().__dict__

    experimentDir = args['experimentDir']
    mapTimestamps(experimentDir)
