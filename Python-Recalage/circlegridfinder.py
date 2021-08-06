import numpy as np
import cv2 as cv
import time
from threading import Thread
# Local modules
from videostream import VideoStream, StreamType
from circledetector import CircleDetector

class CircleGridFinder:
    """Finds circle grids in 1 or more target images"""
    def __init__(self, name, streams, streamTypes, circleDetectors, minImages, secondsToSkip = 2, numRows = 12, numCols = 12):
        if len(streams) != len(streamTypes) or len(streams) != len(circleDetectors):
            raise ValueError("Length of argument lists do not match.")
        self.name, self.streams, self.streamTypes, self.circleDetectors, self.minImages, self.secondsToSkip = name, streams, streamTypes, circleDetectors, minImages, secondsToSkip
        # Array of float32 object points to add to objectPoints list
        # X axis (columns) increments 0, 1, 2, ... numberOfColumns-1, repeating numRows times
        # Y axis (rows) increments 0, 1, 2, ... numberOfRows-1
        self.objectCorners = np.array([[x % numCols, np.floor(x / numCols), 0] for x in range(numCols * numRows)], np.float32)
        self.numRows, self.numCols = numRows, numCols
        self.boardSize = (numRows, numCols)
        # Arrays to store object points and image points from all the images.
        self.objectPositions = [] #3d points in real world space
        self.allImagePositions = [[] for i in streams] # 2d points in image plane, one list for each camera
        # Variable for handling thread
        self.running = False
        self.finished = False

    def start(self):
        if not self.running:
            self.running = True
            self.finished = False
            Thread(target=self._find, args=(), daemon=True).start()
        else:
            raise RuntimeError("Tried to start gridFinder while still running.")

    def stop(self):
        self.running = False

    def _find(self):
        # Checks that thread has not been stopped, and that there are not enough images
        while self.running and len(self.objectPositions) < self.minImages:
            frames = []
            # Gets frames from image streams before processing (to ensure synchronicity)
            for (stream, streamType) in zip(self.streams, self.streamTypes):
                # Checks that streams are still open
                if stream.stopped:
                    self.running = False
                    return                    
                else:
                    frame = stream.read()
                    # Changes frame to single channel, either red - blue (rgb) or direct grayscale (ir)
                    frames.append(self.grayify(frame, streamType))
            # Checks for circles grid in each frame
            boardsFound = True
            # List of grid positions in each frame of all streams
            allGridPositions = []
            for (frame, circleDetector) in zip(frames, self.circleDetectors):
                # CALIB_CB_SYMMETRIC_GRID for grid of parallel rows and cols, CALIB_CB_CLUSTERING for quicker results
                ret, pos = cv.findCirclesGrid(frame, self.boardSize, flags=cv.CALIB_CB_SYMMETRIC_GRID+cv.CALIB_CB_CLUSTERING, blobDetector=circleDetector.get())
                if not ret:
                    # If not found, indicates in boardsFound
                    boardsFound = False
                else:
                    allGridPositions.append(pos)

            # Only adds grids to main list if grids found in all images
            if boardsFound:
                self.objectPositions.append(self.objectCorners)
                # Adds grids for each camera 
                for (gridPosition, imagePositions) in zip(allGridPositions, self.allImagePositions):
                    imagePositions.append(gridPosition)
                # If a grid is found, waits for secondsToSkip so user can reposition grid
                time.sleep(self.secondsToSkip)
            else:
                # Otherwise, repeats immediately (1 ms delay to avoid hanging process)
                cv.waitKey(1)
        
        # When all images found, indicates it has finished and stops thread
        self.running = False
        self.finished = True

    def grayify(self, frame, frameType):
        # Turns frames gray based on frame type
        if frameType == "rgb" or frameType == StreamType.rgb:
            return cv.subtract(frame[:,:,2], frame[:,:,0])
        elif frameType == "ir" or frameType == StreamType.ir:
            return cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    def drawCircles(self, frames):
        # Assumes if multiple frames, there are corresponding multiple circleDetectors
        for (frame, frameType, circleDetector) in zip(frames, self.streamTypes, self.circleDetectors):
            grayFrame = self.grayify(frame, frameType)
            keypoints = circleDetector.get().detect(grayFrame)
            # Draws circles in place on arguments
            frame = cv.drawKeypoints(frame, keypoints, frame, (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS + cv.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG)

    def drawOutlines(self, frames):
        # Draws previously used checkerboards using OpenCV lines in frame
        # Assumes if multiple frames, there are corresponding multiple imagePosition lists
        for (frame, imagePositions) in zip(frames, self.allImagePositions):
            # Draws each position (grid) in imagePositions to the corresponding frame
            for pos in imagePositions:
                # Uses corners of each calibration grid as points for lines
                points = np.array([pos[0], pos[self.numCols - 1], pos[self.numCols*self.numRows - 1], pos[self.numCols*(self.numRows - 1)]], np.int32) 
                # Draws lines in place on arguments
                cv.polylines(frame, [points], True, (20, 220, 90), 2)

    def len(self):
        return len(self.objectPositions)
