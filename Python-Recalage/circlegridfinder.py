import numpy as np
import cv2 as cv
import time
from threading import Thread
# Local modules
from videostream import StreamType

class CircleGridFinder:
    """Finds circle grids in 1 or more target cameras

    Spawns a separate thread to take frames from target camera(s) and 
    search each one for a grid of circles, using a CircleDetector
    and OpenCV's cv.findCirclesGrid(). When finished, contains a list of 
    objectPoints and corresponding imagePoints for calibration/rectification.

    Attributes
    ----------
    streams : list of VideoStream
        Streams in which to search for circle grids
        If there is more than 1 stream, grid will only be saved if found in 
        all streams simultaneously
    streamTypes : list of StreamTypes
        Type of corresponding streams (rgb or ir)
    circleDetectors : list of CircleDetectors
        CircleDetectors, in a corresponding list to streams
        Each detector can be different for better detection depending on stream
    numGrids : int
        Number of grids to find in the streams
    secondsToSkip : float/int
        Number of seconds to ignore streams before searching for grids again
        Used to allow user to move grid between captures
        Otherwise, all grids will be found almost instantly, in the same area
    rows, cols : int, int
        Dimensions of circle grid used to calibrate
    objectCorners : np array of 3D points
        Position of circles on grid used for calibration in 3D space
        z is always 0, as grid is a flat plane 
        (0, 0) is the first circle, other points are relative to first circle
        Dimensions are in mm
    objectPositions : list of np array of 2D points
        Stores position of circles in 3D space on identified grids
        1 list for all streams, as 3D positions are identical for all streams
    allImagePositions : list of list of np array of 2D points
        Stores position of circles in images on identified grids
        1 x list of np arrays is generated per stream
    running : bool
        Indicates if _find thread is still running
    finished : bool
        Indicates if _find thread has found numGrids grids and exited

    Parameters
    ----------
    See attributes above.

    Methods
    -------
    start()
        Begin _find() thread to search for circle grids
    stop()
        Close _find() thread, even if all grids not found
    drawCircles(frames)
        Search for and display circles in given frames
    drawOutlines(frames)
        Draws outlines from allImagePositions on corresponding frames
    len()
        Returns length of objectPositions, aka number of grids found
    """
    def __init__(self, streams, streamTypes, circleDetectors, numGrids, 
                 secondsToSkip=2, numRows=9, numCols=15):
        if len(streams) != len(streamTypes) or \
           len(streams) != len(circleDetectors):
            raise ValueError("Length of argument lists do not match.")
        self.streams, = streams
        self.streamTypes = streamTypes
        self.circleDetectors = circleDetectors
        self.numGrids = numGrids
        self.secondsToSkip = secondsToSkip
        self.numRows, self.numCols = numRows, numCols
        self.boardSize = (numRows, numCols)
        # X axis (columns) increments 0, 1, 2, ... numberOfColumns-1,
        # repeating numRows times
        # Y axis (rows) increments 0, 1, 2, ... numberOfRows-1
        # 21.2 mm is distance between circles in current grid
        self.objectCorners = 21.2 * np.array([[2 * (x % numRows)
            + np.floor(x/numRows) % 2, np.floor(x / numRows), 0]
            for x in range(numRows * numCols)], np.float32)
        self.objectPositions = []
        self.allImagePositions = [[] for i in streams]
        # Variable for handling thread
        self.running = False
        self.finished = False

    def start(self):
        """Begin _find() thread to search for circle grids

        Returns
        -------
        None.
        """
        
        if not self.running:
            self.running = True
            self.finished = False
            Thread(target=self._find, args=(), daemon=True).start()
        else:
            raise RuntimeError(
                "Tried to start gridFinder while still running.")

    def stop(self):
        """Close _find() thread, even if all grids not found

        Returns
        -------
        None.
        """
        self.running = False

    def _find(self):
        """Searches for circle grids in streams, using specified parameters

        Places found grids in objectPositions and allImagePositions.
        Stops when all grids are found or streams are closed.
        If multiple streams, waits to find grid in all streams
        before appending it.

        Returns
        -------
        None.
        """
        # Checks that thread has not been manually stopped, 
        # and that there are not enough images yet
        while self.running and len(self.objectPositions) < self.numGrids:
            frames = []
            # Gets frames from image streams before any processing 
            # to ensure synchronicity
            for stream in self.streams:
                # Checks that streams are still open
                if stream.stopped:
                    self.running = False
                    return                    
                else:
                    frames.append(stream.read())

            # List of grid positions in each frame of all streams
            tempGridPositions = []
            boardsFound = True
            # Checks for circles grid in each frame
            for (frame, circleDetector, streamType) in zip(
                    frames, self.circleDetectors, self.streamTypes):
                # Changes frame to single channel, 
                # either (red - blue) if rgb or direct grayscale if IR
                frame = self._grayify(frame, streamType)
                # Function searches for circleGrids using circleDetector
                # Use CALIB_CB_SYMMETRIC_GRID for grid of parallel rows 
                # and cols, CALIB_CB_CLUSTERING for quicker results
                # Warning : CALIB_CB_CLUSTERING can cause incorrect grids
                # that shift circle rows
                ret, pos = cv.findCirclesGrid(frame, self.boardSize, 
                    flags=cv.CALIB_CB_ASYMMETRIC_GRID, 
                    blobDetector=circleDetector.get())
                if not ret:
                    # If not found, indicates in boardsFound
                    boardsFound = False
                else:
                    tempGridPositions.append(pos)

            # Only adds grids to main list if grids found in all images
            if boardsFound:
                self.objectPositions.append(self.objectCorners)
                # Adds grids for each camera 
                for (gridPosition, imagePositions) in zip(
                        tempGridPositions, self.allImagePositions):
                    imagePositions.append(gridPosition)
                # If a grid is found, waits for secondsToSkip 
                # so user can reposition grid
                time.sleep(self.secondsToSkip)
            else:
                # Otherwise, repeats immediately 
                # 10 ms delay prevents thread from taking 100% of process
                cv.waitKey(10)
        
        # When all images found, indicates it has finished and stops thread
        self.running = False
        self.finished = True

    def _grayify(self, frame, frameType):
        # Turns frames gray based on frame type
        if frameType == "rgb" or frameType == StreamType.rgb:
            return cv.subtract(frame[:,:,2], frame[:,:,0])
        elif frameType == "ir" or frameType == StreamType.ir:
            return cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    def drawCircles(self, frames):
        """Search for and display circles in given frames

        Affects performance because detecting circles is time-consuming.
        Drawing is quick.

        Parameters
        ----------
        frames : list of OpenCV images
            Frames in which to find and draw circles in place
            Be careful not to draw circles multiple times on same frame
            Assumes frames are in same order as streamTypes and circleDetectors

        Returns
        -------
        None.
        """
        # Assumes if multiple frames, there are corresponding multiple 
        # circleDetectors and streamTypes
        for (frame, frameType, circleDetector, imagePosition) in zip(
                frames, self.streamTypes, self.circleDetectors, 
                self.allImagePositions):
            # Detects circles in gray version of frame
            grayFrame = self._grayify(frame, frameType)
            keypoints = circleDetector.get().detect(grayFrame)
            # Draws circles in place on frames
            frame = cv.drawKeypoints(frame, keypoints, frame, (255, 255, 255),
                cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS 
                + cv.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG)
            # Displays special color graphics for latest grid found
            if imagePosition:
                cv.drawChessboardCorners(
                    frame, self.boardSize, imagePosition[-1], True)

    def drawOutlines(self, frames):
        """Draws outlines from allImagePositions on corresponding frames

        Small performance impact because drawing lines is simple.

        Parameters
        ----------
        frames : list of OpenCV images
            Frame on which to draw outlines, in place
            Assumes frames are in same order as streams, as grid positions
            vary depending on the stream

        Returns
        -------
        None.
        """
        # Draws previously used checkerboards using OpenCV lines in frame
        # Assumes if multiple frames, there are corresponding 
        # multiple imagePosition lists
        for (frame, imagePositions) in zip(frames, self.allImagePositions):
            # Draws each position (grid) in imagePositions 
            # to the corresponding frame
            for pos in imagePositions:
                # Uses corners of each calibration grid as points for lines
                points = np.array([pos[0], pos[self.numRows - 1], 
                    pos[self.numCols*self.numRows - 1], 
                    pos[self.numRows*(self.numCols - 1)]], np.int32) 
                # Draws lines in place on arguments
                cv.polylines(frame, [points], True, (20, 220, 90), 2)

    def len(self):
        """ Returns length of objectPositions, aka number of grids found
        """
        return len(self.objectPositions)
