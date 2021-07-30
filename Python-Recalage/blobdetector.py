import cv2 as cv

class CircleDetector:
    def __init__(self, minThreshold = 30, maxThreshold = 240):
        # Blob detector
        blobParams = cv.SimpleBlobDetector_Params()
        blobParams.minDistBetweenBlobs = 4
        # Thresholds used to simplify image for detector
        blobParams.minThreshold = minThreshold
        blobParams.maxThreshold = maxThreshold

        # Filter by area
        blobParams.filterByArea = True
        blobParams.minArea = 25 # In pixels (circle diameter 5 px)
        blobParams.maxArea = 10000 # In pixels

        # Filter by circularity
        blobParams.filterByCircularity = True
        blobParams.minCircularity = 0.1 # Weak circularity filtering for perspective (circles become ellipses)

        # Filter by convexity
        blobParams.filterByConvexity = True
        blobParams.minConvexity = 0.92 # Strong convexity filtering for circles

        # Filter by inertia
        blobParams.filterByInertia = True
        blobParams.minInertiaRatio = 0.01 # Weak elongation filtering for perspective (circles become ellipses)

        # Create a detector with the parameters
        self.blobDetector = cv.SimpleBlobDetector_create(blobParams)
    
    def get(self):
        return self.blobDetector