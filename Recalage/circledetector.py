import cv2.cv2 as cv2


class CircleDetector:
    """Creates a blobdetector tuned for circles/ovals

    Attributes
    ----------
    blobDetector : cv2.BlobDetector
        Object used to detect circles in images, for
        use in circleGridFinder

    Parameters
    ----------
    threshold : (int, int, int)
        Parameters for creation of a series of binary images
        using a binary threshold
        First is first binary image threshold
        Second is maximum binary image threshold
        Third is step between images (determines number of images as well)
    minArea
        Minimum area of circles detected by blobDetector
        Increase for fewer false positive, decrease to detect grids 
        from further away

    Methods
    -------
    get()
        Returns blobDetector with custom parameters
    """

    def __init__(self, thresh=(30, 181, 10), minArea=15):
        self.threshold = thresh
        # Blob detector
        blobParams = cv2.SimpleBlobDetector_Params()
        blobParams.minDistBetweenBlobs = 5
        # Thresholds used to simplify image for detector
        blobParams.minThreshold = self.threshold[0]
        blobParams.maxThreshold = self.threshold[1]
        blobParams.thresholdStep = self.threshold[2]

        # Filter by area
        blobParams.filterByArea = True
        blobParams.minArea = minArea  # In pixels
        blobParams.maxArea = 10000  # In pixels

        # Filter by circularity
        blobParams.filterByCircularity = True
        # Weak circularity filtering for perspective (circles become ellipses)
        blobParams.minCircularity = 0.1

        # Filter by convexity
        blobParams.filterByConvexity = True
        # Strong convexity filtering for circles
        blobParams.minConvexity = 0.92

        # Filter by inertia
        blobParams.filterByInertia = True
        # Weak elongation filtering for perspective (circles become ellipses)
        blobParams.minInertiaRatio = 0.01

        # Create a detector with the parameters
        self.blobDetector = cv2.SimpleBlobDetector_create(blobParams)

    def get(self):
        """ Returns initialized blobDetector
        """
        return self.blobDetector
