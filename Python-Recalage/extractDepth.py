import pyk4a as k4a
import cv2
from typing import Optional, Tuple
import numpy as np
import videostream

def colorize(
        image: np.ndarray,
        clipping_range: Tuple[Optional[int], Optional[int]] = (None, None),
        colormap: int = cv2.COLORMAP_HSV,
) -> np.ndarray:
    if clipping_range[0] or clipping_range[1]:
        img = image.clip(clipping_range[0], clipping_range[1])
    else:
        img = image.copy()
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    img = cv2.applyColorMap(img, colormap)
    return img


def main():
    cameras = 'K'
    # stream, _ = videostream.selectStreams(cameras)
    # stream = stream[0]
    stream = videostream.KinectVideoStream()
    while True:
        capture = stream.read()
        if capture.depth is not None:
            cv2.imshow("Depth", colorize(capture.depth, (None, 5000), colormap=cv2.COLORMAP_BONE))
            # cv2.imshow("Depth", capture.depth)
        # if capture.ir is not None:
        #     cv2.imshow("IR", colorize(capture.ir, (None, 500), colormap=cv2.COLORMAP_JET))
        if capture.color is not None:
            cv2.imshow("Color", capture.color)
        if capture.transformed_depth is not None:
            cv2.imshow("Transformed Depth", colorize(capture.transformed_depth, (None, 5000), cv2.COLORMAP_BONE))
            # cv2.imshow("Transformed Depth", capture.transformed_depth)
        if capture.transformed_color is not None:
            cv2.imshow("Transformed Color", capture.transformed_color)
        # if capture.transformed_ir is not None:
        #     cv2.imshow("Transformed IR", colorize(capture.transformed_ir, (None, 500), colormap=cv2.COLORMAP_JET))

        key = cv2.waitKey(10)
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
        print("Transformed Depth", capture.transformed_depth.shape)
        print("Color", capture.color.shape)
        print("Depth", capture.depth.shape)


    pass


if __name__ == '__main__':
    main()
