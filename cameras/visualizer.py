import os
import cv2
import multiprocessing

def visualizeResults(newSavePath):
    savePathDepth = os.path.join(newSavePath, "depth")
    savePathRGB = os.path.join(newSavePath, "rgb")
    savePathWebcam = os.path.join(newSavePath, "webcam")
    rgb = multiprocessing.Process(target=viz, args=(savePathRGB, "RGB"))
    depth = multiprocessing.Process(target=viz, args=(savePathDepth, "Depth"))
    webcam = multiprocessing.Process(target=viz, args=(savePathWebcam, "Webcam"))

    processes = [rgb, depth, webcam]
    for proc in processes:
        proc.start()

    for proc in processes:
        proc.join()


def viz(path, t):
    images =[os.path.join(path, image) for image in os.listdir(path)]
    cv2.namedWindow(t)  # Create a named window
    if t == "RGB":
        cv2.moveWindow(t, 40, 30)
    elif t == "Depth":
        cv2.moveWindow(t, 840, 30)
    elif t == "Webcam":
        cv2.moveWindow(t, 30, 800)
    for i in range(len(images)):
        if t == "Depth":
            frame =  cv2.imread(images[i], cv2.IMREAD_GRAYSCALE)
        else:
            frame = cv2.imread(images[i])
        cv2.imshow(t, frame)
        if cv2.waitKey(1) == ord("q"):
            break


if __name__ == "__main__":
    p = r"C:\Users\picup\Desktop\PICUPE\sandbox\results\28-09-2021_15-11"
    visualizeResults(p)
