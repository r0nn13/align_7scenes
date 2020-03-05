import argparse
import cv2
import numpy as np
# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPt = []
isDepth = False
def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global refPt, cropping
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt.append((x, y))

    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        # draw a rectangle around the region of interest
        if isDepth:
            tmp = depthC
        else:
            tmp = image

        c = (0,255,0)
        for i in range(len(refPt)):
            if i>4:
                c = (0,0,255)
            cv2.circle(tmp, refPt[i],5,c, thickness=3)
        cv2.imshow("image", tmp)

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="Path to the image")
args = vars(ap.parse_args())

# load the image, clone it, and setup the mouse callback function
image = cv2.imread(args["image"],cv2.IMREAD_UNCHANGED)
depth = cv2.imread(args["image"].replace('color','depth'),cv2.IMREAD_UNCHANGED).astype(np.float)
depth[depth <1] = np.nan
depth[depth > 10000] = np.nan
depth = depth/1000.0
depthC = cv2.applyColorMap(np.uint8(depth/5.0*255.0), cv2.COLORMAP_PLASMA)
clone = image.copy()

cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)
# keep looping until the 'q' key is pressed

while True:
    # display the image and wait for a keypress
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF
    # if the 'c' key is pressed, break from the loop
    if key == ord("c"):
        break

print(refPt)
rgbRefPt = refPt

#image = depthC
isDepth = True

cv2.namedWindow("depth")
cv2.setMouseCallback("depth", click_and_crop)

while True:
    # display the image and wait for a keypress
    cv2.imshow("depth", depthC)
    key = cv2.waitKey(1) & 0xFF
    # if the 'c' key is pressed, break from the loop
    if key == ord("c"):
        break
print(refPt)

# if there are two reference points, then crop the region of interest
# from teh image and display it
# if len(refPt) == 2:
#     roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
#     cv2.imshow("ROI", roi)
#     cv2.waitKey(0)
# close all open windows
cv2.destroyAllWindows()
