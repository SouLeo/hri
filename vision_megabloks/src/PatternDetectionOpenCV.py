#!/usr/bin/env python
import rospy
import numpy as np
import imutils
import cv2
import vision_megabloks.srv

def getColors(colorLetter, colorArray, colorArrayPose, image, kernel1, kernel2):
    for (lower, upper) in colorArray:
        # create NumPy arrays from the
        # boundaries
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        # find the colors within the specified
        #  boundaries and apply the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask=mask)
        imageOut = np.hstack([image, output])
        imageOut = cv2.cvtColor(imageOut, cv2.COLOR_BGR2GRAY)
        imageOut = cv2.threshold(imageOut, 10, 255, cv2.THRESH_BINARY)[1]
        imageOut = cv2.morphologyEx(imageOut, cv2.MORPH_CLOSE, kernel1)
        imageOut = cv2.dilate(imageOut, kernel1, iterations=1)
        imageOut = cv2.GaussianBlur(imageOut, (5, 5), 0)
        cnts = cv2.findContours(imageOut.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        for c in cnts:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            coords = tuple([colorLetter, cX, cY])
            colorArrayPose.append(coords)
            #cv2.circle(original, (cX,cY), 5, (255,0,0), -1)
            #cv2.imshow('RGB', imageOut)
        return colorArrayPose

class PatternDetectionOpenCV(object):
    """
        This class reports linear MegaBloks patterns
        by colors and x orientation
    """
    def __init__(self):
        self.kernel1 = np.ones((15, 15), np.uint8)
        self.kernel2 = np.ones((5, 5), np.uint8)

        self.red = []
        self.yellow = []
        self.blue = []
        self.green = []
        self.allPoints = []
        # define the list of boundaries
        self.red_boundaries = [([0, 5, 80], [15, 20, 255])]
        self.green_boundaries = [([90, 150, 90], [120, 170, 120])]
        self.blue_boundaries = [([100, 90, 5], [255, 120, 60])]
        self.yellow_boundaries = [([45, 170, 200], [80, 195, 240])]

        self.cap = cv2.VideoCapture(2)

    def execute(self):
        # Capture frame-by-frame
        ret, image = self.cap.read()
        blue = getColors('b', self.blue_boundaries, self.blue, image, self.kernel1, self.kernel2)

        self.blue = [i for i in self.blue if i[1] != 324 and i[2] != 239]
        self.blue = [i for i in self.blue if i[2] >= 180]
        self.blue.sort(key=lambda x: x[1])

        red = getColors('r', self.red_boundaries, self.red, image, self.kernel1, self.kernel2)

        self.red = [i for i in self.red if i[1] != 324 and i[2] != 239]
        self.red = [i for i in self.red if i[2] >= 180]
        self.red.sort(key=lambda x: x[1])

        yellow = getColors('y', self.yellow_boundaries, self.yellow, image, self.kernel1, self.kernel2)

        self.yellow = [i for i in self.yellow if i[1] != 324 and i[2] != 239]
        self.yellow = [i for i in self.yellow if i[2] >= 180]
        self.yellow.sort(key=lambda x: x[1])

        green = getColors('g', self.green_boundaries, self.green, image, self.kernel1, self.kernel2)

        self.green = [i for i in self.green if i[1] != 324 and i[2] != 239]
        self.green = [i for i in self.green if i[2] >= 180]
        self.green.sort(key=lambda x: x[1])

        allColors = self.blue + self.red + self.yellow + self.green
        allColors.sort(key=lambda x: x[1])

        print "order is: "
        for i, (a, b, c) in enumerate(allColors):
            print str(allColors[i][0]) + ", "
        
        del self.blue[:]
        del self.red[:]
        del self.yellow[:]
        del self.green[:]
        # Display the resulting frame


def main():
    rospy.init_node('vision_megablocks')
    pd = PatternDetectionOpenCV()
    while not rospy.is_shutdown():
        pd.execute()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    pd.cap.release()

if __name__ == '__main__':
    main()
