import math
import sys
from pyasn1.compat.octets import null
sys.path.append("/home/pi/opencv-3.1.0/build/lib")
import cv2
import numpy as np
from networktables import NetworkTables
import sys
import time
#basad
#Making The

#prints no retro found and sets the table angle to 0
def printNoRetroFound(x,counter3):
    if (counter3 is 0):
        print "no retro found"
#something with the sliders ask omer zip
def nothing(self):
    pass
#prints the angle
def printAngle(angle, counter2):
    if (counter2 is 1):
        print "angle:", angle
#calc the distance
def calcDistance(img, height, fov, widthIrl):
    imgHieght, imgWidth, _ = img.shape
    #the distance calc equation
    distance = (widthIrl / 2) * (imgHieght) / (height * math.tan(math.radians(fov) / 2))
    distance *= 4.546
    #distance *= cv2.getTrackbarPos('distanceMult', 'slider') / 100.0
    return abs(distance)

#makes the hsv picture
def makeHSV(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #lower_green = np.array([cv2.getTrackbarPos('Hmin','slider'), cv2.getTrackbarPos('Smin','slider'), cv2.getTrackbarPos('Vmin','slider')])
    #higher_green = np.array([cv2.getTrackbarPos('Hmax','slider'), cv2.getTrackbarPos('Smax','slider'), cv2.getTrackbarPos('Vmax','slider')])
    lower_green = np.array([40,141,78])
    higher_green = np.array([78,255,193])
    #makes the res image
    mask = cv2.inRange(hsv, lower_green, higher_green)
    res = cv2.bitwise_and(img, img, mask=mask)
    return res

#finds the contours
def findContours(img):
    im2, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

#calcs the distance and prints it once, related to the "Does everything"
def distanceCheck(img, width, fov, widthIrl, counter):
    if (counter is 1):
        imgHieght, imgWidth, _ = img.shape
        distance = (widthIrl / 2) * (imgHieght) / (width * math.tan(math.radians(fov) / 2))
        distance *= 4.546
        print "distance", distance
        return distance

#calcs the angle
def calcAngle(img,midX,tanFOV):
        imgHieght, imgWidth, _ = img.shape
        return math.degrees(math.atan(((imgWidth/2-midX)*2*tanFOV)/imgWidth))

#calcs the mid
def calcMid(b, d):
    y = (b[1] + d[1]) / 2
    x = (b[0] + d[0]) / 2
    return x, y

#returns contours/nothing
def returnCon(countours):
    area = 0
    area1 = 0
    area2 = 0
    x = 0
    con1 = None
    con2 = None
    #sorts the contours by size
    for i in countours:
        area = cv2.contourArea(i)
        if area > 100:
            if x is 0:
                con1 = i
                area1 = area
                con2 = None
                x += 1
            elif area > area1:
                con2 = con1
                area2 = area1
                con1 = i
                area1 = area
            elif area > area2:
                con2 = i
                area2 = area
    #returns the contours
    return x, con1, con2

#prints the images, including the sliders
def printImages(cap):
    cv2.imshow('slider', sliderImg)
    ret, img = cap.read()
    cv2.imshow("uncut image", img)
    res = makeHSV(img)
    cv2.imshow('res', res)
    return res,img

#makes the sliders
def sliders():
    cv2.namedWindow('slider')
    cv2.createTrackbar('Hmin', 'slider', 0, 255, nothing)
    cv2.createTrackbar('Hmax', 'slider', 255, 255, nothing)
    cv2.createTrackbar('Smin', 'slider', 0, 255, nothing)
    cv2.createTrackbar('Smax', 'slider', 255, 255, nothing)
    cv2.createTrackbar('Vmin', 'slider', 0, 255, nothing)
    cv2.createTrackbar('Vmax', 'slider', 255, 255, nothing)
    cv2.createTrackbar('distanceMult', 'slider', 185, 200, nothing)

#prints both distance and angle
def printBoth(distance, angle):
    print "distance: ", distance
    print "angle: ", angle

#puts both distance and angle into the tables
def tableBoth(distance, angle):
    table.putValue("angle", angle)
    table.putValue("distance", distance)

#return the rects, width the final height
def returnRectsAndLengths(con1, con2):
    #makes the rects
    rect1 = cv2.minAreaRect(con1)
    rect2 = cv2.minAreaRect(con2)
    #sorts the width and height
    width1 = min(rect1[1])
    height1 = max(rect1[1])
    height2 = max(rect2[1])
    finalHeight = (height1 + height2) / 2
    #returns them
    return rect1, rect2, width1, finalHeight

#draws the contours and gets the (x,y) of the avg between both of them
def drawCon(rect1, rect2, gray):
    box1 = np.int0(cv2.boxPoints(rect1))
    box2 = np.int0(cv2.boxPoints(rect2))
    cv2.drawContours(gray, [box1], 0, (255, 255, 255), 2)
    cv2.drawContours(gray, [box2], 0, (255, 255, 255), 2)
    midX, midY = calcMid(b=box1[1], d=box1[3])
    midX1, midY2 = calcMid(b=box2[1], d=box2[3])
    topMidX = (midX + midX1) / 2
    topMidY = (midY + midY2) / 2
    return topMidX, topMidY

#as simple as it sounds, it does everything. literally everything. this is very important. do not touch unless not working
def doesEverything(angle1, prevDistance, angle, distance, counter):
    #only prints the angle if the abs hisor of both of them is more than 2
    if (abs(angle1 - angle) < 2):
        #prints both angle and distance
        distanceCheck(img, height1, 47, 5, counter)
        printAngle(angle, counter)
        counter3 = 0
        counter = 0
    #checks for jumps in the distance calcs, should be fine now as it uses only the height.
    #it used both height and width last time by mistake
    else:
        angle1 = angle
        counter = 1
        counter3 = 0
        if (abs(distance - prevDistance) > 5 and prevDistance != 0):
            if (distance > prevDistance):
                #prints both of them
                printBoth(distance, angle)
                prevDistance = distance
            else:
                printBoth(distance, angle)
                # distance = prevDistance
        else:
            #prints both of them
            printBoth(distance, angle)
            prevDistance = distance
    #sets both of them for first time
    if (angle1 is 0):
        angle1 = angle
    if (prevDistance is 0):
        prevDistance = distance
    # prints the angle
    cv2.imshow("gray", gray)
    # if there isn't two contours
    counter3 = 0
    table.putValue("angle", angle)
    #return everything, puts the angle and distance in the table.
    return angle1, prevDistance, counter, counter3

#does the same thing as the other peolot, just for one contour
def onlyOneCon(con1):
    rect1 = cv2.minAreaRect(con1)
    height1 = max(rect1[1])
    box1 = np.int0(cv2.boxPoints(rect1))
    cv2.drawContours(gray, [box1], 0, (255, 255, 255), 2)
    midX, midY = calcMid(b=box1[1], d=box1[3])
    return midX, midY, height1

#connects to the camera, sets the angle to 0.
def netWorkTables():
    NetworkTables.setIPAddress("10.45.86.2")
    NetworkTables.setClientMode()
    NetworkTables.initialize()
    table = NetworkTables.getTable('imgProc')
    table.putValue("angle", 0)
    return table

#defines all vars to help the main.
def defineVar():
    angleCounter1 = 0
    angle1 = 0
    counter = 1
    counter2 = 1
    counter3 = 0
    prevDistance = 0
    checkiftrue = True
    #the tan fov of the camera
    tanFov = math.tan(math.radians(47 / 2))
    return tanFov, angleCounter1, angle1, counter, counter2, counter3, prevDistance, checkiftrue
#Main
if __name__ == '__main__':
    reload(sys)
    #network table aka smartdashboard, connects to the camera
    table = netWorkTables()
    cap = cv2.VideoCapture("http://root:root@10.45.86.12/mjpg/video.mjpg")
    #sliders
    sliderImg = np.zeros((1,300,3),np.uint8)
    #returns vars to help the main
    tanFov,angleCounter1, angle1, counter, counter2, counter3, prevDistance, checkiftrue = defineVar()

    #enters the inf loop
    while True:
        #prints the res, img and gray images
        res, img = printImages(cap)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        #gets the contours and sorts them from big to small
        contours = findContours(gray)
        x, con1, con2 = returnCon(contours)
        #if he finds at least one contour
        if(x!=0):
            #if he finds both contours
            if (con2 != None and con1 != None):
                #makes the rects around the contours, and gets back the avg of the heights.
                rect1, rect2, width1, finalHeight = returnRectsAndLengths(con1, con2)
                #the img height and width
                imgHieght, imgWidth, _ = img.shape
                #get the avg between both (x,y) of the rects
                topMidX, topMidY = drawCon(rect1, rect2, gray)
                #calcs the angle and the distance
                angle = calcAngle(img=img,midX=topMidX,tanFOV=tanFov)
                distance = calcDistance(img, finalHeight, 47, 5)
                #this only activates once, when both prevdistance and angle1 is 0
                if (checkiftrue is True):
                    angle1, prevDistance, counter, counter3 = doesEverything(0, 0, angle, distance, counter3, counter)
                    checkiftrue = False
                #enters every other time
                else:
                    angle1, prevDistance, counter, counter3 = doesEverything(angle1, prevDistance, angle, distance, counter3, counter)
            #when we find only one contour
            elif (con2 is None and con1 != None):
                #gets the img height and width
                imgHieght, imgWidth, _ = img.shape
                #gets the (x,y) and the rect height of the contour
                midX, midY, height1 = onlyOneCon(con1)
                #calcs both angle and distance
                angle = calcAngle(img=img, midX=midX, tanFOV=tanFov)
                distance = calcDistance(img, height1, 47, 5)
                #only enters once
                if (checkiftrue is True):
                    #starts the entire system that sorts out the angle and distance
                    angle1, prevDistance, counter, counter3 = doesEverything(0, 0, angle, distance, counter3, counter)
                    checkiftrue = False
                #enters every other time
                else:
                    #starts the entire system that sorts out the angle and distance
                    angle1, prevDistance, counter, counter3 = doesEverything(angle1, prevDistance, angle, distance, counter3, counter)
        #if he cant find anything
        elif(x is 0):
            #returns 0 to the smartdashboard
            table.putValue("angle",0)
            #prints "No retro found"
            printNoRetroFound(x, counter3)
            counter3 = 1
            #breaks out of the loop
        key = 0xff & cv2.waitKey(1)
        if key == 27:
            break





#amit laba 0



