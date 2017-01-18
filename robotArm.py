
#Created by Joe MacInnes 2015
#Version:  2.0
#LACKING COMMENTS!

import numpy as np
import cv2
import argparse
import math
import serial
#### ARDUINO SETUP
## open the serial port that your ardiono
## is connected to.

ser = serial.Serial("/dev/cu.usbmodem1421", 9600)
def move(servo, angle):
    '''Moves the specified servo to the supplied angle.
        
        Arguments:
        servo
        the servo number to command, an integer from 1-4
        angle
        the desired servo angle, an integer from 0 to 180
        
        (e.g.) >>> servo.move(2, 90)
        ... # "move servo #2 to 90 degrees"'''
    
    if (0 <= angle <= 180):
        ser.write(chr(255))
        ser.write(chr(servo))
        ser.write(chr(angle))
    else:
        print "Servo angle must be an integer between 0 and 180.\n"


refPt = [0,0,0,0] #[x1,y1,x2,y2]
averageContainer = [] #a container for the HSV averages that will be combined
global avgHSV1
global avgHSV2
global avgHSV3
global hsv
screenHeight = 720
screenWidth = 1280
move(1,90)
move(2,90)
move(3,90)



font = cv2.FONT_HERSHEY_SIMPLEX


#cv2.imshow('roi',frame[y1:y2,x1:x2])


#Returns the average values of an array of elements
def calculateAvg(refPt,img):
    average = cv2.mean(img[refPt[1]:refPt[3],refPt[0]:refPt[2]])
    print(average)
    return average


# This is the GUI function that will bind to the setup windows
# It sets the coordinates of a 4 element array that will be used to calculate the average HSV of a rectangle
def setRectPoints(event, x, y, flags, param):
    global refPt
    if event == cv2.EVENT_LBUTTONDOWN: #if the left mouse button is pressed, set the first two elements in the refPt array (x1,y1) to x and y
        refPt[0] = x
        refPt[1] = y
    elif event == cv2.EVENT_LBUTTONUP: #if the left mouse button is released, set the second two elements in the refPt array (x2,y2) to x and y
        refPt[2] = x
        refPt[3] = y

#Very simply returns the user's input
def readInput(prompt):
    usrInput = input(prompt)
    return usrInput

def avgHSVs(count,refPt):
    array = []
    for i in range(0,count):
        while(1):
            #Take each frame
            _, frame = cap.read()
            cv2.putText(frame,'Select %d' % i,(10,500), font, 2,(255,255,255),2,cv2.LINE_AA)
            #Convert BGR to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.imshow('setup',frame)
            k  = cv2.waitKey(5) & 0xFF
            
            if k == 27:
                avgHSV = calculateAvg(refPt,hsv)
                array.append(avgHSV)
                break
    
    return array

def createMasks(array, hueMod, satValMod,img):
    maskContainer = []
    for i in range(0,len(array)):
        hsvValues= np.array(array[i])
        lower = (max(0,hsvValues[0]-15),max(0,hsvValues[1]-satValMod),max(0,hsvValues[2]-satValMod))
        upper = (min(180,hsvValues[0]+15),min(255,hsvValues[1]+satValMod),min(255,hsvValues[2]+satValMod))
        mask = cv2.inRange(img, lower,upper)
        maskContainer.append(mask)
    return maskContainer

#adds the images in an array of images
def addImages(imgArray):
    sum = imgArray[0]
    for i in range(0,len(imgArray)):
        sum = cv2.add(sum,imgArray[i])
    return sum
    
    
def findMaxContour(img):
    _, contours, hierarchy = cv2.findContours(img.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    max_area = -1
    ci = 0
    cnt = np.array([[0,0]]) #safeguard, returns [[0,0]] as the contour if no contours
    if len(contours)>0:
        for i in range(len(contours)):
            cnt=contours[i]
            area = cv2.contourArea(cnt)
            if(area>max_area):
                max_area=area
                ci=i
        cnt=contours[ci]
    return cnt
    
def drawConvexityDefects(img,cnt,hull,desAngle):
    defects = cv2.convexityDefects(cnt,hull)
    count_defects = 0
    for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]
        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])
        a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
        angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
        if angle <= desAngle:
            count_defects += 1
            cv2.circle(img,far,5,[0,0,255],-1)
    return img

def enclosingCircle(contour,img):
    (x,y),radius = cv2.minEnclosingCircle(contour)
    center = (int(x),int(y))
    radius = int(radius)
    cv2.circle(img,center,radius,(0,255,0),2)
    return (img, radius,center)
    
def calibration(averages,refPt):
    calibrationPointsLRTB = [screenWidth/2,screenWidth/2,screenHeight/2,screenHeight/2] #Left, Right, Top, Bottom
    calibrationPointsFB = [0,0] #Left, Right, Top, Bottom
    print("p -> Print Calibration Values")
    print("f or j -> Append Calibration Values (LRTB & FB respectively)")
    print("c -> Clear Calibration Values")
    while(1):
        _, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = createMasks(averages,15,75,hsv)
        fullMask = addImages(masks)
        contour = findMaxContour(fullMask)
        hull = cv2.convexHull(contour)
        hullWithDefect = cv2.convexHull(contour, returnPoints = False)
        cv2.drawContours(frame,[contour],0,(255,0,0),3)
        cv2.drawContours(frame,[hull],0,(255,0,0),3)
        circled,radius,center = enclosingCircle(contour,frame)
        cv2.imshow('running',circled)
        
        if(center[0]> calibrationPointsLRTB[0]): #set left calib point
            calibrationPointsLRTB[0] = center[0]
        if(center[0]< calibrationPointsLRTB[1]): #set right calib point
            calibrationPointsLRTB[1] = center[0]
        if(center[1]> calibrationPointsLRTB[2]): #set top calib point
            calibrationPointsLRTB[2] = center[1]
        if(center[1]< calibrationPointsLRTB[3]): #set bottom calib point
            calibrationPointsLRTB[3] = center[1]
        if(radius > calibrationPointsFB[0]): #set forward calib point
            calibrationPointsFB[0] = radius
        print(calibrationPointsFB)
        k  = cv2.waitKey(5) & 0xFF
        if k == 27: #exit
            break
    return calibrationPointsLRTB, calibrationPointsFB
    
    
cap = cv2.VideoCapture(0)
cv2.namedWindow('setup')
cv2.setMouseCallback('setup',setRectPoints)






# Get the averages and calibration points before the main run loop
averages = avgHSVs(1,refPt)
calibrationPointsLRTB, calibrationPointsFB = calibration(averages,refPt)
DistLtoR = calibrationPointsLRTB[0]-calibrationPointsLRTB[1]
DistTtoB = calibrationPointsLRTB[3]-calibrationPointsLRTB[2]
DistFtoB = calibrationPointsFB[0]-calibrationPointsFB[1]


while(1):
    global calibrationPointsLRTB
    global calibrationPointsFB
    global DistLtoR
    global DistTtoB
    global DistFtoB
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = createMasks(averages,15,75,hsv)
    fullMask = addImages(masks)
    contour = findMaxContour(fullMask)
    hull = cv2.convexHull(contour)
    hullWithDefect = cv2.convexHull(contour, returnPoints = False)
    cv2.drawContours(frame,[contour],0,(255,0,0),3)
    cv2.drawContours(frame,[hull],0,(255,0,0),3)
    circled,radius,center = enclosingCircle(contour,frame)
    #for i in range(0,len(calibrationPointsLRTB)):
        #cv2.circle(circled,calibrationPointsLRTB[i],5,(0,255,0),2)
    cv2.circle(circled,center,5,(0,0,255),2)
    cv2.imshow('running',circled)
    servoLR = int(((center[0]-calibrationPointsLRTB[1])/(DistLtoR/1.0))*180) #LR
    servoTB = int(((center[1]-calibrationPointsLRTB[2])/(DistTtoB/1.0))*180) #TB
    servoFB = int(((radius-calibrationPointsFB[1])/(DistFtoB/1.0))*180)
    
    move(1, servoLR)
    
    move(2, servoFB)
    '''if servoFB > 120:
        servoFB = 160
    elif servoFB < 30:
        servoFB = 20
    else:
        servoFB = servoFB'''
    move(3, servoTB)
    k  = cv2.waitKey(5) & 0xFF
    if k == 27:
        break



                              
                              
