import numpy as np
import cv2
from tempTargetClass import aphid
import math

def findLaserSpotTest(img, spotCoords, lower_blue = np.array([117,26,119]), upper_blue = np.array([179,100,255])):    

    #ScanStart = time.time() # start timer
    isFound = False
    # Filter image to get mask of aphid pixel locations
    maskHSV = cv2.inRange(imHSV, lower_blue, upper_blue)
    # Find contours of potential laser spots
    _,contours,_ = cv2.findContours(maskHSV, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create empty array of candidate laser spot postions
    potLocs = np.array([0,0,0,0])
    
    for cnt in contours:

        x, y, width, height = cv2.boundingRect(cnt) # get width and height
        area = width*height #calculate bounding rectangle area
        AR = height/width   #calculate aspect ratio

        #ensure contour as correct aspect ratio and is large enough
        if 20 <= area <= 150  and (0.6 <= AR <= 1.6):
            isFound = True
            
            potLocs = np.append(potLocs, [x,y,width,height])
            
            spotCoords = np.append(spotCoords,np.array([x,y]))
            spotCoords = spotCoords[1::, :]
        

        
    if isFound == False:
        x = 0
        y = 0
        width = 0
        height = 0
        

    return x, y, width, height

spotCoords = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])

#################################################################################
########                 START IMAGE PROCESSING HERE:                #############
#################################################################################

def nothing(x):
    pass

# Creating a window for later use
cv2.namedWindow('result')
# Creating track bar
cv2.createTrackbar('h', 'result',0,179,nothing)
cv2.createTrackbar('s', 'result',0,255,nothing)
cv2.createTrackbar('v', 'result',0,255,nothing)
# Creating track bar
cv2.createTrackbar('h2', 'result',0,179,nothing)
cv2.createTrackbar('s2', 'result',0,255,nothing)
cv2.createTrackbar('v2', 'result',0,255,nothing)

#Starting Video
cap = cv2.VideoCapture(0)
spotCoords = np.array([0,0])
while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()
    imBGR = frame
    #filename = 'LaserSpot1'
    #imBGR = cv2.imread(filename + '.jpg')
    imBlurr = imBGR
    
    #imBlurr = cv2.medianBlur(imBGR, 3)
    #cv2.imshow('blur',imBlurr)
    # Our operations on the frame come here
    imHSV = cv2.cvtColor(imBlurr, cv2.COLOR_BGR2HSV)
    #cv2.imshow('HSV',imHSV)
    # get info from track bar and appy to result
    h = cv2.getTrackbarPos('h','result')
    s = cv2.getTrackbarPos('s','result')
    v = cv2.getTrackbarPos('v','result')
    h2 = cv2.getTrackbarPos('h2','result')
    s2 = cv2.getTrackbarPos('s2','result')
    v2 = cv2.getTrackbarPos('v2','result')

    lower_blue = np.array([h, s, v])
    upper_blue = np.array([h2, s2, v2])

    #lower_blue = np.array([120, 50, 120])
    #upper_blue = np.array([180, 100, 250])
    x,y,width,height = findLaserSpotTest(imHSV, spotCoords, lower_blue, upper_blue)    

    #print(len(aphidList))

    # Filter image to get mask of aphid pixel locations
    maskHSV = cv2.inRange(imHSV, lower_blue, upper_blue)
    cv2.rectangle(imBGR,(x-(width/2)-5,y-height/2-5),(x+width/2+5,y+height/2+5),(255,0,0),3)
    # Display the resulting frame
    cv2.imshow('RGB',imBGR)
    cv2.imshow('frame',imHSV)
    cv2.imshow('final', maskHSV)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
#cap.release()
cv2.destroyAllWindows()