import vrep
import cv2          #openCV module
import numpy as np
import time
import sys
import math
import matplotlib.pyplot as plt
import FindImageObjects as fio
from tempTargetClass import aphid

## to get range with floats
def frange(start, stop, step):
    i = start
    while i <= stop:
        yield i
        i += step

# Draws epipolar lines
def drawlines(img1,img2,lines,pts1,pts2):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r,c = img1.shape
    img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv2.line(img1, (x0,y0), (x1,y1), color,1)
        #img1 = cv2.circle(img1,tuple(pt1),5,color,-1)
        #img2 = cv2.circle(img2,tuple(int(pt2)),5,color,-1)
    return img1,img2

print('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

#check connection was successful
if clientID!=-1:
    print('Connected to remote API server')
    vrep.simxSynchronous(clientID,True)
else:
    print('Connection unsuccessful')
    sys.exit('Could not connect')

#Trigger First Step
vrep.simxSynchronousTrigger(clientID)



# Get Joint Handles and current positions, set joint angles to zero
errorCodePitch1,LaserPitchHandle = vrep.simxGetObjectHandle(clientID, 'LaserPitch', vrep.simx_opmode_blocking)
errorCodeYaw1,LaserYawHandle = vrep.simxGetObjectHandle(clientID, 'LaserYaw', vrep.simx_opmode_blocking)
errCode, prevPitchAng = vrep.simxGetJointPosition(clientID, LaserPitchHandle, vrep.simx_opmode_streaming)
errCode, prevYawAng = vrep.simxGetJointPosition(clientID, LaserYawHandle, vrep.simx_opmode_streaming)
errorCodePitch2 = vrep.simxSetJointPosition(clientID, LaserPitchHandle, 0, vrep.simx_opmode_oneshot)
errorCodeYaw2 = vrep.simxSetJointPosition(clientID, LaserYawHandle, 0, vrep.simx_opmode_oneshot)

#Get vision handle, call it cam1, then set up image sub, then stream image 
errrorCode,Cam1Handle = vrep.simxGetObjectHandle(clientID,'Cam1', vrep.simx_opmode_blocking)
err_code,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_streaming)

#Wait for response from camera
image = [0]
while sum(image) == 0:
    errorCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
    #Keep stepping through time until image is recieved
    vrep.simxSynchronousTrigger(clientID)

points1 = np.array([])  #camera points
points2 = np.array([])  #laser virtual points

## Calculate the fundamental matrix ##
d = 256/math.sin(math.radians(30)) # distance from laser source to virtual plane
offset = 256    #offset needed for tranform to pixel values
#Cycle through laser point locations and calculate virtual plane coordinates and also
# find the real image plane coordinates
for gamma in frange(-0.15,0.45,0.05):
    for alpha in frange(-0.3,0.3,0.05):
        errorCodePitch2 = vrep.simxSetJointPosition(clientID, LaserPitchHandle, (alpha), vrep.simx_opmode_oneshot)
        errorCodeYaw2 = vrep.simxSetJointPosition(clientID, LaserYawHandle, (gamma), vrep.simx_opmode_oneshot)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        ###Retrieve Image:
        errorCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
        # Process image in RBG array
        img = np.array(image, dtype = np.uint8)     #convert V-REP output to numpy format
        img.resize([resolution[0],resolution[1],3]) #Resize image into pixel resolution by 3 deep for RGB
        x,y, isFound = fio.findLaserSpotNoLine(img, [255,255])
        if isFound == True:
            points1 = np.append(points1, [int(x),int(y)])

            ##Calculate virtual coordinates:
            xVirt = round(offset - d*math.sin(gamma))
            yVirt = round(offset + d*math.sin(alpha))
            points2 = np.append(points2, [int(xVirt),int(yVirt)])
        
nPoints = np.floor(len(points1) / 2)
#reshape point arrays, but store unshaped
pts1D_1 = points1
pts1D_2 = points2
points1 = np.reshape(points1, (int(nPoints),2))
points2 = np.reshape(points2, (int(nPoints),2))

# while True:
print('Fundamental Matrix: ')
fMat, mask = cv2.findFundamentalMat(points1,points2) #,cv2.FM_LMEDS)
print(fMat)

#Now try draw the epipolar lines
img2 = np.zeros(shape = (512,512,3), dtype = float)
for row in xrange(int(nPoints)):
    xidx = points2[row, 0]
    yidx = points2[row, 1]
    img2[int(xidx),int(yidx),:] = 1., 0. , 0.
# Find epilines corresponding to points in right image (imaginary image) and
# drawing its lines on left image
lines1 = cv2.computeCorrespondEpilines(points2, 2,fMat)
lines1 = lines1.reshape(-1,3)
img3 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
img4 = img3 #cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)
img5,img6 = drawlines(img3,img4,lines1,points1,points2)
cv2.imshow("Mask",img5)
cv2.waitKey(0)

#Initialise errors above tolerance, Time and  
startTime = time.time()
img_processed = 0
error_pixel_tol = 2
x_err = 15
y_err = 15
current_target = 0
nextTarget = True
aphidList = []      #List containing aphid objects
aphidList.append(aphid(255,255,0,0,0))
#Set initial laser pos
laserSpotPos = [255, 255]
## SET GAINS HERE: ##
K_Pitch = 0.0004     #set proportional gain pitch
K_Yaw = 0.0004     #set proportional gain yaw



#### ESTIMATING TARGET POSITION IN VIRTUAL PLANE: #################

###Retrieve Image:
errorCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
        
# Process image in RBG array
img_processed += 1      #increase image count 1
img = np.array(image, dtype = np.uint8)     #convert V-REP output to numpy format
img.resize([resolution[0],resolution[1],3]) #Resize image into pixel resolution by 3 deep for RGB





# ########     Visual Servoing Code Begins   ##########
# while True:

#     ###Retrieve Image:
#     errorCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
        
#     # Process image in RBG array
#     img_processed += 1      #increase image count 1
#     img = np.array(image, dtype = np.uint8)     #convert V-REP output to numpy format
#     img.resize([resolution[0],resolution[1],3]) #Resize image into pixel resolution by 3 deep for RGB

#     ##mainLoopStart = time.time()   #TO TIME LOGIC LOOP UNCOMMENT THIS LINE AND PRINT LINE AT END
    
#     #Process image to find aphid and laser spot locations
#     aphidList = fio.findBlueAphidsCont(img, aphidList)    #Find Location of the aphids in the image      
#     laserSpotPos = fio.findLaserSpotNoLine(img=img, old_position = laserSpotPos)  #Find laser spot location

#     # Determine which target is current (idx)
#     if nextTarget == True:
#         idx = fio.findNextTargetIdx(aphidList, laserSpotPos)    #Get index of next target
#         nextTarget = False

#     #Calculate X and Y laser spot pixel position errors
#     x_err = (laserSpotPos[0] - aphidList[idx].currentX)   #Error in x direction
#     y_err = (laserSpotPos[1] - aphidList[idx].currentY)   #Error in y direction    
    
#     #set tolerances in the position errors
#     x_tol = aphidList[idx].width 
#     y_tol = aphidList[idx].height 

#     #Print errors
#     #print('ErrorX = ' + str(x_err) + ' ErrorY = ' + str(y_err) + ' Allowable = ' + str(x_tol) + ' ' + str(y_tol))

#     #If error is less than aphid size then move to next target
#     if abs(x_err) <= 1 and abs(y_err) <= 1 and idx != 0:
#         current_target += 1
#         aphidList[idx].targetHit()
#         nextTarget = True
#         #print('Number destroyed ' + str(current_target) + ' from ' + str(len(aphidList)))


#     #Rotate laser proportional to error:
    
#     #find current pitch angles
#     errCode, prevPitchAng = vrep.simxGetJointPosition(clientID, LaserPitchHandle, vrep.simx_opmode_buffer)
#     errCode, prevYawAng = vrep.simxGetJointPosition(clientID, LaserYawHandle, vrep.simx_opmode_buffer)
    
#     #calculate new joint angles
#     demandPitchAng = float(prevPitchAng) - (float(y_err)*K_Pitch)
#     demandYawAng = float(prevYawAng) + (float(x_err)*K_Yaw)
    
#     #set new joint angles

#     errorCodePitch2 = vrep.simxSetJointPosition(clientID, LaserPitchHandle, demandPitchAng, vrep.simx_opmode_oneshot)
#     errorCodeYaw2 = vrep.simxSetJointPosition(clientID, LaserYawHandle, demandYawAng, vrep.simx_opmode_oneshot)
    
#     #print("Loop takes " + str(mainLoopStart - time.time()) + " seconds.")  #UNCOMMENT THIS LINE TO TIME LOOP
#     vrep.simxSynchronousTrigger(clientID)

vrep.simxFinish(clientID) #close connetion