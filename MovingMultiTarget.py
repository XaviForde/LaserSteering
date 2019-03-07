import vrep
import cv2          #openCV module
import numpy as np
import time
import sys
import matplotlib.pyplot as plt
import FindImageObjects as fio
from tempTargetClass import aphid

print('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

#check connection was successful
if clientID!=-1:
    print('Connected to remote API server')
else:
    print('Connection unsuccessful')
    sys.exit('Could not connect')

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

#Initialise errors above tolerance, Time and  
startTime = time.time()
img_processed = 0
error_pixel_tol = 2
x_err = 15
y_err = 15
current_target = 0
nextTarget = True
aphidList = []
aphidList.append(aphid(255,255,0,0,0))
## SET GAINS HERE: ##
K_Pitch = 0.0012     #set proportional gain pitch
K_Yaw = 0.0012     #set proportional gain yaw

########     Visual Servoing Code Begins   ##########
while True:

    ###Retrieve Image:
    errorCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
        
    # Process image in RBG array
    img_processed += 1      #increase image count 1
    img = np.array(image, dtype = np.uint8)     #convert V-REP output to numpy format
    img.resize([resolution[0],resolution[1],3]) #Resize image into pixel resolution by 3 deep for RGB

    
    aphids = fio.findBlueAphidsCont(img, aphidList)    #Find Location of the aphids in the image      
    laserSpotPos = fio.findLaserSpotHSV(img)  #Find laser spot location

    if nextTarget == True:
        idx = fio.findNextTargetIdx(aphidList, laserSpotPos)    #Get index of next target
        nextTarget = False

    #Calculate X and Y laser spot pixel position errors
    x_err = (laserSpotPos[0] - aphidList[idx].currentX)   #Error in x direction
    y_err = (laserSpotPos[1] - aphidList[idx].currentY)   #Error in y direction    
    
    #set tolerances in the position errors
    x_tol = aphidList[idx].width 
    y_tol = aphidList[idx].height 

    #Print errors
    print('ErrorX = ' + str(x_err) + ' ErrorY = ' + str(y_err) + ' Allowable = ' + str(x_tol) + ' ' + str(y_tol))

    #If error is less than aphid size then move to next target
    if abs(x_err) <= 1 and abs(y_err) <= 1 and idx != 0:
        current_target += 1
        aphidList[idx].targetHit()
        nextTarget = True
        print('Number destroyed ' + str(current_target) + ' from ' + str(len(aphidList)))

    


    #Rotate laser proportional to error:
    
    #find current pitch angles
    errCode, prevPitchAng = vrep.simxGetJointPosition(clientID, LaserPitchHandle, vrep.simx_opmode_buffer)
    errCode, prevYawAng = vrep.simxGetJointPosition(clientID, LaserYawHandle, vrep.simx_opmode_buffer)
    
    #calculate new joint angles
    demandPitchAng = float(prevPitchAng) - (float(y_err)*K_Pitch)
    demandYawAng = float(prevYawAng) + (float(x_err)*K_Yaw)
    
    #set new joint angles
    errorCodePitch2 = vrep.simxSetJointPosition(clientID, LaserPitchHandle, demandPitchAng, vrep.simx_opmode_oneshot)
    errorCodeYaw2 = vrep.simxSetJointPosition(clientID, LaserYawHandle, demandYawAng, vrep.simx_opmode_oneshot)
    



# CURRENTLY UNUSED CODE WHIHC HELPS FOR DEBUGGING
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
print('Visual Servoing Loop Complete')
print('Frames Analysed = ' + str(img_processed))
vrep.simxFinish(clientID) #close connetion