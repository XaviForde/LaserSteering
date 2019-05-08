#---------------------------------------------------#
#   Uses weighted matrix to find partners           #
#---------------------------------------------------#

import vrep
import math
import cv2          #openCV module
import numpy as np
import csv
import time
import sys
#import matplotlib.pyplot as plt
import FindImageObjects as fio
from tempTargetClass import aphid

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
errCode,LaserPitchHandle = vrep.simxGetObjectHandle(clientID, 'LaserPitch', vrep.simx_opmode_blocking)
errCode,LaserYawHandle = vrep.simxGetObjectHandle(clientID, 'LaserYaw', vrep.simx_opmode_blocking)
errCode, prevPitchAng = vrep.simxGetJointPosition(clientID, LaserPitchHandle, vrep.simx_opmode_streaming)
errCode, prevYawAng = vrep.simxGetJointPosition(clientID, LaserYawHandle, vrep.simx_opmode_streaming)
errCode = vrep.simxSetJointPosition(clientID, LaserPitchHandle, 0, vrep.simx_opmode_oneshot)
errCode = vrep.simxSetJointPosition(clientID, LaserYawHandle, 0, vrep.simx_opmode_oneshot)

#Get actuator handle and set to position to zero, actuator moves the system
errCode,ActuatorHandle = vrep.simxGetObjectHandle(clientID, 'Actuator', vrep.simx_opmode_blocking)
errCode = vrep.simxSetJointPosition(clientID, ActuatorHandle, 0, vrep.simx_opmode_oneshot)

#Get vision handle, call it cam1, then set up image sub, then stream image 
errCode,Cam1Handle = vrep.simxGetObjectHandle(clientID,'Cam1', vrep.simx_opmode_blocking)
errCode,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_streaming)

#Wait for response from camera
image = [0]
while sum(image) == 0:
    errorCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
    #Keep stepping through time until image is recieved
    vrep.simxSynchronousTrigger(clientID)

#Initialise parameters for the main loop
startTime = time.time()
stpCnt = 0  #used to cound number of servo loop iterations
killCnt = 0
aphidCount = 0
error_pixel_tol = 2
x_err = 4
y_err = 4
x_err_old = 3
y_err_old = 3
currentID = 0
nextTarget = True
isFirstTarget = True
aphidList = []      #List containing aphid objects
aphidList.append(aphid(255,255,0,0,0))
x_err_cum = 0
y_err_cum = 0
timestep = 50e-3    #length of timestep in second
errOUT = [[0,0,0,0,0,0,0]]    #for exporting to CSV for matlab analysis
laserSpotPos = [255, 255]
sysPos = 0  #set initial system position to zero
sysVel100 = 30 #for filename, fwd velocity in centimetres per second
dist = 'floor'
sysVel = float(sysVel100)/100 #Actuator velocity in metres per second
maxAng = math.radians(22.5)
minAng = -(math.radians(13.5))
offset = '50'
#--------------------------------------------------#
##              SET GAINS HERE:                   ##
#-------------------0-------------------------------#
Kp = 14
Ki = 240
Kd = 0
Kp_Pitch = Kp/1e4   #set proportional gain pitch
Kp_Yaw = Kp/1e4     #set proportional gain yaw
Ki_Pitch =  Ki/1e4  #set intergral gain pitch
Ki_Yaw = .008    #set integral gain yaw
Kd_Pitch = Kd/1e6   #set derivative gain pitch
Kd_Yaw = Kd/1e6     #set derivative gain yaw

#For writing the csv file:
writeResult = False
filenameCSV = 'Kp' + str(Kp) + 'Ki' + str(Ki) + 'Kd' + str(Kd) + 'V' + str(sysVel100) + 'Floor'  + '.csv'


#---------------------------------------------------#
#           Visual Servoing Loop Begins Here:       #
#---------------------------------------------------#
while True:

    print('Number of Aphids Created = ' + str(aphidCount))
    #increase step count
    stpCnt += 1

    ###Retrieve Image:
    errCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
        
    # Process image in RBG array
    img = np.array(image, dtype = np.uint8)     #convert V-REP output to numpy format
    img.resize([resolution[0],resolution[1],3]) #Resize image into pixel resolution by 3 deep for RGB
  
    ##mainLoopStart = time.time()   #TO TIME LOGIC LOOP UNCOMMENT THIS LINE AND PRINT LINE AT END
    
    #Process image to find aphid and laser spot locations
    aphidList, isShorter, aphidCount = fio.updateAphidsWeighted(img, aphidList, aphidCount, stpCnt)    #Find Location of the aphids in the image      
    x, y, isFound = fio.findLaserSpotNoLine(img=img, old_position = laserSpotPos)  #Find laser spot location
    laserSpotPos = [x,y]
    
    #if list has been altered due to lost aphid need to find new index location of the aphid
    if isShorter == True:
        isFoundIdx = False
        for i, target in enumerate(aphidList):
            if target.ID == currentID:
                idx = i
                isFoundIdx = True

            if isFoundIdx == False:
                x_err_cum = 0
                #y_err_cum = 0
                idx = fio.findNextTargetIdxSat(aphidList, laserSpotPos)
                currentID = aphidList[idx].ID
                

    #Find first target or if 0th aphid keep looking for new targets
    if isFirstTarget == True or aphidList[idx].ID == 0:
        print('Target went missing')
        x_err_cum = 0
        y_err_cum = 0
        idx = fio.findNextTargetIdxSat(aphidList, laserSpotPos)
        currentID = aphidList[idx].ID
        isFirstTarget = False

    #Calculate X and Y laser spot pixel position errors and cumulative errror
    x_err = (laserSpotPos[0] - aphidList[idx].currentX)   #Error in x direction
    y_err = (laserSpotPos[1] - aphidList[idx].currentY)   #Error in y direction

    #Integral of the error signals and the derivitive
    x_err_cum += x_err*timestep
    y_err_cum += y_err*timestep
    x_err_der = (x_err - x_err_old) / timestep
    y_err_der = (y_err - y_err_old) / timestep

    errOUT.append([stpCnt, x_err, y_err, x, y, aphidList[idx].currentX, aphidList[idx].currentY])
    if (writeResult == True) and (stpCnt % 50 == 0):
        with open(filenameCSV, 'w') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerows(errOUT)

    #set tolerances in the position errors
    x_tol = aphidList[idx].width 
    y_tol = aphidList[idx].height 

    #If error is less than aphid size then move to next target
    if abs(x_err) <= x_tol+2 and abs(y_err) <= y_tol+2:
        aphidList[idx].targetHit()
        idx = fio.findNextTargetIdxSat(aphidList, laserSpotPos)    #Get index of next target
        currentID = aphidList[idx].ID
        x_err_cum = 0
        y_err_cum = 0
        x_err_der = 0
        y_err_der = 0
        stepCnt = 0
        if aphidList[idx].ID != 0:
            killCnt += 1
            print('Number Killed = ' + str(killCnt))
        #Reset errors based on new target
        x_err = (laserSpotPos[0] - aphidList[idx].currentX)   #Error in x direction
        y_err = (laserSpotPos[1] - aphidList[idx].currentY)   #Error in y direction
        #Print New Error   
        errOUT.append([stpCnt, x_err, y_err, x, y, aphidList[idx].currentX, aphidList[idx].currentY]) 
        #print(stpCnt)

    #Rotate laser proportional to error, but account for velocity of aphid:
    
    #find current pitch angles
    errCode, prevPitchAng = vrep.simxGetJointPosition(clientID, LaserPitchHandle, vrep.simx_opmode_buffer)
    errCode, prevYawAng = vrep.simxGetJointPosition(clientID, LaserYawHandle, vrep.simx_opmode_buffer)

    #calculate new joint angles
    demandPitchAng = float(prevPitchAng) - (float(y_err)*Kp_Pitch) - (float(y_err_cum)*Ki_Pitch) - (float(y_err_der)*Kd_Pitch)
    demandYawAng = float(prevYawAng) + (float(x_err)*Kp_Yaw) + (float(x_err_cum)*Ki_Yaw) + (float(x_err_der)*Kd_Yaw)
    
    # Do not let angle exceed limits
    if demandPitchAng > (math.radians(18)):
        demandPitchAng = math.radians(18)
        y_err_cum = 0   #stop integral error build up at saturation
    if demandYawAng > maxAng:
        demandYawAng = maxAng
        x_err_cum = 0   #stop integral error build up at saturation
    if demandPitchAng < -(math.radians(17)):
        demandPitchAng = -(math.radians(17))
        x_err_cum = 0   #stop integral error build up at saturation
    if demandYawAng < minAng:
        demandYawAng = minAng 
        x_err_cum = 0   #stop integral error build up at saturation
    #progress timestep in error
    x_err_old = x_err
    y_err_old = y_err

    #Calculate new system postion according to assigned speed
    sysPos = sysPos + timestep*sysVel
    #set new joint angles
    errCode = vrep.simxSetJointPosition(clientID, LaserPitchHandle, demandPitchAng, vrep.simx_opmode_oneshot)
    errCode = vrep.simxSetJointPosition(clientID, LaserYawHandle, demandYawAng, vrep.simx_opmode_oneshot)
    #set platform velocity, read by lua script emedded in pd3x moving robot
    vrep.simxSetJointPosition(clientID, ActuatorHandle, sysPos, vrep.simx_opmode_oneshot)
    #print("Loop takes " + str(mainLoopStart - time.time()) + " seconds.")  #UNCOMMENT THIS LINE TO TIME LOOP
    #Have to signal syncronous trigger three times so that camera buffer updates
    vrep.simxSynchronousTrigger(clientID)
    #vrep.simxSetFloatSignal(clientID, 'wheelPos', 0, vrep.simx_opmode_oneshot)
    vrep.simxSynchronousTrigger(clientID)
    #vrep.simxSetFloatSignal(clientID, 'wheelPos', 0, vrep.simx_opmode_oneshot)
    vrep.simxSynchronousTrigger(clientID)