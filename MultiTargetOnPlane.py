import vrep
import cv2          #openCV module
import numpy as np
import time
import sys
import matplotlib.pyplot as plt
import FindImageObjects as fio

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

K_Pitch = 0.002     #set proportional gain pitch
K_Yaw = 0.002      #set proportional gain yaw

#Visual Servoing Code Begins
current_target = 0
while True:

    ###Retrieve Image:
    errorCodeImage,resolution,image = vrep.simxGetVisionSensorImage(clientID,Cam1Handle,0,vrep.simx_opmode_buffer)
        
    # Process image in RBG array
    img_processed += 1      #increase image count 1
    img = np.array(image, dtype = np.uint8)     #convert V-REP output to numpy format
    img.resize([resolution[0],resolution[1],3]) #Resize image into pixel resolution by 3 deep for RGB
    #cv2.imshow('image',cv2.flip(img[..., ::-1],0))
    #cv2.waitKey(10000)
    #cv2.destroyAllWindows()
    #Find location of individual aphids in the image every 10th scan
    if (img_processed+99) % 100 == 0: 
        aphids = fio.findBlueAphidsMulti(img)    #Find Location of the aphid in the image      
        laserSpotPos = fio.findLaserSpot(img)  
        targetOrdered = fio.routeFinder(aphids,laserSpotPos)
    #Find location of the laser spot in the image
    laserSpotPos = fio.findLaserSpot(img)           


    #Calculate X and Y laser spot pixel position errors
    x_err = (laserSpotPos[0] - targetOrdered[current_target,0])   #Error in x direction
    y_err = (laserSpotPos[1] - targetOrdered[current_target,1]) #Error in y direction    
    
    #If error is less than aphid size then move to next target
    if (
        x_err <= (targetOrdered[current_target,2] +1)
        and y_err <= (targetOrdered[current_target,2] + 1)
        and current_target!= len(targetOrdered[:,0])-1):
        current_target += 1
        print(' Targets destroyed = ' + str(current_target) + ' from ' + str(len(targetOrdered[:,0])))

    print('ErrorX = ' + str(x_err) + ' ErrorY = ' + str(y_err) + ' Allowable = ' + str(targetOrdered[current_target,2]))

    #Rotate laser as per error 
    ####print('Current Pitch = ' + str(prevPitchAng) + ' Current Yaw = ' + str(prevYawAng))

    #calculate new joint angles
    demandPitchAng = float(prevPitchAng) - (float(y_err)*K_Pitch)
    demandYawAng = float(prevYawAng) + (float(x_err)*K_Yaw)

    ###print('Demand Pitch = ' + str(demandPitchAng) + ' Demand Yaw = ' + str(demandYawAng))
    
    #set new joint angles
    errorCodePitch2 = vrep.simxSetJointPosition(clientID, LaserPitchHandle, demandPitchAng, vrep.simx_opmode_oneshot)
    errorCodeYaw2 = vrep.simxSetJointPosition(clientID, LaserYawHandle, demandYawAng, vrep.simx_opmode_oneshot)
    #update pitch angles
    errCode, prevPitchAng = vrep.simxGetJointPosition(clientID, LaserPitchHandle, vrep.simx_opmode_buffer)
    errCode, prevYawAng = vrep.simxGetJointPosition(clientID, LaserYawHandle, vrep.simx_opmode_buffer)




#Post process the image and some debugging parameters
#plt.imshow(img,origin="lower")
#plt.show()
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
print('Visual Servoing Loop Complete')
#print('Laser Spot at Pixel (' + str(laserSpotPos[0]) + ',' + str(laserSpotPos[1]) + ')')
x_err = (laserSpotPos[0] - aphidPos[0]) #Calculate error in laser spot pixel postion in x direction
y_err = (laserSpotPos[1] - aphidPos[1]) #Calculate error in laser spot pixel position in y direction  
print('ErrorX = ' + str(x_err) + ' ErrorY = ' + str(y_err))
print('Run Time = ' + str(img_processed))
vrep.simxFinish(clientID) #close connetion