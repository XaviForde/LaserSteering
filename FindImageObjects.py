## Create some funtions to find where the laser spot is and where the aphids are
import time 
import numpy as np
import math
import cv2
from tempTargetClass import aphid # as aphid

def findAphids(img):    #img is an [Y_resolution, X_resolution, 3] size numpy array ( where 3 is R,G,B)
    
    AphidStart = time.time()

    #Set initial coordinates to top left
    pixY = 511
    pixX = 0

    # Scan over image until potential aphid pixel found
    #while (img[pixY, pixX,][1] < 180 or img[pixY, pixX,][0] > 0.5*img[pixY, pixX,][1]  or img[pixY, pixX,][2] > 0.5*img[pixY, pixX,][1]):
    while (img[pixY, pixX][1] < 180 or img[pixY, pixX][0] > 120  or img[pixY, pixX][2] > 120):

        #If at end of line:
        if (pixX > len(img[1,:,0])-3   and    pixY >1):  
            
            #Drop to start of next line
            pixY -=2    #drop two rows (quicker than dropping 1)
            pixX = 0    #revert to start of row

        #If in middle of line:
        elif pixY > 1:   
            pixX +=2  #move  pixel to the right          

        #If reached end of the image:
        else:
            print('No Aphid Found!')
            pixX = math.ceil((len(img[0,:,0])-1)/2)
            pixY = math.ceil((len(img[:,0,0])-1)/2)
            break
    
    return np.array([pixX, pixY])

    #prints out aphid location for checking and the time taken
    #print('Aphid Starts at pixels ' + str(pixX) + ',' + str(pixY) + 'time taken = ' + str(time.time() - AphidStart))

     
    
def findLaserSpot(img): #Find the end of the 'Laser line' which VREP uses to show laser path

    #Set initial coordinates to bottom right %%%%%%%% CHANGE THIS TO UPDATE WITH RESOLUTION
    #startTime = time.time()
    #print('Start Laser Search')
    pixY = 0
    pixX = len(img[0,:,0])-1
    startFound = False
    
    # Scan up right side of image until laser pixel found on outside
    while ( 
            startFound == False and
            (img[pixY, pixX][0] < 180 or 
            img[pixY, pixX][1] > 50  or 
            img[pixY, pixX][2] > 50
            )):

        #Before top reached:
        if (pixY != len(img[:,1,0])-1):
            pixY += 1

        #Move across the top (right to left)
        elif (pixY == len(img[:,1,0])-1 and pixX > 0 ):
            pixX -=1
        
        #if no laser found
        else:
            pixX = 0
            pixY = 0
            print('Laser Could not be Located')
            startFound =  True
    

    #Cycles through pixels until the end spot has been found
    #as laser source is right of the camera the next laser
    #occupied cell never be pixX +=1 (i.e to the right of current)
    endFound = False
    firstPixel = True   
    nonLeftMoveCount = 0    #counter to track consecutive up/down motions
    
    #Until end of laser path reached find next red pixel on path
    while endFound == False:

        #check left first
        if  (    
                img[pixY, pixX-1][0] > 180 and 
                img[pixY, pixX-1][1] < 50  and 
                img[pixY, pixX-1][2] < 50
                ):

            pixX -=1
            firstPixel = False
            nonLeftMoveCount = 0    #reset counter for up/down moves
            
        #check diagonal below left
        elif (    
                img[pixY-1, pixX-1][0] > 180 and 
                img[pixY-1, pixX-1][1] < 50  and 
                img[pixY-1, pixX-1][2] < 50
                ):

            pixX -=1
            pixY -=1
            firstPixel = False
            nonLeftMoveCount = 0    #reset counter for up/down moves
            
        #check diagonal above left
        elif (    
                img[pixY+1, pixX-1][0] > 200 and 
                img[pixY+1, pixX-1][1] < 50  and 
                img[pixY+1, pixX-1][2] < 50
                ):

            pixY += 1
            pixX -= 1
            firstPixel = False
            nonLeftMoveCount = 0    #reset counter for up/down moves
                
        #check above 
        elif (
                img[pixY+1, pixX][0] > 200 and 
                img[pixY+1, pixX][1] < 50  and 
                img[pixY+1, pixX][2] < 50
                ):

            
            firstPixel = False
            nonLeftMoveCount += 1
            #stop aimless upward movement, correct to downward
            if nonLeftMoveCount < 3:
                pixY += 1           #move up 1 pixel
            
            else:
                pixY -= 1           #move down 1 pixel as 
                
        #check below 
        elif (
                img[pixY-1, pixX][0] < 180 and 
                img[pixY-1, pixX][1] > 50  and 
                img[pixY-1, pixX][2] > 50
                ):

            pixY -= 1
            firstPixel = False
            nonLeftMoveCount += 1
            if nonLeftMoveCount > 6:
                break

        #if laser has gone through the camera range and out the other side
        elif firstPixel == False and (pixY == len(img[:,0,0])-1 or pixX == len(img[0,:,0])-1):
            print('Laser spot passed beyond camera range!')
            endFound = True
        else:
            endFound = True
    #print('Laser search complete, time = ' + str(time.time()-startTime))
    return np.array([pixX, pixY])
        
    
def findBlueAphids(img):    #img is an [Y_resolution, X_resolution, 3] size numpy array ( where 3 is R,G,B)
    
    AphidStart = time.time()

    #Set initial coordinates to top left
    pixY = 511
    pixX = 0

    # Scan over image until potential aphid pixel found
    #while (img[pixY, pixX,][1] < 180 or img[pixY, pixX,][0] > 0.5*img[pixY, pixX,][1]  or img[pixY, pixX,][2] > 0.5*img[pixY, pixX,][1]):
    while (img[pixY, pixX][2] < 180 or img[pixY, pixX][0] > 120  or img[pixY, pixX][0] > 120):

        #If at end of line:
        if (pixX > len(img[1,:,0])-3   and    pixY >1):  
            
            #Drop to start of next line
            pixY -=2    #drop two rows (quicker than dropping 1)
            pixX = 0    #revert to start of row

        #If in middle of line:
        elif pixY > 1:   
            pixX +=2  #move  pixel to the right          

        #If reached end of the image:
        else:
            print('No Aphid Found!')
            pixX = math.ceil((len(img[0,:,0])-1)/2)
            pixY = math.ceil((len(img[:,0,0])-1)/2)
            break

    
    return np.array([pixX, pixY])


def findBlueAphidsMulti(img):    #img is an [Y_resolution, X_resolution, 3] size numpy array ( where 3 is R,G,B)
    
    ScanStart = time.time()
    #create pixel array to append pixels which contain aphids
    aphidPixels = np.array([[0,0]])
    # Scan over image identifying aphid pixels
    for y in range(0,(len(img[:,0,0]-1)), 2):
        for x in range(0, (len(img[0,:,0]-1)), 2):
            if (
                img[y, x][2] > 180 
                and img[y, x][0] < 100  
                and img[y, x][1] < 100
                ):
                #append aphid coordinates to array of coordinates
                aphidPixels = np.append(aphidPixels, [[x,y]], axis = 0)
    
    #Delete the initial placeholder coordinates (0,0)
    aphidPixels = np.delete(aphidPixels, (0), axis=0)

    #print time taken
    print('Aphid Pixel scan took ' + str(time.time() - ScanStart) + 'seconds.')

    #if no aphids found demand position is center pixel
    if len([aphidPixels[:,0]]) == 0:
        print('No Aphid Found!')
        pixX = math.ceil((len(img[0,:,0])-1)/2) #center x pixel
        pixY = math.ceil((len(img[:,0,0])-1)/2) #center y pixel
        aphids = (pixX, pixY, 0, 0)
    
    #Now group the coordinates into single aphids with a size in pixels
    else:
        #create first aphid, array format: [xstart, ystart, width, height]
        # where the start coordinates are from the top left 
        aphids = np.array(np.append(aphidPixels[0,:], [0,0]))
        aphids = aphids[np.newaxis, :]

        for pixel in range(0,len(aphidPixels[:,0]-1)):
            
            aphids = checkNeighb(aphidPixels[pixel,:], aphids)
    print('Number of Aphids found = ' + str(len(aphids[:,0])))
    return aphids

####################################################################
#This function takes pixel coordinates which have been identified
#as aphids coordinates and finds if it is part of existing aphid
#or the start of a new aphid. It will update size of existing aphid
#if necessary or create a new aphid entry if necessary.
#Inputs:
# - pixels = numpy 1x2 array of form [x_coord, y _coord] 
# - aphids = numpy nx4 array of form [x_start, y_start, width, height]

def checkNeighb(pixels, aphids):

    #Logical used to break loop when neighbour is found
    neighFound = False
    (r,c) = np.shape(aphids)

    #Search for neighbour
    for j in range(0, r):
        
        #check if it is whithin bounds of other aphid
        if (aphids[j,0]-2 <= pixels[0] <= (aphids[j,0]+aphids[j,2]+2) 
                and (aphids[j,1]-2 <= pixels[1] <= (aphids[j,1]+aphids[j,3]+2))):

            neighFound = True
            #if true check if the start/height/width of aphid needs to be updated:
            # check x start
            if (pixels[0] < aphids[j,0]):
                aphids[j,0]= pixels[0]
            
            #check width
            if pixels[0] > (aphids[j,0] + aphids[j,2]):
                aphids[j,2] = pixels[0] - aphids[j,0]

            #its not possible to have new y start as algorithm
            #sweeps down through the image

            #check height
            if pixels[1] < aphids[j,1] - aphids[j,3]:
                aphids[j,3] = aphids[j,1] - pixels[1]

            else:
                pass
        
        #Break if a neighbour is found as don't need to check other aphids
        if neighFound == True:
                break
       
    #If not neighbour was found make new aphid location
    if neighFound == False:
        aphids = np.append(aphids, [np.append(pixels[:],[0,0])], axis = 0)
    
    return aphids

#####################################################################
#This function finds the optimal sequence
# of targets
#Inputs:
# - aphids = numpy nx4 array of form [x_start, y_start, width, height]
# - laserSpot = 1x2 list of laser spot pixel coordinates

def routeFinder(aphids, laserSpot):

    r,c = np.shape(aphids)
    targets = np.zeros([r,c-1])
    dist2laser = np.zeros(r)
    idx_unused = np.arange(r)
    
    for j in idx_unused:
        targets[j,0] = math.ceil(aphids[j,0] + (aphids[j,2]/2)) #centre x pixel of aphid
        targets[j,1] = math.ceil(aphids[j,1] - (aphids[j,3]/2)) #centre y pixel of aphid
        targets[j,2] = min(aphids[j,2], aphids[j,3])            #smallest length of aphid to be used as max error for laser position
        dist2laser[j] = np.sqrt((targets[j,0] - laserSpot[0])**2 + (targets[j,1] - laserSpot[1])**2)

    #create an array of targets not hit yet,
    # then create an array of targets which have been hit.
    # Add index of target closest to laser to hit array index
    # and remove from unhit index.
    
    idx_closest = np.argmin(dist2laser) 
    targets_sorted = [targets[idx_closest, :]]  

    idx_unused = np.arange(len(aphids[:,0]))
    idx_target_order = np.array([idx_closest])
    idx_unused = np.delete(idx_unused, idx_closest)

    while len(idx_unused) > 1:
        #reinitialise distances to zero as laser in new location
        dist2laser = np.zeros(len(idx_unused))
        for j in idx_unused:
                
            #get current position of the laser
            current_target = targets[idx_target_order[-1], :]

            #use pythag to work out distance to remaining targets
            dist2laser_local = np.sqrt((targets[j,0] - laserSpot[0])**2 + (targets[j,1] - laserSpot[1])**2)
            dist2laser = np.append(dist2laser,dist2laser_local)
                
        # Find the closest next target from prvious target
        idx_closest = np.argmin(dist2laser[1:-1])
        idx_target_order = np.append(idx_target_order, idx_unused[idx_closest])
        idx_unused = np.delete(idx_unused, idx_closest)

    #add the remaining target to the target order
    idx_target_order = np.append(idx_target_order, idx_unused[0])

    #sort targets into servoing order 
    
    for i in idx_target_order[1::]:
        targets_sorted = np.append(targets_sorted,[targets[i,:]], axis = 0)

    return targets_sorted
        


###################################################################
#                                                                 #      
# THIS IS THE BEGINNING OF THE openCV image detection algorithms  #
#                                                                 #
###################################################################

##############################################################################
#Filters out aphid targets using HSV, similar to findBlueAphidsMulti
# but uses openCV and HSV rather than custom RGB filtering code
def findBlueAphidsHSV(img):    #img is an [Y_resolution, X_resolution, 3] size numpy array ( where 3 is R,G,B)
    
    ScanStart = time.time()
    #convert image to HSV and filter to find aphids
    imHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)   # !! Could do have this as function input instead
    lower_blue = np.array([50, 200, 100])
    upper_blue = np.array([160, 255, 255])
    
    maskHSV = cv2.inRange(imHSV, lower_blue, upper_blue)
    # get non zero values from mask in [[x_coordinate, y_coordinate],...] form
    aphidPixels = np.transpose(np.roll(np.nonzero(maskHSV),1, axis=0)) 

    #if no aphids found demand position is center pixel
    if len([aphidPixels[:,0]]) == 0:
        print('No Aphid Found!')
        pixX = math.ceil((len(img[0,:,0])-1)/2) #center x pixel
        pixY = math.ceil((len(img[:,0,0])-1)/2) #center y pixel
        aphids = (pixX, pixY, 0, 0)
    
    #Now group the coordinates into single aphids with a size in pixels
    else:
        #create first aphid, array format: [xstart, ystart, width, height]
        # where the start coordinates are from the top left 
        aphids = np.array(np.append(aphidPixels[0,:], [0,0]))
        aphids = aphids[np.newaxis, :]

        for pixel in range(0,len(aphidPixels[:,0]-1)):
            
            aphids = checkNeighb(aphidPixels[pixel,:], aphids)

    #print time taken and number of aphids found
    print('Aphid Pixel scan took ' + str(time.time() - ScanStart) + 'seconds.')
    print('Number of Aphids found = ' + str(len(aphids[:,0])))
    return aphids


#################################################################################
# Finds the laser spot using openCV HSV filtering and some post filter algorithm
def findLaserSpotHSV(img): #Find the end of the 'Laser line' which VREP uses to show laser path

    #startTime = time.time() 
    startFound = False
    #convert to from RGB to HSV
    imHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    #Filters for laser pixels
    lower_laser = np.array([0, 150, 0])
    upper_laser = np.array([100, 255, 255])

    #Mask containing 0 or 1 depening if pixel has laser
    img = cv2.inRange(imHSV, lower_laser, upper_laser)
    pixY = 0
    pixX = len(img[0,:])-1

    # Scan up right side of mask until laser pixel found on outside
    while startFound == False and img[pixY, pixX] == 0:

        #Before top reached:
        if (pixY != len(img[:,1])-1):
            pixY += 1

        #Move across the top (right to left)
        elif (pixY == len(img[:,1])-1 and pixX > 0 ):
            pixX -=1
        
        #if no laser found
        else:
            pixX = 0
            pixY = 0
            print('Laser Could not be Located')
            startFound =  True
    
    #Cycles through pixels until the end spot has been found
    #as laser source is right of the camera the next laser
    #occupied cell never be pixX +=1 (i.e to the right of current)
    endFound = False
    firstPixel = True   
    nonLeftMoveCount = 0    #counter to track consecutive up/down motions
    
    #Until end of laser path reached find next red pixel on path
    while endFound == False:

        #check left first
        if img[pixY, pixX-1] != 0:

            pixX -=1
            firstPixel = False
            nonLeftMoveCount = 0    #reset counter for up/down moves
            
        #check diagonal below left
        elif img[pixY-1, pixX-1] != 0:

            pixX -=1
            pixY -=1
            firstPixel = False
            nonLeftMoveCount = 0    #reset counter for up/down moves
            
        #check diagonal above left
        elif img[pixY+1, pixX-1] != 0:

            pixY += 1
            pixX -= 1
            firstPixel = False
            nonLeftMoveCount = 0    #reset counter for up/down moves
                
        #check above 
        elif img[pixY+1, pixX] != 0:

            firstPixel = False
            nonLeftMoveCount += 1

            #stop aimless upward movement, correct to downward
            if nonLeftMoveCount < 3:
                pixY += 1           #move up 1 pixel
            
            else:
                pixY -= 1           #move down 1 pixel as 
                
        #check below 
        elif img[pixY-1, pixX] != 0:

            pixY -= 1
            firstPixel = False
            nonLeftMoveCount += 1

            if nonLeftMoveCount > 6:
                break

        #if laser has gone through the camera range and out the other side
        elif firstPixel == False and (pixY == len(img[:,0])-1 or pixX == len(img[0,:])-1):
            print('Laser spot passed beyond camera range!')
            endFound = True
        else:
            endFound = True
    
    #print('Laser search complete, time = ' + str(time.time()-startTime))
    return np.array([pixX, pixY])

##############################################################################
# This is similar to findBlueAphidHSV but uses contour finder
# from openCV to identify individual aphids boundaries and return 
# center x and y as well as height and width. 
# Does not need the "checkNeighb" function as a result (should improve speed)
def findBlueAphidsCont(img, aphidList):    

    #ScanStart = time.time() # start timer
    # Convert image to HSV and filter to find aphids
    imHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)   # !! Could do have this as function input instead
    # Set HSV filter values
    lower_blue = np.array([50, 200, 100])
    upper_blue = np.array([160, 255, 255])
    # Filter image to get mask of aphid pixel locations
    maskHSV = cv2.inRange(imHSV, lower_blue, upper_blue)
    # Find contours around each aphid in the mask
    _,contours,_ = cv2.findContours(maskHSV, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Get postion and size of bounding boxes for each aphid and append
    # check if centroid falls within estimated rectangle of existing aphid
    for cnt in contours:

        x,y,width,height = cv2.boundingRect(cnt) # get width and height
        M = cv2.moments(cnt)    
        #Can't divide by zero so check this first, then get centroid location
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00']) #center x pixel
            cy = int(M['m01']/M['m00']) #center y pixel
        else:
            cx = math.ceil(x+(width/2))
            cy = math.ceil(y + (height/2))
        newTarget = True
        #Check if aphid is within region of existing aphids
        for target in aphidList:
            #Get corners of aphids estimated bounding box [xmin, xmax, ymin, ymax]
            endPts = target.estimateNewPos()
            #Check if centroid is within an existing aphids estimated bounding box
            if (endPts[0] <= cx <= endPts[1]) and (endPts[2] <= cy <= endPts[3]):
                target.updatePos(cx, cy, width, height)
                newTarget = False
                #print('Aphid Updated')
                break

        # If not in region of existing aphid create new aphid:
        if newTarget == True:
            ref = aphidList[-1].ID + 1
            newAphid = aphid(cx, cy, width, height, ref)
            aphidList.append(newAphid)
            #print('New Aphid Found')

    #print time taken and number of aphids found
    #print('Aphid Pixel scan took ' + str(time.time() - ScanStart) + 'seconds.')
    print('Number of Aphids = ' + str(len(aphidList)))

    return aphidList


#######################################################
# Finds the next closest target using pythagoras in 
# the image plane. Outputs the idx of the target object
# in the list of target objects 'aphidList':

def findNextTargetIdx(aphidList, laserSpot):

    aphidCount = len(aphidList)
    dist2laser = np.zeros(aphidCount)

    for idx in range(1,aphidCount):
        dist2laser[0] = 1e4
        if aphidList[idx].active == True:
            dist2laser[idx] = np.sqrt((aphidList[idx].currentX - laserSpot[0])**2 + (aphidList[idx].currentY - laserSpot[1])**2)
        elif aphidList[idx].active == False:
            dist2laser[idx] = 1e5
    
    nxtIdx = np.argmin(dist2laser)

    return nxtIdx