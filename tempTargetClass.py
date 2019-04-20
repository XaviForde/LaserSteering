import numpy as np
import math

class aphid(object):

    def __init__(self, centerX, centerY, width, height, ref):
        
        #set current x and y determined through image moments
        self.currentX = centerX
        self.currentY = centerY
        self.estX = centerX
        self.estY = centerY
        self.velY = -3
        self.velX = 0
        self.Area = width * height
        #as no previous x,y data set previous equal to current
        self.prevX = centerX
        self.prevY = centerY
        #set width and height
        self.height = height
        self.width = width
        #Give the target a unique identifier
        self.ID = ref
        #Set target to be active
        self.active = True      # Indicates that the target has not been hit yet
        self.MIA = False        # Indicates that the target is still in the frame
        #Set number frames sice last found to zero
        self.notFoundCount = 0
        #Set logical to state aphid has been created this image frame so estimated postion can 
        self.isFirstFrame = True

    # Activates logic to mark aphid inactive
    def targetHit(self):
        if self.ID != 0:
            #print('Aphid ' + str(self.ID) + ' Destroyed.')
            self.active = False
    
    # estimateNewPos:
    # Calculates pixel velocity.
    # Uses pixel velocity and current postion to predict new position.
    # Creates bounding box for new postion which centroid should lie within.
    def estimateNewPos(self):

        #Set bounds for new positon ...
        # Set tolerance on estimated postion (a box)

        #If it is the aphids first frame it does not have velocity
        #  so the estimated box must be stretched in direction that
        #  the system is travelling to find aphid at high speed (Y dir.)
        if self.isFirstFrame == True:
            
            #No longer first frame
            self.isFirstFrame = False
            #Add extra tolerance to estimated bounding box
            boxHalfHeight = 3 + self.height
            boxHalfWidth = 3 + self.width

            xmin = self.estX - boxHalfWidth
            xmax = self.estX + boxHalfWidth
            ymin = self.estY - boxHalfHeight - 10
            ymax = self.estY + boxHalfHeight     #stretch in direction of travel

            estRect = [xmin, xmax, ymin, ymax]
            
        else:
            # It would be an imporvement to make box sizing
            # Intelligent as larger velocities will give greater
            # uncertainties 
            boxHalfHeight = 4 + self.height
            boxHalfWidth = 4 + self.width

            xmin = self.estX - boxHalfWidth #+ math.ceil(self.velX)
            xmax = self.estX + boxHalfWidth #+ math.ceil(self.velX)
            ymin = self.estY - boxHalfHeight - 4 + math.ceil(self.velY/2)
            ymax = self.estY + boxHalfHeight

            estRect = [xmin, xmax, ymin, ymax]

        #edge case for central pixel target
        if self.ID == 0:
            estRect = [self.currentX, self.currentX, self.currentY, self.currentY]

        return estRect

    # updatePosition:
    # Takes new postion and size variables.
    # Sets current postion as previous postion.
    # Sets new postion as current postion
    # Estimates next position based on image velocity
    # Sets new height and width and area
    def updatePos(self, centerX, centerY, width, height):

        #set velocities in x and y
        self.velX = centerX - self.currentX #New take old position
        self.velY = centerY - self.currentY
        self.estX = centerX + self.velX   #Estimate next position
        self.estY = centerY + self.velY
        #print('Some info: ID, Yvel, height ' + str(self.ID) + ' ' + str(self.velY) +' '+ str(self.height))
        #set current x and y to the nex bounding box center
        self.currentX = centerX
        self.currentY = centerY
        #update height and width and by extension area
        self.height = height
        self.width = width
        self.Area = height*width

    #Records how many image frames since aphid was last found
    def changeNotFoundCount(self, diff):
        if self.ID != 0:
            if diff == 0:
                self.notFoundCount = 0
            else:
                self.notFoundCount += diff
                
            if self.notFoundCount > 1:
                self.targetHit()
                self.MIA = True
                #print('Aphid MIA : Aphid' + str(self.ID))

