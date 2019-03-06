import numpy as np

class aphid(object):

    def __init__(self, centerX, centerY, width, height, ref):
        
        #set current x and y determined through image moments
        self.currentX = centerX
        self.currentY = centerY
        
        #as no previous x,y date set previous equal to current
        self.prevX = centerX
        self.prevY = centerY
        #set width and height
        self.height = height
        self.width = width
        #Give the target a unique identifier
        self.ID = ref
        #Set target to be active
        self.active = True

    # Activates logic to mark aphid inactive
    def targetHit(self):
        print('Aphid ' + str(self.ID) + ' Destroyed.')
        self.active = False
    
    # estimateNewPos:
    # Calculates pixel velocity.
    # Uses pixel velocity and current postion to predict new position.
    # Creates bounding box for new postion which centroid should lie within.
    def estimateNewPos(self):
        
        #get X velocity in pixels/frame
        self.velX = self.currentX - self.prevX
        self.velY = self.currentY - self.prevY
        #Estimate new X and Y coordinates
        estX = self.currentX + self.velX
        estY = self.currentY + self.velY

        #Set bounds for new positon ...
        # Set tolerance on estimated postion (a box)
        # It would be an imporvement to make box sizing
        # Intelligent as larger velocities will give greater
        # uncertainties 
        boxHalfHeight = 20 + self.height
        boxHalfWidth = 20 + self.width

        xmin = estX - boxHalfHeight
        xmax = estX + boxHalfHeight
        ymin = estY - boxHalfWidth
        ymax = estY + boxHalfWidth

        estRect = [xmin, xmax, ymin, ymax]
        if self.ID == 0:
            estRect = [self.currentX, self.currentX, self.currentY, self.currentY]
        return estRect

    # updatePosition:
    # Takes new postion and size variables.
    # Sets current postion as previous postion.
    # Sets new postion as current postion
    # Sets new height and width
    def updatePos(self, centerX, centerY, width, height):

        #update previous postions to the current postion
        self.prevX = self.currentX
        self.prevY = self.currentY
        #set current x and y determined through image moments
        self.currentX = centerX
        self.currentY = centerY
        #update height and width
        self.height = height
        self.width = width

