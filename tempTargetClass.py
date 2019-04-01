import numpy as np

class aphid(object):

    def __init__(self, centerX, centerY, width, height, ref):
        
        #set current x and y determined through image moments
        self.currentX = centerX
        self.currentY = centerY
        self.estX = centerX
        self.estY = centerY
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
        self.Area = 0
        #Set number frames sice last found to zero
        self.notFoundCount = 0

    # Activates logic to mark aphid inactive
    def targetHit(self):
        print('Aphid ' + str(self.ID) + ' Destroyed.')
        self.active = False
    
    # estimateNewPos:
    # Calculates pixel velocity.
    # Uses pixel velocity and current postion to predict new position.
    # Creates bounding box for new postion which centroid should lie within.
    def estimateNewPos(self):

        #Set bounds for new positon ...
        # Set tolerance on estimated postion (a box)
        # It would be an imporvement to make box sizing
        # Intelligent as larger velocities will give greater
        # uncertainties 
        boxHalfHeight = 20 + self.height
        boxHalfWidth = 20 + self.width

        xmin = self.estX - boxHalfHeight
        xmax = self.estX + boxHalfHeight
        ymin = self.estY - boxHalfWidth
        ymax = self.estY + boxHalfWidth

        estRect = [xmin, xmax, ymin, ymax]
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
        self.velX = centerX - self.currentX
        self.velY = centerY - self.currentY
        self.estX = self.currentX + self.velX
        self.estY = self.currentY + self.velY
        #update previous postions to the current postion
        self.prevX = self.currentX
        self.prevY = self.currentY
        #set current x and y determined through image moments
        self.currentX = centerX
        self.currentY = centerY
        #update height and width and by extension area
        self.height = height
        self.width = width
        self.Area = height*width

    #Records how many image frames since aphid was last found
    def changeNotFoundCount(self, diff):
        if diff == 0:
            self.notFoundCount = 0
        else:
            self.notFoundCount += diff
            
        if self.notFoundCount > 20:
            self.targetHit()
