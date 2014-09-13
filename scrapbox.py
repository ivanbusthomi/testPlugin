from qgis.core import *
import math

lay=iface.mapCanvas().layers()
pLayerA=lay[0]
pLayerB=lay[1]
startB=pLayerB.selectedFeatures()
startA=pLayerA.selectedFeatures()
fStartA=startA[0]
fStartB=startB[0]
pStartA=fStartA.geometry().asPoint()
pStartB=fStartB.geometry().asPoint()

endA=pLayerA.selectedFeatures()
endB=pLayerB.selectedFeatures()
fEndB=endB[0]
fEndA=endA[0]
pEndA=fEndA.geometry().asPoint()
pEndB=fEndB.geometry().asPoint()

def midPoint(point1,point2):        # working fine
    #this function returns midpoint from two input point as result.
    x1=point1.x()
    y1=point1.y()
    x2=point2.x()
    y2=point2.y()
    xMid=x1+(x2-x1)/2.0
    yMid=y1+(y2-y1)/2.0
    mid=(xMid,yMid)
    return mid

def direction(pStartA,pStartB,pEndA,pEndB):
    # this function is to dynamically define direction for perpendicular line creation
    #extA= pLayerA.extent()
    #extB= pLayerB.extent()
    m_start= midPoint(pStartA,pStartB)
    m_end= midPoint(pEndA,pEndB)
    pm_start = QgsPoint(m_start[0],m_start[1])
    pm_end = QgsPoint(m_end[0],m_end[1])
    if pm_start.x()<pm_end.x():
        if pm_start.y()<pm_end.y():
            dir_ = 1
        elif pm_start.y()>pm_end.y():
            dir_ = 2
    elif pm_start.x()>pm_end.x():
        if pm_start.y()<pm_end.y():
            dir_ = 3
        elif pm_start.y()>pm_end.y():
            dir_ = 4
    return dir_

def perpendicularLine(pointA,pointB,pEndA,pEndB,pLayerA,pLayerB):
    # this function is to find a line that is perpendicular to line AB at its midpoint
    # param 1 and 2 type is point

    x1 = pointA.x()
    y1 = pointA.y()
    x2 = pointB.x()
    y2 = pointB.y()

    #middle coordinate as start point for perpendicular line
    middleCoord = midPoint(pointA,pointB)
    middlePoint = QgsPoint(middleCoord[0],middleCoord[1])
    xm = middlePoint.x()
    ym = middlePoint.y()
    #-------------------------------------
    # get max extent value from both point layer
    extA = pLayerA.extent()
    extB = pLayerB.extent()
    #maximum value
    if extA.xMaximum() > extB.xMaximum():               #define xmax
        xmax = extA.xMaximum()
    else:
        xmax = extB.xMaximum()
    if extA.xMinimum() < extB.xMinimum():               #define xmin
        xmin = extA.xMinimum()
    else:
        xmin = extB.xMinimum()
    #minimum value
    if extA.yMaximum() > extB.yMaximum():               #define xmax
        ymax = extA.yMaximum()
    else:
        ymax = extB.yMaximum()
    if extA.yMinimum() < extB.yMinimum:                 #define xmax
        ymin = extA.yMinimum()
    else:
        ymin = extB.yMinimum()
    # in order to shorten the equation
    P = x2 - x1
    Q = y2 - y1
    R = xm - x1
    S = ym - y1
    # line gradien, for determining max distance of perpendicular line
    gradien1 = Q / P
    gradien2 = -1/gradien1
    #-------------------------------------
    u = R / P
    #u_ = S / Q                                                      # optional value, only for checking
    d2 = pointA.sqrDist(pointB)                                     # d is the distance. d2 is square of d
    #-------------------------------------
    # here comes the equation
    # y3*Q = (-P)*x3 + P*x1 + Q*y1 + u*d2
    # x3*P = (-Q)*y3 + P*x1 + Q*y1 + u*d2
    #-------------------------------------
    # we want the perpendicular line goes until the maximum value of pointA and B, just to make sure it works
    # in any shape of topography,,,,,,,,,, or maybe minimum value of pointA and B. need more consideration later.
    dir_ = direction(pointA,pointB,pEndA,pEndB)
    if dir_ == 1:                  # line direction is up-right
        xlim = xmax
        ylim = ymax
    elif dir_ == 2:                # line direction is down-right
        xlim = xmax
        ylim = ymin
    elif dir_ == 3:                # line direction is up-left
        xlim = xmin
        ylim = ymax
    elif dir_ == 4:                # line direction is down-left
        xlim = xmin
        ylim = ymin
    #--------------------------------------------------------
    if math.pow(gradien2,2) > 1:                        #define x3 and y3, which are the second point coordinates of ppLine
        y3 = ylim
        x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
    else:
        x3 = xlim
        y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
    #-------------------------------------
    ppdPoint = QgsPoint(x3,y3)                          # create it as Point
    pointList = []                                      # create list to contain point for line creation
    pointList.append(middlePoint)                       # insert 1st point, which is mid point
    pointList.append(ppdPoint)                          # insert 2nd point, the one we just calculate
    #-------------------------------------
    p3Line = QgsGeometry.fromPolyline(pointList)        # create line geometry from list of point
    return p3Line                                               # return the result.its type is QgsGeometry