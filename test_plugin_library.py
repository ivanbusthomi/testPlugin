from qgis.core import *
from PyQt4.QtCore import QVariant
import math

def circumCenter(pointA, pointB, pointC):
    xA=pointA.x()
    yA=pointA.y()

    xB=pointB.x()
    yB=pointB.y()

    xC=pointC.x()
    yC=pointC.y()

    a2 = pow(xA,2)+pow(yA,2)
    b2 = pow(xB,2)+pow(yB,2)
    c2 = pow(xC,2)+pow(yC,2)

    const = 2*(xA*(yB-yC)+xB*(yC-yA)+xC*(yA-yB))
    xO = (a2*(yB-yC)+b2*(yC-yA)+c2*(yA-yB))/const
    yO = (a2*(xC-xB)+b2*(xA-xC)+c2*(xB-xA))/const
    pointO = QgsPoint(xO,yO)
    return pointO

def lineToPoint(lineLayer,attr):                                       # STATUS : working      OUTPUT : memory layer
    #from PyQt4.QtCore import QVariant
    # function to convert line feature to point feature
    # PS: add function to add Attribute of A and B layer, to differentiate in processing later
    pointLayer = QgsVectorLayer("Point", "Point Result", "memory")
    prPoint = pointLayer.dataProvider()
    # add attribute collumns
    prPoint.addAttributes([QgsField("fid",QVariant.Int),QgsField("ket",QVariant.String)])
    feats = []
    for line in lineLayer.getFeatures():
        #get coordinate list from asPolyline function
        lineGeom = line.geometry()
        listCoord = lineGeom.asPolyline()
        inc = 0
        #convert to point
        for c in listCoord:
            increase = inc
            point = QgsPoint(c[0],c[1])
            geomPoint = QgsGeometry.fromPoint(point)
            #feat = QgsFeature(fields_)
            feat = QgsFeature()
            feat.setGeometry(geomPoint)
            # add atributes
            feat.setAttributes([increase,attr])
            inc = increase + 1
            feats.append(feat)
        prPoint.addFeatures(feats)
        QgsMapLayerRegistry.instance().addMapLayer(pointLayer)                      #add to defined point layer
    return pointLayer

def pointToLine(pointList):                                    # STATUS : working      OUTPUT : memory layer
    # function to convert point features to line feature
    #canvas = iface.mapCanvas()
    lineLayer = QgsVectorLayer("LineString", "Line Result", "memory")              # line Layer saved in memory
    prLine = lineLayer.dataProvider()
    #pointList = []
    #for point in pointLayer.getFeatures():
    #    p = point.geometry().asPoint()
    #    pointList.append(p)
    feat = QgsFeature()
    lineGeom = QgsGeometry.fromPolyline(pointList)
    feat.setGeometry(lineGeom)
    prLine.addFeatures([feat])
    QgsMapLayerRegistry.instance().addMapLayer(lineLayer)

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

def perpendicularLine(pointA,pointB,pLayerA,pLayerB,dir_):
    # this function is to find a line that is perpendicular to line AB at its midpoint
    # param 1 and 2 type is point
    x1 = pointA.x()
    y1 = pointA.y()
    x2 = pointB.x()
    y2 = pointB.y()
    #-------------------------------------
    middleCoord = midPoint(pointA,pointB)
    middlePoint = QgsPoint(middleCoord[0],middleCoord[1])
    xm = middlePoint.x()
    ym = middlePoint.y()
    #-------------------------------------
    # get max extent value from both point layer
    extA = pLayerA.extent()          #global
    extB = pLayerB.extent()          #global
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
    if extA.yMaximum() > extB.yMaximum():               #define ymax
        ymax = extA.yMaximum()
    else:
        ymax = extB.yMaximum()

    if extA.yMinimum() < extB.yMinimum():                 #define ymin
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
    u_ = S / Q                                                      # optional value, only for checking
    d2 = pointA.sqrDist(pointB)                                     # d is the distance. d2 is square of d
    #-------------------------------------
    # here comes the equation
    # y3*Q = (-P)*x3 + P*x1 + Q*y1 + u*d2
    # x3*P = (-Q)*y3 + P*x1 + Q*y1 + u*d2
    #-------------------------------------
    # we want the perpendicular line goes until the maximum value of pointA and B, just to make sure it works
    # in any shape of topography,,,,,,,,,, or maybe minimum value of pointA and B. need more consideration later.
    if dir_ == 1:                  # line direction is up-right
        if math.pow(gradien2,2)>1:
            y3 = ymax
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        elif math.pow(gradien2,2)<1:
            x3 = xmax
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
    elif dir_ == 2:                # line direction is down-right
        if math.pow(gradien2,2)>1:
            y3 = ymin
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        elif math.pow(gradien2,2)<1:
            x3 = xmax
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
    elif dir_ == 3:                # line direction is up-left
        if math.pow(gradien2,2)>1:
            y3 = ymax
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        elif math.pow(gradien2,2)<1:
            x3 = xmin
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
    elif dir_ == 4:                # line direction is down-left
        if math.pow(gradien2,2)>1:
            y3 = ymin
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        elif math.pow(gradien2,2)<1:
            x3 = xmin
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
    #-------------------------------------
    ppdPoint = QgsPoint(x3,y3)                          # create it as Point
    pointList = []                                      # create list to contain point for line creation
    pointList.append(middlePoint)                       # insert 1st point, which is mid point
    pointList.append(ppdPoint)                          # insert 2nd point, the one we just calculate
    #-------------------------------------
    p3Line = QgsGeometry.fromPolyline(pointList)        # create line geometry from list of point
    return p3Line   #,xmax,ymax,xmin,ymin                                               # return the result.its type is QgsGeometry

def nearestPoint(point, points):                                # STATUS working
    import math                                                 # param1 = point, param 2=list of features
    currentDistance = 99999999999999999                         # output = nearest feature to param1
    for i in points:
        iAsPoint = i.geometry().asPoint()
        distance = math.sqrt(point.sqrDist(iAsPoint))
        if distance < currentDistance:
            currentDistance = distance
            nearest = i
    return nearest

def distanceFromPoints(point1,point2):
    import math
    return math.sqrt(point1.sqrDist(point2))

def join(pLayerA,pLayerB):
    # join two points layer
    aC = []
    bC = []
    for a in pLayerA.getFeatures():
        aC.append(a)
    for b in pLayerB.getFeatures():
        bC.append(b)
    mC = aC + bC
    return mC       #return list of feature

def intersected(geom,list_of_feat):           #STATUS : working
    list_of_result=[]
    for f in list_of_feat:
        fGeom = f.geometry()
        fPoint = fGeom.asPoint()
        if fGeom.intersects(geom):
            list_of_result.append(f)
    return list_of_result

def iteratePoint(startA,startB,pEndA,pEndB,jLayer,itv):
    #param 1 and 2 type is point, param 3 and 4 type i point Layer, param 5 type is integer
    #output two list which contain list of three points (list_threePoint),
    #and list of equidistance point result (list_equiPoint)
    #p = point type, g=geom type , f = feature type
    dir_ = direction(startA,startB,pEndA,pEndB)                          #                                           OK
    g_ppLine = perpendicularLine(startA,startB,pLayerA,pLayerB,dir_)   # create ppLine geom                             OK                                                                                 #OK
    cDistance = 0                                 # current distance. used for creating point along ppLine geom
    stop = 0
    r=QgsFeature()
    #equiGeom = QgsGeometry()
    while cDistance < g_ppLine.length():
        equiGeom = g_ppLine.interpolate(cDistance)        # equidistance point candidate
        #eG = QgsGeometry.fromPoint(eP)              # geometry eP
        dA = distanceFromPoints(startA, equiGeom.asPoint())
        buffer = equiGeom.buffer(dA,15)
        res =  intersected(buffer,jLayer)
        cDistance = cDistance + itv
        if len(res)>1:
            r=nearestPoint(equiGeom.asPoint(),res)
            break
        elif len(res)==1:
            r=res[0]
            break
        elif len(res)==0:
            continue
    else:
        stop = 1
    return equiGeom,r, stop, g_ppLine, buffer

def removePoint(jLayer,p1,p2,p3):
    for j in jLayer:
        if j.geometry().asPoint()==p1 or j.geometry().asPoint()==p2 or j.geometry().asPoint()==p3.geometry().asPoint():
            jLayer.remove(j)
    return jLayer

def deploy(pLayerA,pLayerB,itv):
    pointListA=[]
    pointListB=[]
    for a in pLayerA.getFeatures():
        pointListA.append(a)
    for b in pLayerB.getFeatures():
        pointListB.append(b)
    pStartA= pointListA[0].geometry().asPoint()
    pStartB= pointListB[0].geometry().asPoint()
    pEndA= pointListA[-1].geometry().asPoint()
    pEndB= pointListA[-1].geometry().asPoint()
    #-------------------
    list_equiPoint = []
    list_thirdPoint = []
    jLayer = join(pLayerA,pLayerB)
    pIterA = pStartA
    pIterB = pStartB
    stop = 0
    while stop ==0 :
        eqPoint,thirdPoint,stop = iteratePoint(pIterA,pIterB,pEndA,pEndB,jLayer,itv)
        #print stop
        jLayer = removePoint(jLayer,pIterA,pIterB,thirdPoint)
        if stop ==0:
            list_thirdPoint.append([pIterA, pIterB, thirdPoint.geometry().asPoint()])
            list_equiPoint.append(eqPoint)
            if thirdPoint['ket']=='A':
                pIterA=thirdPoint.geometry().asPoint()
                #print "A changed",pIterA,n['fid']
            elif thirdPoint['ket']=='B':
                pIterB=thirdPoint.geometry().asPoint()
                #print "B changed",pIterB,n['fid']
    else:
        pass
        #print "stop !=0"
    return list_equiPoint,list_thirdPoint