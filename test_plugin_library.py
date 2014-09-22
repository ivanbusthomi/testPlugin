from qgis.core import *
from PyQt4.QtCore import QVariant
import math

def circumCenter(pointA, pointB, pointC):
    xA=pointA.x()
    yA=pointA.y()
    #---------------------
    xB=pointB.x()
    yB=pointB.y()
    #---------------------
    xC=pointC.x()
    yC=pointC.y()
    #---------------------
    a2 = pow(xA,2)+pow(yA,2)
    b2 = pow(xB,2)+pow(yB,2)
    c2 = pow(xC,2)+pow(yC,2)
    #---------------------
    const = 2*(xA*(yB-yC)+xB*(yC-yA)+xC*(yA-yB))
    xO = (a2*(yB-yC)+b2*(yC-yA)+c2*(yA-yB))/const
    yO = (a2*(xC-xB)+b2*(xA-xC)+c2*(xB-xA))/const
    pointO = QgsPoint(xO,yO)
    return pointO

def lineToPoint(lineLayer,attr):                                       # STATUS : working      OUTPUT : memory layer
    pointLayer = QgsVectorLayer("Point?crs=EPSG:32749", "Point Result", "memory")
    layerProv = pointLayer.dataProvider()
    layerProv.addAttributes([QgsField("fid",QVariant.Int),QgsField("ket",QVariant.String)])
    feats = []
    list =[]
    id = 0
    for lineFeature in lineLayer.getFeatures():
        listLineCoord = lineFeature.geometry().asPolyline()
        for coord in listLineCoord:
            list.append(coord)
    for coord in list:
        point = QgsPoint(coord[0],coord[1])
        pGeom = QgsGeometry.fromPoint(point)
        pFeat = QgsFeature()
        pFeat.setGeometry(pGeom)
        pFeat.setAttributes([id,attr])
        id +=1
        feats.append(pFeat)    
    layerProv.addFeatures(feats)
    QgsMapLayerRegistry.instance().addMapLayer(pointLayer)
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
    m_start= midPoint(pStartA,pStartB)
    m_end= midPoint(pEndA,pEndB)
    pm_start = QgsPoint(m_start[0],m_start[1])
    pm_end = QgsPoint(m_end[0],m_end[1])
    dir_ = 0
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
    dy = pm_end.y()-pm_start.y()
    dx = pm_end.x()-pm_start.x()
    if dx == 0:
        grad_=5
    else:
        grad_ = dy/dx
    return dir_,grad_

def perpendicularLine(pointA,pointB,pLayerA,pLayerB,dir_):
    # this function is to find a line that is perpendicular to line AB at its midpoint
    # param 1 and 2 type is point
    x1 = pointA.x()
    y1 = pointA.y()
    x2 = pointB.x()
    y2 = pointB.y()
    x3 = 0
    y3 = 0
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
    if P==0:
        #gradien1 = infinity. the line is vertical.then gradien2 must be horizontal
        gradien2 = 0
        u = 0
    elif Q==0:
        #gradien1 = horizontal, then gradien2 must be vertical. so the gradien value is infinity
        gradien2 = 2
        u = R / P
    else:
        gradien1 = Q / P
        gradien2 = -1/gradien1
        u = R / P
    #-------------------------------------
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

def sortList(featureList,grad,dir_):
    point_list = []
    for f in featureList:
        point_list.append(f.geometry().asPoint())
    if dir_ == 1:
        if grad >1:
            sorted_point_list = sorted(point_list,key=lambda x:x[1])
        elif grad<1:
            sorted_point_list = sorted(point_list,key=lambda x:x[0])
    elif dir_ == 2:
        if grad >1:
            sorted_point_list = sorted(point_list,key=lambda x:x[1], reverse = True)
        elif grad<1:
            sorted_point_list = sorted(point_list,key=lambda x:x[0])
    elif dir_ == 3:
        if grad >1:
            sorted_point_list = sorted(point_list,key=lambda x:x[1])
        elif grad<1:
            sorted_point_list = sorted(point_list,key=lambda x:x[0], reverse = True)
    elif dir_ == 4:
        if grad >1:
            sorted_point_list = sorted(point_list,key=lambda x:x[1], reverse = True)
        elif grad<1:
            sorted_point_list = sorted(point_list,key=lambda x:x[0], reverse = True)
    return point_list
def pointlistToFeature(pointList,attr):
    feature_list=[]
    field_fid = QgsField("fid",QVariant.Int)
    field_ket = QgsField("ket",QVariant.String)
    fields = QgsFields()
    fields.append(field_fid)
    fields.append(field_ket)
    id = 0
    for coord in pointList:
        pt = QgsPoint(coord[0],coord[1])
        feat_geom = QgsGeometry.fromPoint(pt)
        feat=QgsFeature(fields)
        feat.setGeometry(feat_geom)
        feat.setAttributes([id,attr])
        id+=1
        feature_list.append(feat)
    #add result as layer
    #pointLayer = QgsVectorLayer("Point","Sorted Point", "memory")
    #pointProv = pointLayer.dataProvider()
    #pointProv.addAttributes(fields)
    #pointProv.addFeatures(feature_list)
    #QgsMapLayerRegistry.instance().addMapLayer(pointLayer)
    return feature_list
def iteratePoint(startA,startB,fListA,fListB,g_ppLine,dir_,grad_,itv):
    #param 1 and 2 type is point, param 3 and 4 type i point Layer, param 5 type is integer
    #output two list which contain list of three points (list_threePoint),
    #and list of equidistance point result (list_equiPoint)
    #p = point type, g=geom type , f = feature type
    #--------------------------------------------------------sort by max-min coordinate
    #sorted_pointA = sortList(fListA,grad_,dir_)
    #sorted_pointB = sortList(fListB,grad_,dir_)
    #sorted_featuresA = pointlistToFeature(sorted_pointA,"A")
    #sorted_featuresB = pointlistToFeature(sorted_pointB,"B")
    #--------------------------------------------------------
    cDistance = 0                                 # current distance. used for creating point along ppLine geom
    stop = 0
    r=QgsFeature()
    #buffer = QgsGeometry()
    equiGeom = QgsGeometry()
    while cDistance <= g_ppLine.length():
        equiGeom = g_ppLine.interpolate(cDistance)        # equidistance point candidate
        #eG = QgsGeometry.fromPoint(eP)              # geometry eP
        dA = distanceFromPoints(startA, equiGeom.asPoint())
        #dB = distanceFromPoints(startB, equiGeom.asPoint())
        buffer = equiGeom.buffer(dA,15)
        res =  intersectedNew(buffer,fListA,fListB,startA,startB,dir_ ,grad_)
        cDistance = cDistance + itv
        if len(res)>1:
            r=nearestPoint(equiGeom.asPoint(),res)
            break
        elif len(res)==1:
            r=res[0]
            break
        elif len(res)==0:
            continue
    if cDistance > g_ppLine.length():
        stop = 1
    return equiGeom,r, stop #, g_ppLine, buffer   #geom,feat,int,geom,geom,int,list

def removePoint(jLayer,p1,p2,p3):
    for j in jLayer:
        if j.geometry().asPoint()==p1 or j.geometry().asPoint()==p2 or j.geometry().asPoint()==p3.geometry().asPoint():
            jLayer.remove(j)
    return jLayer


def addPointL(list_of_pointGeom):
    pointLayer = QgsVectorLayer("Point","Point", "memory")
    pointProv = pointLayer.dataProvider()
    lFeat=[]
    for p in list_of_pointGeom:
        pFeat = QgsFeature()
        pFeat.setGeometry(p)
        lFeat.append(pFeat)
    pointProv.addFeatures(lFeat)
    QgsMapLayerRegistry.instance().addMapLayer(pointLayer)

def addPointG(pointGeom):
    pointLayer = QgsVectorLayer("point?crs=epsg:32749","Point", "memory")
    pFeat = QgsFeature()
    pFeat.setGeometry(pointGeom)
    pointProv = pointLayer.dataProvider()
    pointProv.addFeatures([pFeat])
    QgsMapLayerRegistry.instance().addMapLayer(pointLayer)

def addPointF(pointFeatures):
    pointLayer = QgsVectorLayer("Point","Point Layer", "memory")
    #pFeat = QgsFeature()
    #pFeat.setGeometry(pointGeom)
    pointProv = pointLayer.dataProvider()
    pointProv.addFeatures(pointFeatures)
    QgsMapLayerRegistry.instance().addMapLayer(pointLayer)

def addLine(lineGeom):
    lineLayer = QgsVectorLayer("LineString?crs=epsg:32749","Line Result", "memory")
    lFeat = QgsFeature()
    lFeat.setGeometry(lineGeom)
    lineProv = lineLayer.dataProvider()
    lineProv.addFeatures([lFeat])
    QgsMapLayerRegistry.instance().addMapLayer(lineLayer)

def addPoly(polyGeom):
    polyLayer = QgsVectorLayer("Polygon?crs=epsg:32749","Polygon Result", "memory")
    pFeat = QgsFeature()
    pFeat.setGeometry(polyGeom)
    polyProv = polyLayer.dataProvider()
    polyProv.addFeatures([pFeat])
    QgsMapLayerRegistry.instance().addMapLayer(polyLayer)

def intersectedNew(buffer,fListA,fListB,startA,startB,dir_,grad):
    list_of_result = []
    for feature in fListA:
        f_geom=feature.geometry()
        f_point=f_geom.asPoint()
        if dir_ == 1 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()>startA.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()>startA.x():
                list_of_result.append(feature)
        elif dir_ == 2 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()<startA.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()>startA.x():
                list_of_result.append(feature)
        elif dir_ == 3 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()>startA.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()<startA.x():
                list_of_result.append(feature)
        elif dir_ == 4 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()<startA.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()<startA.x():
                list_of_result.append(feature)
    #feature B
    for feature in fListB:
        f_geom=feature.geometry()
        f_point=f_geom.asPoint()
        if dir_ == 1 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()>startB.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()>startB.x():
                list_of_result.append(feature)
        elif dir_ == 2 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()<startB.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()>startB.x():
                list_of_result.append(feature)
        elif dir_ == 3 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()>startB.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()<startB.x():
                list_of_result.append(feature)
        elif dir_ == 4 and f_geom.intersects(buffer):
            if grad >=1 and f_point.y()<startB.y():
                list_of_result.append(feature)
            elif grad<1 and f_point.x()<startB.x():
                list_of_result.append(feature)
    #result returned
    return list_of_result

def deploy(p_start_a, p_start_b, p_end_a, p_end_b,point_layer_a,point_layer_b,itv):
    list_equiGeom = []
    list_thirdFeature = []
    p_iter_a = p_start_a
    p_iter_b = p_start_b
    stop = 0
    list_feat_a=[]
    list_feat_b=[]
    for feature in point_layer_a.getFeatures():
        list_feat_a.append(feature)
    for feature in point_layer_b.getFeatures():
        list_feat_b.append(feature)
    # iterate
    while stop == 0:
        dir_,grad_ = direction(p_iter_a, p_iter_b, p_end_a, p_end_b)
        g_pp_line = perpendicularLine(p_iter_a, p_iter_b ,point_layer_a ,point_layer_b ,dir_)
        g_equi, f_third,stop= iteratePoint(p_iter_a,p_iter_b,list_feat_a,list_feat_b,g_pp_line,dir_,grad_,itv)
        if stop ==0 and f_third.geometry().asPoint()!=p_iter_a and f_third.geometry().asPoint()!=p_iter_b :
            addPointG(g_equi)
            #addPointG(f_third.geometry())
            list_equiGeom.append(g_equi)
            list_thirdFeature.append([p_iter_a, p_iter_b, f_third.geometry().asPoint()])
            if f_third['ket']=='A':
                p_iter_a=f_third.geometry().asPoint()
                print "A changed",p_iter_a,f_third['fid']
            elif f_third['ket']=='B':
                p_iter_b=f_third.geometry().asPoint()
                print "B changed",p_iter_b,f_third['fid']
    else:
        print "Process has been stopped"
    return list_equiGeom,list_thirdFeature