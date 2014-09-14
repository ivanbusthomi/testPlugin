from qgis.core import *
import math

def init():
lay=iface.mapCanvas().layers()
layerA=lay[0]
layerB=lay[1]

startB=pLayerB.selectedFeatures()
startA=pLayerA.selectedFeatures()
fStartA=startA[0]
fStartB=startB[0]
pStartA=fStartA.geometry().asPoint()
pStartB=fStartB.geometry().asPoint()
#-------------------
endA=pLayerA.selectedFeatures()
endB=pLayerB.selectedFeatures()
fEndB=endB[1]
fEndA=endA[1]
pEndA=fEndA.geometry().asPoint()
pEndB=fEndB.geometry().asPoint()
    

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
    x1 = pointA.x()
    y1 = pointA.y()
    x2 = pointB.x()
    y2 = pointB.y()
    #-----------------------------------------
    middleCoord = midPoint(pointA,pointB)
    middlePoint = QgsPoint(middleCoord[0],middleCoord[1])
    xMid = middlePoint.x()
    yMid = middlePoint.y()
    #-------------------------------------
    # get max extent value from both point layer
    extA = pLayerA.boundingBoxOfSelected()
    extB = pLayerB.boundingBoxOfSelected()
    #maximum value
    if extA.xMaximum() > extB.xMaximum():
        xMax = extA.xMaximum()
    else:
        xMax = extB.xMaximum()
    if extA.xMinimum() < extB.xMinimum():
        xMin = extA.xMinimum()
    else:
        xMin = extB.xMinimum()
    #minimum value
    if extA.yMaximum() > extB.yMaximum():
        yMax = extA.yMaximum()
    else:
        yMax = extB.yMaximum()
    if extA.yMinimum() < extB.yMinimum:
        yMin = extA.yMinimum()
    else:
        yMin = extB.yMinimum()
    print "max = " , xMax,yMax
    print "min = " , xMin,yMin
    # in order to shorten the equation
    P = x2 - x1
    Q = y2 - y1
    R = xMid - x1
    S = yMid - y1
    # line gradien, for determining max distance of perpendicular line
    gradien1 = Q / P
    gradien2 = -1/gradien1
    #-------------------------------------
    u = R / P
    #u_ = S / Q
    d2 = pointA.sqrDist(pointB)
    #-------------------------------------
    dir_ = direction(pointA,pointB,pEndA,pEndB)
    if dir_ == 1:                  # line direction is up-right
        if math.pow(gradien2,2) > 1:
            print dir_, ", grad >1, ", yMax
            y3 = yMax
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        else:
            print dir_, ", grad <1, ", xMax
            x3 = xMax
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
        #xlim = xMax
        #ylim = yMax
    elif dir_ == 2:                # line direction is down-right
        if math.pow(gradien2,2) > 1:
            print dir_, ", grad >1, ", yMin
            y3 = yMin
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        else:
            print dir_, ", grad <1, ", xMax
            x3 = xMax
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
    #    xlim = xMax
    #    ylim = yMin
    elif dir_ == 3:                # line direction is up-left
        if math.pow(gradien2,2) > 1:
            print dir_, ", grad >1, ", yMax
            y3 = yMax
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        else:
            print dir_, ", grad <1, ", xMin
            x3 = xMin
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
        #xlim = xMin
        #ylim = yMax
    elif dir_ == 4:                # line direction is down-left
        if math.pow(gradien2,2) > 1:
            print dir_, ", grad >1, ", yMin
            y3 = yMin
            x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
        else:
            print dir_, ", grad <1, ", xMin
            x3 = xMin
            y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
        #xlim = xMin
        #ylim = yMin

    #--------------------------------------------------------
    #if math.pow(gradien2,2) > 1:
    #    y3 = ylim
    #    x3 = ((-Q)*y3 + P*x1 + Q*y1 + u*d2) / P
    #else:
    #    x3 = xlim
    #    y3 = ((-P)*x3 + P*x1 + Q*y1 + u*d2) / Q
    #-------------------------------------
    ppdPoint = QgsPoint(x3,y3)
    pointList = []
    pointList.append(middlePoint)
    pointList.append(ppdPoint)
    #-------------------------------------
    p3Line = QgsGeometry.fromPolyline(pointList)
    return p3Line

"""
TEEEEEEEEEESSSSSSSSSSSSSTTTTTTTTTTTTTTT
"""
#def deploy(pLayerA,pLayerB,itv):
pLayerA = lineToPoint(layerA,"A")
pLayerB = lineToPoint(layerB,"B")
pLayerA.startEditing()                      #tricky stuff.i am too lazy
pLayerA.commitChanges()
pLayerB.startEditing()
pLayerB.commitChanges()
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
    
eqPoint,thirdPoint,stop,line,buffer = iteratePoint(pIterA,pIterB,pEndA,pEndB,jLayer,50)
#print stop, len(jLayer)
jLayer = removePoint(jLayer,pIterA,pIterB,thirdPoint)
addPointG(eqPoint)
addPointG(thirdPoint.geometry())
addLine(line)
addPoly(buffer)
if stop ==0:
    list_thirdPoint.append([pIterA, pIterB, thirdPoint.geometry().asPoint()])
    list_equiPoint.append(eqPoint)
    if thirdPoint['ket']=='A':
        pIterA=thirdPoint.geometry().asPoint()
        print "A changed",pIterA,thirdPoint['fid']
    elif thirdPoint['ket']=='B':
        pIterB=thirdPoint.geometry().asPoint()
        print "B changed",pIterB,thirdPoint['fid']
        
else:
    print "stop !=0"
return list_equiPoint,list_thirdPoint