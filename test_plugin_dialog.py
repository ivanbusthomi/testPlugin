# -*- coding: utf-8 -*-
"""
/***************************************************************************
 testPluginDialog
                                 A QGIS plugin
 testing plugin
                             -------------------
        begin                : 2014-07-05
        git sha              : $Format:%H$
        copyright            : (C) 2014 by private
        email                : testing@adasd.cosaf
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
"""

import os
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from qgis.core import *
import testPlugin_guiNew
import helpDialog
from test_plugin_library import *
#import for mouseEvent
from qgis.gui import *
#from qgis.utils import iface

#from aaa_aa_tool import aa_tool



class testPluginDialog(QDialog,testPlugin_guiNew.Ui_Dialog):
    def __init__(self, parent=None):
        """Constructor."""
        #self.iface = iface                                                          #for mouse click event
        #self.canvas=self.iface.mapCanvas()
        #self.clickTool = QgsMapToolEmitPoint(self.canvas)                        #for mouse click event
        super(testPluginDialog, self).__init__(parent)
        # Set up the user interface from Designer.
        # After setupUI you can access any designer object by doing
        # self.<objectname>, and you can use autoconnect slots - see
        # http://qt-project.org/doc/qt-4.8/designer-using-a-ui-file.html
        # #widgets-and-dialogs-with-auto-connect


        self.setupUi(self)

        # connect browse button
        self.connect(self.buttonBrowse, SIGNAL("clicked()"),self.browseOutfile)
        self.connect(self.buttonRun, SIGNAL("clicked()"),self.runYouFools)

        #self.connect(self.buttonStartA, SIGNAL("clicked()"),self.getStartA)
        #self.connect(self.buttonStartB, SIGNAL("clicked()"),self.getStartB)
        #self.connect(self.buttonEndA, SIGNAL("clicked()"),self.getEndA)
        #self.connect(self.buttonStartB_2, SIGNAL("clicked()"),self.getEndB)
        #self.connect(self.buttonStartA,SIGNAL("clicked()"),self.getCoord)

    #def getStartA(self):
        #self.iface.mapCanvas().setMapTool(self.tool)                            #initialize mouseclick
        #self.textBrowser.append(p_map)
        # map canvas tool

    def runYouFools(self):
        indexA= self.comboInputA.currentIndex()
        indexB= self.comboInputB.currentIndex()
        inputA= self.comboInputA.itemData(indexA)
        inputB= self.comboInputA.itemData(indexB)
        #intv =  str(self.doubleSpinBox.value())
        #nameA= unicode(inputA.name())
        #nameB= unicode(inputB.name())
        #self.textBrowser.append(nameA + " , " +nameB + " , " + intv)

        #convert line input to point layer
        pLayerA = lineToPoint(inputA,"A")
        pLayerB = lineToPoint(inputB,"B")
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
        #temporary
        #eq,third = deploy(pLayerA,pStartA,pLayerB,pStartB)
        # circumCenter function with threePointList as input. output cc_midPointList
        #temporary
        #cc_midPointList=[]
        #midStart = midPoint(pStartA,pStartB)        # create start point for equidistant line. hasn't been ch
        #midEnd = midPoint(pEndA,pEndB)                # create end point for equidistant line
        drct = direction(pStartA,pStartB,pEndA,pEndB)
        ppLine = perpendicularLine(pStartA,pStartB,drct,pLayerA,pLayerB)
        lineLayer = QgsVectorLayer("LineString","Line Result", "memory")
        lFeat = QgsFeature()
        lFeat.setGeometry(ppLine)
        lineProv = lineLayer.dataProvider()
        lineProv.addFeatures([lFeat])
        QgsMapLayerRegistry.instance().addMapLayer(lineLayer)

        #temporary
        #cc_midPointList.append(QgsPoint(midStart[0],midStart[1]))            # insert first item to list
        #temporary
        #for i in third:
        #    O = circumCenter(i[0],i[1],i[2])
        #    cc_midPointList.append(O)
        #cc_midPointList.append(QgsPoint(midEnd[0],midEnd[1]))              # insert last item to list

        # line_from_point with midPointList as input. output ed_line_v1

        # line_from_point with cc_midPointList as input. output ed_line_v2
        
        #for i in eq:
        #    cc_midPointList.append(i.asPoint())
        #temporary
        #pointToLine(cc_midPointList)
        #self.textBrowser.append(str(len(cc_midPointList)))
        '''for i in cc_midPointList:
            self.textBrowser.append(str(i))'''
        # compare ver1 vs ver2


        #self.textBrowser.append(str(pLayerA) + " " + str(pLayerB))

    def browseOutfile(self):
        outName = QFileDialog.getSaveFileName(self, "Output Shapefile",self.lineInput.displayText(), "Shapefile (*.shp)")
        if outName != None:
           self.lineInput.setText(outName)

class help_testPluginDialog(QDialog,helpDialog.ui_helpDialog):
    def __init__(self, parent=None):
        """Constructor."""
        super(help_testPluginDialog, self).__init__(parent)
        # Set up the user interface from Designer.
        # After setupUI you can access any designer object by doing
        # self.<objectname>, and you can use autoconnect slots - see
        # http://qt-project.org/doc/qt-4.8/designer-using-a-ui-file.html
        # #widgets-and-dialogs-with-auto-connect


        self.setupUi(self)
'''
class mouseEventTool(QgsMapTool,QDialog,testPlugin_guiNew.Ui_Dialog):
    def __init__(self,canvas):
        self.canvas=canvas
        QgsMapTool.__init__(self,self.canvas)
    def canvasPressEvent(self, click):
        p_canvas=click.pos()
        p_map = self.toMapCoordinates(p_canvas)
        self.textBrowser.append(p_map)
        #QMessageBox.information(None,"hello"," click:\n"+str(p_canvas)+"\n"+str(p_map))

'''