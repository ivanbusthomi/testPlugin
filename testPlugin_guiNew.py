# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'testPlugin.ui'
#
# Created: Sun Sep 07 19:50:38 2014
#      by: PyQt4 UI code generator 4.8.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
#import helpDialog

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(366, 392)
        self.gridLayout_11 = QtGui.QGridLayout(Dialog)
        self.gridLayout_11.setObjectName(_fromUtf8("gridLayout_11"))
        self.groupBoxA = QtGui.QGroupBox(Dialog)
        self.groupBoxA.setObjectName(_fromUtf8("groupBoxA"))
        self.gridLayout_9 = QtGui.QGridLayout(self.groupBoxA)
        self.gridLayout_9.setObjectName(_fromUtf8("gridLayout_9"))
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.comboInputA = QtGui.QComboBox(self.groupBoxA)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.comboInputA.sizePolicy().hasHeightForWidth())
        self.comboInputA.setSizePolicy(sizePolicy)
        self.comboInputA.setObjectName(_fromUtf8("comboInputA"))
        self.gridLayout_2.addWidget(self.comboInputA, 0, 0, 1, 1)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.buttonStartA = QtGui.QToolButton(self.groupBoxA)
        self.buttonStartA.setObjectName(_fromUtf8("buttonStartA"))
        self.gridLayout.addWidget(self.buttonStartA, 0, 0, 1, 1)
        self.buttonEndA = QtGui.QToolButton(self.groupBoxA)
        self.buttonEndA.setObjectName(_fromUtf8("buttonEndA"))
        self.gridLayout.addWidget(self.buttonEndA, 0, 1, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 1, 1, 1)
        self.gridLayout_9.addLayout(self.gridLayout_2, 0, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBoxA, 0, 0, 1, 3)
        self.groupBoxB = QtGui.QGroupBox(Dialog)
        self.groupBoxB.setObjectName(_fromUtf8("groupBoxB"))
        self.gridLayout_8 = QtGui.QGridLayout(self.groupBoxB)
        self.gridLayout_8.setObjectName(_fromUtf8("gridLayout_8"))
        self.gridLayout_4 = QtGui.QGridLayout()
        self.gridLayout_4.setObjectName(_fromUtf8("gridLayout_4"))
        self.comboInputB = QtGui.QComboBox(self.groupBoxB)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.comboInputB.sizePolicy().hasHeightForWidth())
        self.comboInputB.setSizePolicy(sizePolicy)
        self.comboInputB.setObjectName(_fromUtf8("comboInputB"))
        self.gridLayout_4.addWidget(self.comboInputB, 0, 0, 1, 1)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.buttonStartB = QtGui.QToolButton(self.groupBoxB)
        self.buttonStartB.setObjectName(_fromUtf8("buttonStartB"))
        self.gridLayout_3.addWidget(self.buttonStartB, 0, 0, 1, 1)
        self.buttonStartB_2 = QtGui.QToolButton(self.groupBoxB)
        self.buttonStartB_2.setObjectName(_fromUtf8("buttonStartB_2"))
        self.gridLayout_3.addWidget(self.buttonStartB_2, 0, 1, 1, 1)
        self.gridLayout_4.addLayout(self.gridLayout_3, 0, 1, 1, 1)
        self.gridLayout_8.addLayout(self.gridLayout_4, 0, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBoxB, 1, 0, 1, 3)
        self.groupBoxOutput = QtGui.QGroupBox(Dialog)
        self.groupBoxOutput.setObjectName(_fromUtf8("groupBoxOutput"))
        self.gridLayout_10 = QtGui.QGridLayout(self.groupBoxOutput)
        self.gridLayout_10.setObjectName(_fromUtf8("gridLayout_10"))
        self.gridLayout_5 = QtGui.QGridLayout()
        self.gridLayout_5.setObjectName(_fromUtf8("gridLayout_5"))
        self.lineInput = QtGui.QLineEdit(self.groupBoxOutput)
        self.lineInput.setObjectName(_fromUtf8("lineInput"))
        self.gridLayout_5.addWidget(self.lineInput, 0, 0, 1, 1)
        self.buttonBrowse = QtGui.QPushButton(self.groupBoxOutput)
        self.buttonBrowse.setObjectName(_fromUtf8("buttonBrowse"))
        self.gridLayout_5.addWidget(self.buttonBrowse, 0, 1, 1, 1)
        self.gridLayout_10.addLayout(self.gridLayout_5, 0, 0, 1, 1)
        self.gridLayout_6 = QtGui.QGridLayout()
        self.gridLayout_6.setObjectName(_fromUtf8("gridLayout_6"))
        self.label_3 = QtGui.QLabel(self.groupBoxOutput)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout_6.addWidget(self.label_3, 0, 0, 1, 1)
        spacerItem = QtGui.QSpacerItem(138, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout_6.addItem(spacerItem, 0, 1, 1, 1)
        self.spinboxInterval = QtGui.QDoubleSpinBox(self.groupBoxOutput)
        self.spinboxInterval.setMinimum(0.01)
        self.spinboxInterval.setMaximum(10000.0)
        self.spinboxInterval.setObjectName(_fromUtf8("spinboxInterval"))
        self.gridLayout_6.addWidget(self.spinboxInterval, 0, 2, 1, 1)
        self.gridLayout_10.addLayout(self.gridLayout_6, 1, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBoxOutput, 2, 0, 1, 3)
        self.buttonHelp = QtGui.QPushButton(Dialog)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.buttonHelp.sizePolicy().hasHeightForWidth())
        self.buttonHelp.setSizePolicy(sizePolicy)
        self.buttonHelp.setObjectName(_fromUtf8("buttonHelp"))
        self.gridLayout_11.addWidget(self.buttonHelp, 3, 0, 1, 1)
        spacerItem1 = QtGui.QSpacerItem(94, 22, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout_11.addItem(spacerItem1, 3, 1, 1, 1)
        self.gridLayout_7 = QtGui.QGridLayout()
        self.gridLayout_7.setObjectName(_fromUtf8("gridLayout_7"))
        self.buttonRun = QtGui.QPushButton(Dialog)
        self.buttonRun.setObjectName(_fromUtf8("buttonRun"))
        self.gridLayout_7.addWidget(self.buttonRun, 0, 0, 1, 1)
        self.buttonClose = QtGui.QPushButton(Dialog)
        self.buttonClose.setObjectName(_fromUtf8("buttonClose"))
        self.gridLayout_7.addWidget(self.buttonClose, 0, 1, 1, 1)
        self.gridLayout_11.addLayout(self.gridLayout_7, 3, 2, 1, 1)
        self.textBrowser = QtGui.QTextBrowser(Dialog)
        self.textBrowser.setObjectName(_fromUtf8("textBrowser"))
        self.gridLayout_11.addWidget(self.textBrowser, 4, 0, 1, 3)

        self.retranslateUi(Dialog)
        QtCore.QObject.connect(self.buttonClose, QtCore.SIGNAL(_fromUtf8("clicked()")), Dialog.close)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
        Dialog.setTabOrder(self.comboInputA, self.lineInput)
        Dialog.setTabOrder(self.lineInput, self.buttonBrowse)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QtGui.QApplication.translate("Dialog", "Dialog", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBoxA.setTitle(QtGui.QApplication.translate("Dialog", "Input Layer A", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonStartA.setText(QtGui.QApplication.translate("Dialog", "S", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonEndA.setText(QtGui.QApplication.translate("Dialog", "E", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBoxB.setTitle(QtGui.QApplication.translate("Dialog", "Input Layer B", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonStartB.setText(QtGui.QApplication.translate("Dialog", "S", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonStartB_2.setText(QtGui.QApplication.translate("Dialog", "E", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBoxOutput.setTitle(QtGui.QApplication.translate("Dialog", "Output", None, QtGui.QApplication.UnicodeUTF8))
        self.lineInput.setPlaceholderText(QtGui.QApplication.translate("Dialog", "Define directory for output shapefile", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonBrowse.setText(QtGui.QApplication.translate("Dialog", "Browse...", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("Dialog", "Interval (m)", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonHelp.setText(QtGui.QApplication.translate("Dialog", "?", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonRun.setText(QtGui.QApplication.translate("Dialog", "Run", None, QtGui.QApplication.UnicodeUTF8))
        self.buttonClose.setText(QtGui.QApplication.translate("Dialog", "Close", None, QtGui.QApplication.UnicodeUTF8))

