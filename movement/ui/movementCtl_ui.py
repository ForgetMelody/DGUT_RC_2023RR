# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'movementCtl.ui'
##
## Created by: Qt User Interface Compiler version 6.5.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QDoubleSpinBox, QGroupBox,
    QLabel, QMainWindow, QPushButton, QSizePolicy,
    QSlider, QStatusBar, QTabWidget, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(600, 400)
        MainWindow.setMinimumSize(QSize(600, 400))
        MainWindow.setMaximumSize(QSize(600, 400))
        MainWindow.setContextMenuPolicy(Qt.NoContextMenu)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(-1, 0, 601, 291))
        self.tabWidget.setTabPosition(QTabWidget.North)
        self.tabWidget.setTabShape(QTabWidget.Rounded)
        self.tabWidget.setElideMode(Qt.ElideLeft)
        self.tabWidget.setMovable(False)
        self.tabWidget.setTabBarAutoHide(False)
        self.speed_tab = QWidget()
        self.speed_tab.setObjectName(u"speed_tab")
        self.angularSlider = QSlider(self.speed_tab)
        self.angularSlider.setObjectName(u"angularSlider")
        self.angularSlider.setGeometry(QRect(150, 10, 300, 30))
        self.angularSlider.setMinimum(-3174)
        self.angularSlider.setMaximum(3174)
        self.angularSlider.setSingleStep(10)
        self.angularSlider.setPageStep(10)
        self.angularSlider.setOrientation(Qt.Horizontal)
        self.angularSlider.setInvertedAppearance(False)
        self.angularSlider.setTickPosition(QSlider.NoTicks)
        self.linearSlider = QSlider(self.speed_tab)
        self.linearSlider.setObjectName(u"linearSlider")
        self.linearSlider.setGeometry(QRect(285, 50, 30, 200))
        self.linearSlider.setMinimum(-1195)
        self.linearSlider.setMaximum(1195)
        self.linearSlider.setSingleStep(10)
        self.linearSlider.setPageStep(10)
        self.linearSlider.setSliderPosition(0)
        self.linearSlider.setOrientation(Qt.Vertical)
        self.linearSlider.setInvertedAppearance(False)
        self.linearSlider.setInvertedControls(False)
        self.angularLabel = QLabel(self.speed_tab)
        self.angularLabel.setObjectName(u"angularLabel")
        self.angularLabel.setGeometry(QRect(60, 5, 71, 30))
        font = QFont()
        font.setPointSize(12)
        self.angularLabel.setFont(font)
        self.linearLabel = QLabel(self.speed_tab)
        self.linearLabel.setObjectName(u"linearLabel")
        self.linearLabel.setGeometry(QRect(190, 135, 71, 30))
        self.linearLabel.setFont(font)
        self.stopButton = QPushButton(self.speed_tab)
        self.stopButton.setObjectName(u"stopButton")
        self.stopButton.setGeometry(QRect(480, 200, 100, 50))
        font1 = QFont()
        font1.setPointSize(14)
        self.stopButton.setFont(font1)
        self.angularEdit = QDoubleSpinBox(self.speed_tab)
        self.angularEdit.setObjectName(u"angularEdit")
        self.angularEdit.setGeometry(QRect(460, 10, 90, 30))
        self.angularEdit.setMinimum(-3174.500000000000000)
        self.angularEdit.setMaximum(3174.500000000000000)
        self.angularEdit.setSingleStep(0.500000000000000)
        self.angularEdit.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.linearEdit = QDoubleSpinBox(self.speed_tab)
        self.linearEdit.setObjectName(u"linearEdit")
        self.linearEdit.setGeometry(QRect(320, 135, 90, 30))
        self.linearEdit.setMinimum(-11.949999999999999)
        self.linearEdit.setMaximum(11.949999999999999)
        self.linearEdit.setSingleStep(0.500000000000000)
        self.linearEdit.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.tabWidget.addTab(self.speed_tab, "")
        self.position_tab = QWidget()
        self.position_tab.setObjectName(u"position_tab")
        self.Label1 = QLabel(self.position_tab)
        self.Label1.setObjectName(u"Label1")
        self.Label1.setGeometry(QRect(10, 230, 501, 30))
        self.Label1.setFont(font)
        self.moveLabel = QLabel(self.position_tab)
        self.moveLabel.setObjectName(u"moveLabel")
        self.moveLabel.setGeometry(QRect(30, 50, 81, 30))
        self.moveLabel.setFont(font)
        self.turnLabel = QLabel(self.position_tab)
        self.turnLabel.setObjectName(u"turnLabel")
        self.turnLabel.setGeometry(QRect(30, 120, 81, 30))
        self.turnLabel.setFont(font)
        self.publishButton = QPushButton(self.position_tab)
        self.publishButton.setObjectName(u"publishButton")
        self.publishButton.setGeometry(QRect(380, 50, 200, 100))
        self.publishButton.setFont(font1)
        self.moveEdit = QDoubleSpinBox(self.position_tab)
        self.moveEdit.setObjectName(u"moveEdit")
        self.moveEdit.setGeometry(QRect(110, 50, 240, 40))
        self.moveEdit.setMinimum(-99999.990000000005239)
        self.moveEdit.setMaximum(99999.990000000005239)
        self.moveEdit.setSingleStep(0.100000000000000)
        self.moveEdit.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.turnEdit = QDoubleSpinBox(self.position_tab)
        self.turnEdit.setObjectName(u"turnEdit")
        self.turnEdit.setGeometry(QRect(110, 120, 240, 40))
        self.turnEdit.setMinimum(-99999.990000000005239)
        self.turnEdit.setMaximum(99999.990000000005239)
        self.turnEdit.setSingleStep(0.100000000000000)
        self.turnEdit.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.tabWidget.addTab(self.position_tab, "")
        self.lreflactBox = QGroupBox(self.centralwidget)
        self.lreflactBox.setObjectName(u"lreflactBox")
        self.lreflactBox.setGeometry(QRect(5, 290, 290, 80))
        self.lreflactBox.setFont(font)
        self.lreflactBox.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.lreflactBox.setFlat(False)
        self.lrpmLabel = QLabel(self.lreflactBox)
        self.lrpmLabel.setObjectName(u"lrpmLabel")
        self.lrpmLabel.setGeometry(QRect(10, 30, 41, 30))
        self.lrangeLabel = QLabel(self.lreflactBox)
        self.lrangeLabel.setObjectName(u"lrangeLabel")
        self.lrangeLabel.setGeometry(QRect(150, 30, 51, 30))
        self.lrpmValLabel = QLabel(self.lreflactBox)
        self.lrpmValLabel.setObjectName(u"lrpmValLabel")
        self.lrpmValLabel.setGeometry(QRect(60, 30, 81, 30))
        self.lrangeValLabel = QLabel(self.lreflactBox)
        self.lrangeValLabel.setObjectName(u"lrangeValLabel")
        self.lrangeValLabel.setGeometry(QRect(210, 30, 71, 30))
        self.rreflactBox = QGroupBox(self.centralwidget)
        self.rreflactBox.setObjectName(u"rreflactBox")
        self.rreflactBox.setGeometry(QRect(305, 290, 290, 80))
        self.rreflactBox.setFont(font)
        self.rreflactBox.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.rreflactBox.setFlat(False)
        self.rreflactBox.setCheckable(False)
        self.rreflactBox.setChecked(False)
        self.rrpmLabel = QLabel(self.rreflactBox)
        self.rrpmLabel.setObjectName(u"rrpmLabel")
        self.rrpmLabel.setGeometry(QRect(10, 30, 41, 30))
        self.rrangeLabel = QLabel(self.rreflactBox)
        self.rrangeLabel.setObjectName(u"rrangeLabel")
        self.rrangeLabel.setGeometry(QRect(150, 30, 51, 30))
        self.rrpmValLabel = QLabel(self.rreflactBox)
        self.rrpmValLabel.setObjectName(u"rrpmValLabel")
        self.rrpmValLabel.setGeometry(QRect(60, 30, 81, 30))
        self.rrangeValLabel = QLabel(self.rreflactBox)
        self.rrangeValLabel.setObjectName(u"rrangeValLabel")
        self.rrangeValLabel.setGeometry(QRect(210, 30, 71, 30))
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"\u79fb\u52a8\u63a7\u5236", None))
#if QT_CONFIG(statustip)
        self.speed_tab.setStatusTip("")
#endif // QT_CONFIG(statustip)
        self.angularLabel.setText(QCoreApplication.translate("MainWindow", u"\u8f6c\u5411\u901f\u5ea6:", None))
        self.linearLabel.setText(QCoreApplication.translate("MainWindow", u"\u524d\u540e\u901f\u5ea6:", None))
        self.stopButton.setText(QCoreApplication.translate("MainWindow", u"\u6025\u505c", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.speed_tab), QCoreApplication.translate("MainWindow", u"\u901f\u5ea6\u73af", None))
        self.Label1.setText(QCoreApplication.translate("MainWindow", u"\u6ce8:\u4f4d\u7f6e\u73af\u8bbe\u7f6e\u4e3a\u53ea\u80fd\u5355\u72ec\u8fdb\u884c\u524d\u540e\u79fb\u52a8\u6216\u8005\u8f6c\u5411\uff0c\u4e0d\u53ef\u540c\u65f6\u8fdb\u884c", None))
        self.moveLabel.setText(QCoreApplication.translate("MainWindow", u"\u524d\u540e\u79fb\u52a8:", None))
        self.turnLabel.setText(QCoreApplication.translate("MainWindow", u"\u5de6\u53f3\u65cb\u8f6c:", None))
        self.publishButton.setText(QCoreApplication.translate("MainWindow", u"\u53d1\u5e03\u6307\u4ee4", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.position_tab), QCoreApplication.translate("MainWindow", u"\u4f4d\u7f6e\u73af", None))
        self.lreflactBox.setTitle(QCoreApplication.translate("MainWindow", u"\u5de6\u7535\u673a", None))
        self.lrpmLabel.setText(QCoreApplication.translate("MainWindow", u"RPM:", None))
        self.lrangeLabel.setText(QCoreApplication.translate("MainWindow", u"\u91cc\u7a0b\u8ba1:", None))
        self.lrpmValLabel.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.lrangeValLabel.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rreflactBox.setTitle(QCoreApplication.translate("MainWindow", u"\u53f3\u7535\u673a", None))
        self.rrpmLabel.setText(QCoreApplication.translate("MainWindow", u"RPM:", None))
        self.rrangeLabel.setText(QCoreApplication.translate("MainWindow", u"\u91cc\u7a0b\u8ba1:", None))
        self.rrpmValLabel.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.rrangeValLabel.setText(QCoreApplication.translate("MainWindow", u"0", None))
    # retranslateUi
