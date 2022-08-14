# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'v1.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from tokenize import String
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QPixmap
import cv2
import numpy as np

# ========================= #
import rospy
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
# ========================= #

class VideoThread(QThread):
    img_hand_signal = pyqtSignal(np.ndarray)
    img_yolo_signal = pyqtSignal(np.ndarray)
    img_plan_signal = pyqtSignal(np.ndarray)
    cap = cv2.VideoCapture(0)
    rospy.init_node('user_interface_node', anonymous=True)
    bridge = CvBridge()
    cmd_publisher = rospy.Publisher('/plan_command', String, queue_size=1)

    def img_hand_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_hand_signal.emit(cv_img)
        except CvBridgeError as e:
            rospy.logerr('bridge error {e}')

    def img_yolo_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_yolo_signal.emit(cv_img)
        except CvBridgeError as e:
            rospy.logerr('bridge error {e}')

    def img_plan_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_plan_signal.emit(cv_img)
        except CvBridgeError as e:
            rospy.logerr('bridge error {e}')

    def send_command(self, cmd):
        rospy.wait_for_service('/pathplan/command')
        try:
            send_command_service = rospy.ServiceProxy('/pathplan/command', String)
            resp1 = send_command_service(cmd)
            return resp1.data
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def run(self):
        rospy.Subscriber('/hand/image', Image, self.img_hand_callback, queue_size=1)
        rospy.Subscriber('/yolo/image', Image, self.img_yolo_callback, queue_size=1)
        rospy.Subscriber('/pathplan/image', Image, self.img_plan_callback, queue_size=1)
        rospy.spin()
        # while True:
        #     ret, cv_img = self.cap.read()
        #     if ret:
        #         self.img_hand_signal.emit(cv_img)
        #         self.img_yolo_signal.emit(cv_img)
        #         self.img_plan_signal.emit(cv_img)


class App(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi()

    def setupUi(self):
        self.setObjectName("MainWindow")
        self.resize(1280, 720)
        self.setMaximumSize(QtCore.QSize(1280, 720))
        self.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(820, 10, 302, 331))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.simImageLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.simImageLayout.setContentsMargins(0, 0, 0, 0)
        self.simImageLayout.setObjectName("simImageLayout")
        self.simImage = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.simImage.setMinimumSize(QtCore.QSize(300, 300))
        self.simImage.setMaximumSize(QtCore.QSize(300, 300))
        self.simImage.setText("")
        self.simImage.setPixmap(QtGui.QPixmap("321266.jpg"))
        self.simImage.setAlignment(QtCore.Qt.AlignCenter)
        self.simImage.setObjectName("simImage")
        self.simImageLayout.addWidget(self.simImage)
        self.simLable = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.simLable.setTextFormat(QtCore.Qt.AutoText)
        self.simLable.setAlignment(QtCore.Qt.AlignCenter)
        self.simLable.setObjectName("simLable")
        self.simImageLayout.addWidget(self.simLable)
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(50, 300, 485, 390))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.realImageLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.realImageLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.realImageLayout.setContentsMargins(0, 0, 0, 0)
        self.realImageLayout.setObjectName("realImageLayout")
        self.realImage = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.realImage.setMinimumSize(QtCore.QSize(480, 360))
        self.realImage.setMaximumSize(QtCore.QSize(480, 360))
        self.realImage.setText("")
        self.realImage.setPixmap(QtGui.QPixmap("86866185_p0_master1200.jpg"))
        self.realImage.setObjectName("realImage")
        self.realImageLayout.addWidget(self.realImage)
        self.realLabel = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.realLabel.setMaximumSize(QtCore.QSize(640, 50))
        self.realLabel.setTextFormat(QtCore.Qt.AutoText)
        self.realLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.realLabel.setObjectName("realLabel")
        self.realImageLayout.addWidget(self.realLabel)
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(820, 370, 386, 320))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.yoloLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.yoloLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.yoloLayout.setContentsMargins(0, 0, 0, 0)
        self.yoloLayout.setObjectName("yoloLayout")
        self.yoloImage = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.yoloImage.setMinimumSize(QtCore.QSize(384, 288))
        self.yoloImage.setMaximumSize(QtCore.QSize(384, 288))
        self.yoloImage.setText("")
        self.yoloImage.setPixmap(QtGui.QPixmap("321266.jpg"))
        self.yoloImage.setObjectName("yoloImage")
        self.yoloLayout.addWidget(self.yoloImage)
        self.yoloLabel = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.yoloLabel.setMaximumSize(QtCore.QSize(640, 50))
        self.yoloLabel.setTextFormat(QtCore.Qt.AutoText)
        self.yoloLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.yoloLabel.setObjectName("yoloLabel")
        self.yoloLayout.addWidget(self.yoloLabel)
        self.verticalLayoutWidget_4 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(570, 265, 200, 400))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.panelLayout = QtWidgets.QGridLayout(self.verticalLayoutWidget_4)
        self.panelLayout.setContentsMargins(0, 0, 0, 0)
        self.panelLayout.setObjectName("panelLayout")
        self.initButton = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.initButton.setMinimumSize(QtCore.QSize(0, 40))
        self.initButton.setObjectName("initButton")
        self.initButton.clicked.connect(self.on_init_button_click)
        self.panelLayout.addWidget(self.initButton, 1, 0, 1, 1)
        self.pauseButton = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.pauseButton.setMinimumSize(QtCore.QSize(0, 80))
        self.pauseButton.setObjectName("pauseButton")
        self.pauseButton.setEnabled(False)
        self.pauseButton.clicked.connect(self.on_pause_button_click)
        self.panelLayout.addWidget(self.pauseButton, 10, 0, 1, 1)
        self.stopButton = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.stopButton.setMinimumSize(QtCore.QSize(0, 40))
        self.stopButton.setObjectName("stopButton")
        self.stopButton.setEnabled(False)
        self.stopButton.clicked.connect(self.on_stop_button_click)
        self.panelLayout.addWidget(self.stopButton, 7, 0, 1, 1)
        self.simulateCheckBox = QtWidgets.QCheckBox(self.verticalLayoutWidget_4)
        self.simulateCheckBox.setEnabled(True)
        self.simulateCheckBox.setMinimumSize(QtCore.QSize(0, 50))
        font = QtGui.QFont()
        font.setFamily("Agency FB")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.simulateCheckBox.setFont(font)
        self.simulateCheckBox.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.simulateCheckBox.setAutoFillBackground(False)
        self.simulateCheckBox.setStyleSheet("margin-left:10%; margin-right:50%;")
        self.simulateCheckBox.setInputMethodHints(QtCore.Qt.ImhUrlCharactersOnly)
        self.simulateCheckBox.setChecked(False)
        self.simulateCheckBox.setAutoRepeat(False)
        self.simulateCheckBox.setAutoExclusive(False)
        self.simulateCheckBox.setObjectName("simulateCheckBox")
        self.panelLayout.addWidget(self.simulateCheckBox, 4, 0, 1, 1)
        self.startButton = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.startButton.setMinimumSize(QtCore.QSize(0, 40))
        self.startButton.setObjectName("startButton")
        self.startButton.setEnabled(False)
        self.startButton.clicked.connect(self.on_start_button_click)
        self.panelLayout.addWidget(self.startButton, 6, 0, 1, 1)
        self.removeButton = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.removeButton.setMinimumSize(QtCore.QSize(0, 40))
        self.removeButton.setObjectName("removeButton")
        self.removeButton.setEnabled(False)
        self.removeButton.clicked.connect(self.on_remove_button_click)
        self.panelLayout.addWidget(self.removeButton, 5, 0, 1, 1)
        self.detectButton = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.detectButton.setMinimumSize(QtCore.QSize(0, 40))
        self.detectButton.setObjectName("detectButton")
        self.detectButton.setEnabled(False)
        self.detectButton.clicked.connect(self.on_detect_button_click)
        self.panelLayout.addWidget(self.detectButton, 2, 0, 1, 1)
        self.verticalLayoutWidget_5 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_5.setGeometry(QtCore.QRect(50, 80, 529, 110))
        self.verticalLayoutWidget_5.setObjectName("verticalLayoutWidget_5")
        self.titleLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_5)
        self.titleLayout.setContentsMargins(0, 0, 0, 0)
        self.titleLayout.setObjectName("titleLayout")
        self.mainTitle = QtWidgets.QLabel(self.verticalLayoutWidget_5)
        self.mainTitle.setObjectName("mainTitle")
        self.titleLayout.addWidget(self.mainTitle)
        self.subTitle = QtWidgets.QLabel(self.verticalLayoutWidget_5)
        self.subTitle.setObjectName("subTitle")
        self.titleLayout.addWidget(self.subTitle)
        self.verticalLayoutWidget_6 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(1130, 230, 160, 80))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.captionLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_6)
        self.captionLayout.setContentsMargins(0, 0, 0, 0)
        self.captionLayout.setObjectName("captionLayout")
        self.captionHole = QtWidgets.QCheckBox(self.verticalLayoutWidget_6)
        self.captionHole.setStyleSheet("QCheckBox::indicator { background-color : lightgreen; } QCheckBox::indicator { width : 20px; height : 20px; } QCheckBox { spacing: 5px; font-size:20px; font-weight: bold; }")
        self.captionHole.setIconSize(QtCore.QSize(20, 20))
        self.captionHole.setCheckable(False)
        self.captionHole.setChecked(False)
        self.captionHole.setObjectName("captionHole")
        self.captionLayout.addWidget(self.captionHole)
        self.captionErrorPin = QtWidgets.QCheckBox(self.verticalLayoutWidget_6)
        self.captionErrorPin.setStyleSheet("QCheckBox::indicator { background-color : red; } QCheckBox::indicator { width : 20px; height : 20px; } QCheckBox { spacing: 5px; font-size:20px; font-weight: bold; }")
        self.captionErrorPin.setIconSize(QtCore.QSize(20, 20))
        self.captionErrorPin.setCheckable(False)
        self.captionErrorPin.setChecked(False)
        self.captionErrorPin.setObjectName("captionErrorPin")
        self.captionLayout.addWidget(self.captionErrorPin)
        self.captionPin = QtWidgets.QCheckBox(self.verticalLayoutWidget_6)
        self.captionPin.setStyleSheet("QCheckBox::indicator { background-color : blue; } QCheckBox::indicator { width : 20px; height : 20px; } QCheckBox { spacing: 5px; font-size:20px; font-weight: bold; }")
        self.captionPin.setIconSize(QtCore.QSize(20, 20))
        self.captionPin.setCheckable(False)
        self.captionPin.setChecked(False)
        self.captionPin.setObjectName("captionPin")
        self.captionLayout.addWidget(self.captionPin)
        self.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1280, 21))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)
        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

        self.thread = VideoThread()
        self.thread.img_hand_signal.connect(self.update_sim_image)
        self.thread.img_yolo_signal.connect(self.update_yolo_image)
        self.thread.img_plan_signal.connect(self.update_real_image)
        self.thread.start()

    @pyqtSlot(np.ndarray)
    def update_sim_image(self, image):
        self.simImage.setPixmap(self.convert_cv_qt(image, 300, 300))

    @pyqtSlot(np.ndarray)
    def update_yolo_image(self, image):
        self.yoloImage.setPixmap(self.convert_cv_qt(image, 384, 288))

    @pyqtSlot(np.ndarray)
    def update_real_image(self, image):
        self.realImage.setPixmap(self.convert_cv_qt(image, 480, 360, Qt.KeepAspectRatio))

    def on_init_button_click(self):
        self.thread.send_command("init")
        self.detectButton.setEnabled(True)

    def on_detect_button_click(self):
        self.thread.send_command("detect")
        self.removeButton.setEnabled(True)
        self.pauseButton.setEnabled(True)
        self.stopButton.setEnabled(True)
        self.startButton.setEnabled(True)

    def on_remove_button_click(self):
        self.thread.send_command("remove_ng_pin")
        pass

    def on_pause_button_click(self):
        self.thread.send_command("pause")
        pass

    def on_stop_button_click(self):
        self.thread.send_command("end")
        pass

    def on_start_button_click(self):
        self.thread.send_command("start")
        pass

    def convert_cv_qt(self, cv_img, width, height, mode=Qt.IgnoreAspectRatio):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(width, height, mode)
        return QPixmap.fromImage(p)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "人機協作系統操作介面"))
        self.simLable.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:14pt; font-weight:600;\">模擬畫面</span></p></body></html>"))
        self.realLabel.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:14pt; font-weight:600;\">實際畫面</span></p></body></html>"))
        self.yoloLabel.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:14pt; font-weight:600;\">YOLO檢測畫面</span></p></body></html>"))
        self.initButton.setText(_translate("MainWindow", "前置設定"))
        self.pauseButton.setText(_translate("MainWindow", "暫停"))
        self.stopButton.setText(_translate("MainWindow", "結束運行"))
        self.simulateCheckBox.setText(_translate("MainWindow", "模擬模式"))
        self.startButton.setText(_translate("MainWindow", "開始運行"))
        self.removeButton.setText(_translate("MainWindow", "去除瑕疵品"))
        self.detectButton.setText(_translate("MainWindow", "YOLO檢測"))
        self.mainTitle.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:48pt; font-weight:600;\">ROBOT CAN HELP</span></p></body></html>"))
        self.subTitle.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:20pt;\">人機協作系統操作介面</span></p></body></html>"))
        self.captionHole.setText(_translate("MainWindow", "HOLE"))
        self.captionErrorPin.setText(_translate("MainWindow", "NG PIN"))
        self.captionPin.setText(_translate("MainWindow", "PIN"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ui = App()
    ui.show()
    sys.exit(app.exec_())