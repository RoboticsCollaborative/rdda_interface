#!/usr/bin/env python

import sys
import rospy
from RddaProxy import RddaProxy
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QSlider, QPushButton
from PyQt4.QtCore import Qt


class RosThread(QtCore.QThread):

    def __init__(self, pos_ref):
        QtCore.QThread.__init__(self)
        self.pos_ref = pos_ref
        self._isRunning = True


    def run(self):
        rdda = RddaProxy()
        rate = rospy.Rate(500)

        while not rospy.is_shutdown():
            if self._isRunning == True:
                rdda.publish_joint_cmds(self.pos_ref)
                rospy.loginfo("set pos_ref[0]: " + str(self.pos_ref))
                rate.sleep()
            else:
                break

        print 'Terminate ROS'


    def stop(self):
        self._isRunning = False


class RddaGui(QtGui.QWidget):

    def __init__(self):
        super(RddaGui, self).__init__()

        self.setObjectName('RddaGui')
        self.initUI()


    def initUI(self):

        """Initialize ROS node."""
        self.joint_cmds = ([0.0, 0.0], [0.0, 0.0])

        self.setGeometry(300, 300, 500, 300)
        self.setWindowTitle('RDDA_GUI')

        btnRun = QPushButton('Run')
        btnRun.setFixedWidth(150)
        self.btnCls = QPushButton('Stop')
        self.btnCls.setFixedWidth(150)
        sldPos = QSlider()
        sldPos.setOrientation(Qt.Horizontal)
        sldPos.setMinimum(0)
        sldPos.setMaximum(100)
        sldStf = QSlider()
        sldStf.setOrientation(Qt.Horizontal)
        sldStf.setMinimum(0)
        sldStf.setMaximum(100)
        self.labelPos = QLabel()
        self.labelPos.setFixedWidth(200)
        self.labelPos.setText("Position: " + str(0))
        self.labelStf = QLabel()
        self.labelStf.setFixedWidth(200)
        self.labelStf.setText("Stiffness: " + str(0))

        hboxBtn = QHBoxLayout()
        hboxBtn.addStretch(1)
        hboxBtn.addWidget(btnRun)
        hboxBtn.addSpacing(20)
        hboxBtn.addWidget(self.btnCls)

        hboxPos = QHBoxLayout()
        hboxPos.addWidget(sldPos)
        hboxPos.addSpacing(50)
        hboxPos.addWidget(self.labelPos)

        hboxStf = QHBoxLayout()
        hboxStf.addWidget(sldStf)
        hboxStf.addSpacing(50)
        hboxStf.addWidget(self.labelStf)

        layout = QVBoxLayout()
        layout.addLayout(hboxPos)
        layout.addLayout(hboxStf)
        layout.addLayout(hboxBtn)
        self.setLayout(layout)

        self.rosThread = RosThread(self.joint_cmds)

        btnRun.clicked.connect(self.start_ros)
        self.btnCls.clicked.connect(self.rosThread.stop)
        sldPos.valueChanged.connect(self.set_pos)
        sldStf.valueChanged.connect(self.set_stf)

#        self.finished.connect(self.stop_thread)


    def set_pos(self, value):
        self.labelPos.setText("Position: " + str(value))
        self.joint_cmds[0][0] = -value
        self.joint_cmds[0][1] = value


    def set_stf(self, value):
        self.labelStf.setText("Stifness: " + str(value))
        self.joint_cmds[1][0] = value
        self.joint_cmds[1][1] = value


    def start_ros(self):
        self.thread_pool = []
        self.thread_pool.append(self.rosThread)
        self.rosThread.start()

"""
    def stop_thread(self):
        self.rosThread.stop()
        self.rosThread.quit()
        self.rosThread.wait()
"""

if __name__ == '__main__':
    try:
        rospy.init_node('RddaGui')

        app = QtGui.QApplication(sys.argv)
        rddaGui = RddaGui()
        rddaGui.show()
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass