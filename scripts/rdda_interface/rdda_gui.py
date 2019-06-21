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
        self.flag = 1


    def run(self):
        rdda = RddaProxy()
        rate = rospy.Rate(500)

        while not rospy.is_shutdown():
            if self.flag == 1:
                rdda.publish_joint_cmds(self.pos_ref)
                rospy.loginfo("set pos_ref[0]: " + str(self.pos_ref))
                rate.sleep()
            else:
                break

        print 'Terminate ROS'


    def stop(self):
        self.flag == 0



class RddaGui(QtGui.QWidget):

    def __init__(self):
        super(RddaGui, self).__init__()

        self.setObjectName('RddaGui')
        self.init_ui()


    def init_ui(self):

        """Initialize ROS node."""
        self.joint_cmds = ([0.0, 0.0], [0.0, 0.0])
        self.pos_ref = [0.0, 0.0]

        btn = QPushButton()
        btn.setFixedWidth(130)
        btn.setText('Run')
        btn.clicked.connect(self.start_ros)

        hlay = QHBoxLayout()
        hlay.addWidget(btn)
        hlay.addSpacing(50)

        self.label = QLabel()
        self.label.setFixedWidth(140)
        self.label.setText("pos_ref: " + str(0))
        self.label.setEnabled(False)

        hlay.addWidget(self.label)

        slider = QSlider()
        slider.setMinimum(-100)
        slider.setMaximum(100)
        slider.setOrientation(Qt.Horizontal)
        slider.valueChanged.connect(self.set_pos)
        #slider.setTickInterval(0.1)
        #slider.setSingleStep(0.1)

        vlay = QVBoxLayout()
        vlay.addWidget(slider)

        layout = QVBoxLayout()
        layout.addLayout(hlay)
        layout.addLayout(vlay)
        self.setLayout(layout)


    def set_pos(self, value):
        self.label.setText("pos_ref: " + str(value))
        self.pos_ref[0] = value


    def start_ros(self):
        self.thread_pool = []
        rosThread = RosThread(self.pos_ref)
        self.thread_pool.append(rosThread)
        rosThread.start()


if __name__ == '__main__':
    try:
        rospy.init_node('RddaGui')

        app = QtGui.QApplication(sys.argv)
        rddaGui = RddaGui()
        rddaGui.show()
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass