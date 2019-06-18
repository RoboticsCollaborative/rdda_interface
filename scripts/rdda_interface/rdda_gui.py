#!/usr/bin/env python

import sys
import rospy
from RddaProxy import RddaProxy
from PyQt4 import QtGui
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QSlider, QPushButton
from PyQt4.QtCore import Qt


class RddaGui(QtGui.QWidget):

    def __init__(self):
        super(RddaGui, self).__init__()

        self.setObjectName('RddaGui')
        self.init_ui()


    def init_ui(self):

        """Initialize ROS node."""
        self.joint_cmds = ([0.0, 0.0], [0.0, 0.0])
        self.pos_ref = [0.0, 0.0]
        self.rdda = RddaProxy()

        btn = QPushButton()
        btn.setFixedWidth(130)
        btn.setText('Publisher')
        btn.clicked.connect(self.pub_pos)

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

    def pub_pos(self):
        self.rdda.publish_joint_cmds(self.pos_ref)

        rospy.loginfo("set pos_ref[0]: " + str(self.pos_ref))


    def set_pos(self, value):
        self.label.setText("pos_ref: " + str(value))
        self.pos_ref[0] = value


if __name__ == '__main__':
    try:
        rospy.init_node('RddaGui')

        app = QtGui.QApplication(sys.argv)
        rddaGui = RddaGui()
        rddaGui.show()
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass