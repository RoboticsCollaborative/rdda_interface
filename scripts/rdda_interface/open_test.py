#!/usr/bin/env python

import rospy
from RddaProxy import RddaProxy


def main():
    rdda = RddaProxy()

    rdda.open()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass