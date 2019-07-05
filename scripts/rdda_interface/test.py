#!/usr/bin/env python
import rospy
from RddaProxy import RddaProxy


def main():
    rdda = RddaProxy()
    rospy.init_node('homing', anonymous=True)

    rdda.homing()
    # rdda.harmonic_wave()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
