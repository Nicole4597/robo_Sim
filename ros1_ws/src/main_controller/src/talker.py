#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from main_msgs.msg import ActorInfo

if __name__ == '__main__':

    rospy.init_node('actor_info_transmitter', anonymous=True)

    pub = rospy.Publisher("/actor_info", ActorInfo, queue_size=10)

    answer = "Y"
    while answer == "Y":
        msg = ActorInfo()
        msg.actorPosX = int(input("Enter actor's X position: "))
        msg.actorPosY = int(input("Enter actor's Y position: "))
        msg.scenicAction = input("Enter Scenic Action: ")
        pub.publish(msg)
        answer = input("Continue ? (y/n): ").upper()

    rospy.loginfo("Node was stopped")
