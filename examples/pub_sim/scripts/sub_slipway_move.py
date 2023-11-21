import rospy
from pub_sim.msg import Slipway_control

def domsg(s):
    rospy.loginfo("发布的信号为 x轴移动: %d,y轴移动: %d,z轴移动: %d",s.x, s.y, s.z)
    return 0


if __name__ == "__main__":
    rospy.init_node("slipway_sub")
    sub = rospy.Subscriber("slipway_control", Slipway_control, domsg, queue_size=10)

    rospy.spin()


