import rospy
from pub_sim.msg import Slipway_control

def slipwaycontrol(x, y, z):
    slipway = Slipway_control()
    slipway.x = x
    slipway.y = y
    slipway.z = z
    return slipway


if __name__ == "__main__":
    rospy.init_node("slipway_pub")
    pub = rospy.Publisher("slipway_control", Slipway_control, queue_size=10)

    rospy.sleep(5)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        s = slipwaycontrol(10, 10, 10)
        pub.publish(s)

        rospy.loginfo("发布的信号为 x轴移动: %d,y轴移动: %d,z轴移动: %d",s.x, s.y, s.z)
        rate.sleep()


