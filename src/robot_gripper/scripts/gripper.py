#!/usr/bin/python3
from matplotlib.pyplot import close
import rospy
import pigpio
from std_msgs.msg import Bool, Int8


class RobotGripper():
    NODE_NAME_TO_SUBSCRIBE = "desired_gripper_pos"
    NODE_NAME_TO_PUBLISH = "gripper_pos"
    

    def __init__(self):
        self.pub = rospy.Publisher(
            self.NODE_NAME_TO_PUBLISH,
            Bool,
            queue_size = 10
        )
        self.sub = rospy.Subscriber(
            self.NODE_NAME_TO_SUBSCRIBE,
            Bool,
            self.callback
        )

        self.rpi = pigpio.pi()
        self.rpi.set_mode(18, pigpio.OUTPUT)

    def callback(self, close: Bool):
        if close.data: 
            self.rpi.set_servo_pulsewidth(18,1500)
            print('should be closed')
        
        else:
            self.rpi.set_servo_pulsewidth(18,2000)
            print('should be open')

        self.pub.publish(close)



def main():
    rospy.init_node('robot_gripper',anonymous = True)
    
    rg = RobotGripper()

    rospy.spin()


if __name__ == '__main__':
    main()