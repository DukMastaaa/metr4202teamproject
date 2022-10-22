#!/usr/bin/python3
import rospy
import pigpio
from std_msgs.msg import Bool


class RobotGripper():
    NODE_NAME_TO_SUBSCRIBE = "desired_gripper_pos"
    NODE_NAME_TO_PUBLISH = "gripper_pos"
    
    CLOSE = 1850
    OPEN = 2400
    PWM_PIN = 18

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
        self.rpi.set_mode(self.PWM_PIN, pigpio.OUTPUT)

    def callback(self, close: Bool):
        if close.data: 
            self.rpi.set_servo_pulsewidth(self.PWM_PIN, self.CLOSE)
            print('should be closed')
        
        else:
            self.rpi.set_servo_pulsewidth(self.PWM_PIN, self.OPEN)
            print('should be open')

        self.pub.publish(close)



def main():
    rospy.init_node('robot_gripper',anonymous = True)
    
    rg = RobotGripper()

    rospy.spin()


if __name__ == '__main__':
    main()