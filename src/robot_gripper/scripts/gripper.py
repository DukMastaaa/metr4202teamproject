#!/usr/bin/python3
import rospy
import pigpio
from std_msgs.msg import Bool


class RobotGripper():
    
    # 1 Node names
    NODE_NAME_TO_SUBSCRIBE = "desired_gripper_pos"
    NODE_NAME_TO_PUBLISH = "gripper_pos"
    
    # 2 Servo motor parameters
    CLOSE = 1850 # pulse width for closed state
    OPEN = 2400 # pulse width for open state
    PWM_PIN = 18 # the pin designated for communicating with the gripper servo motor
    
    # 3 Class initialisation
    def __init__(self):
        
        # 3.1 Setting the publishers and subscribers
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
        
        # 3.2 Setting an object of pigpio.pi() type
        self.rpi = pigpio.pi()
        self.rpi.set_mode(self.PWM_PIN, pigpio.OUTPUT)
        
    # 4 Callback
    def callback(self, close: Bool):
        """
        Based on the close input, the servo motor controlling the gripper sets it to an closed (True) or open (False) state 
        """
        
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
