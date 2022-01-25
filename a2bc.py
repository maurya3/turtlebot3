import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class RobotControl():
    def __init__(self, robot_name="turtlebot"):
        rospy.init_node("robot_control_node", anonymous=True)
        self._check_laser_ready()
        cmd_vel = "/cmd_vel"

        self.vel_publisher = rospy.Publisher(cmd_vel, Twist, queue_size=10)
        self.cmd = Twist()
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
        self.ctrl_c =False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)


    def _check_laser_ready(self):
        self.laser_msg = None
        rospy.loginfo("checking laser")

        while self.laser_msg is None and not rospy.is_shutdown():
            try:
                self.laser_msg = rospy.wait_for_message("/scan",LaserScan,timeout=1)
                rospy.loginfo("laser scan ready")
            except:
                rospy.logerr(" laser is not ready")

            return self.laser_msg

    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections >0:
                self.vel_publisher.publish(self.cmd)
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        self.ctrl_c = True

    def laser_callback(self,msg):
        self.laser_msg =msg

    def get_laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.range

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.publish_once_in_cmd_vel()

    def move_straight(self):
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion,speed,time):
        self.cmd.linear.x = speed
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        if motion =="forward":
            self.cmd.linear.x = speed
        elif motion=="backward":
            self.cmd.linear = -speed

        i = 0
        while (i <= time):
            self.vel_publisher.publish(self.cmd)
            i+=1
            self.rate.sleep()

        self.stop_robot()

    def turn(self,turn_type,speed,time):
        if turn_type=="left":
            self.cmd.angular.z = speed
        elif turn_type=="right":
            self.cmd.angular.z = -speed

        i = 0
        while(i<=time):
            self.vel_publisher.publish(self.cmd)
            i+=1
            self.rate.sleep()


        self.stop_robot()


if __name__=='__main__':
    RobotControl_object = RobotControl()
    try:
        RobotControl_object.move_straight_time('backward',0.2,1)
        RobotControl_object.turn('left',0.1,1)
        RobotControl_object.move_straight_time('forward',0.2,3)
        RobotControl_object.turn('left',0.1,1)
        RobotControl_object.move_straight_time('forward',0.2,3)
        RobotControl_object.stop_robot()
    except rospy.ROSInterruptException:
        pass