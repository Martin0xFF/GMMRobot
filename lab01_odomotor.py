#!/usr/bin/env python
import rospy
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Bonobo():
    def __init__(self, **kwargs):
        # create pub and sub objects
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=kwargs.get('queue', 1))
        self.sub = rospy.Subscriber('odom', Odometry, self.callback, queue_size=1)
        rospy.init_node('bonobo', anonymous=True)
        self.rate = rospy.Rate(kwargs.get('freq', 10))
        self.cur_pose = None
        rospy.loginfo("Rate: %s" % self.rate)

    def get_yaw_from_quarternion(self, q):
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1-2*(q.y*q.y+q.z*q.z)
        yaw = math.atan(siny_cosp/cosy_cosp)
        return yaw

    def callback(self, odom_data):
        point = odom_data.pose.pose.position
        quart = odom_data.pose.pose.orientation
        theta = self.get_yaw_from_quarternion(quart)

        # Save the current Position and Orientation in space in a place Bonobo instance can reach it
        self.cur_pose = (point.x, point.y, theta)
        #rospy.loginfo(self.cur_pose)

    def bonobo_move(self, linear, angular):
        '''
        General function to set velocity of Bonobo movement components
        linear - tuple (x, y, z)
        angular - tuple (0, 0, z)
        '''

        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]

        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]

        #rospy.loginfo("Linear: %f" % float(twist.linear.x))
        #rospy.loginfo("Angular: %f" % float(twist.angular.z))

        self.pub.publish(twist)

    def bonobo_smart_distance(self, displacement, linear, angular):
        '''
        Issues commands to move Bonobo forward and back a specified distance
        linear -  tuple (x, y, z)
        angular - tuple (0, 0, z)
        displacement - float - distance in meters
        '''

        # if the initial readings are not valid, wait until we get a good reading
        while self.cur_pose is None:
            pass

        # create start position and current position variable
        initial_d = tuple(self.cur_pose)
        current = tuple(self.cur_pose)

        # calculate the absolute difference in distance between positions
        norm = math.sqrt((initial_d[0] - current[0])**2 + (initial_d[1] - current[1])**2)

        # if the norm is less than the desired displacement break
        while norm < displacement:
            rospy.loginfo("Start - Current: %i - %i" % (initial_d[0], current[0]))
            self.bonobo_move(linear, angular)
            current = tuple(self.cur_pose)
            norm = math.sqrt((initial_d[0] - current[0])**2 + (initial_d[1] - current[1])**2)
            self.rate.sleep()

    def bonobo_smart_angle(self, angle, linear, angular):
        '''
        Issues commands to rotate Bonobo
        angle - desired  angle
        linear - tuple (x, y, z) velocity
        angular - tuple (x, y, z) angular velocity
        '''
        while self.cur_pose is None:
            pass
        initial_d = tuple(self.cur_pose)
        current = tuple(self.cur_pose)

        # Check to see if the difference between angles is greater than the desired angle, if so, break
        # Works for small angles
        # TODO: FIX THIS METHOD OF CHECKING FOR ANGLE, DOES NOT ACCOUNT FOR [-PI/2, PI/2] RANGE OF OBSERVED ANGLE
        while abs(current[2] - initial_d[2]) < angle:
            rospy.loginfo("Start - Current: %f - %f" % (initial_d[2], current[2]))
            self.bonobo_move(linear, angular)
            current = tuple(self.cur_pose)
            self.rate.sleep()


    def bonobo_hold(self, duration, linear, angular):
        # Time based velocity hold
        start = rospy.Time.now().secs
        while start < 1:
            start = rospy.Time.now().secs
        current = start

        while current - start < duration:
            rospy.loginfo("Start - Current: %i - %i" % (start, current))
            self.bonobo_move(linear, angular)

            current = rospy.Time.now().secs
            rospy.loginfo("Time - Target: %i - %i" % (current - start, duration))
            self.rate.sleep()

    def bonobo_strut(self):
        self.bonobo_hold(6, (0.18, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.bonobo_hold(11, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_hold(2, (0.0, 0.0, 0.0), (0.0, 0.0, 0))

    def bonobo_smart_strut(self):
        self.bonobo_smart_distance(1, (0.18, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.bonobo_smart_angle(math.pi/4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_smart_angle(math.pi/4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_smart_angle(math.pi/4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_smart_angle(math.pi/4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_smart_angle(math.pi/4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_smart_angle(math.pi/4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_smart_angle(math.pi/4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_smart_angle(math.pi/6, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        self.bonobo_hold(2, (0.0, 0.0, 0.0), (0.0, 0.0, 0))

def publisher_node():
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #rospy.init_node('bonobo', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x=0.1
        twist.angular.z=0.1

        #rospy.loginfo(twist.linear)
        #rospy.loginfo(twist.angular)
        cmd_pub.publish(twist)
        rate.sleep()

def main():

    try:
        #rospy.init_node('motor')
        #publisher_node()
        caesar = Bonobo()
        caesar.bonobo_smart_strut()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
