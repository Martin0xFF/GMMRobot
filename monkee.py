#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
import time
from collections import deque
import numpy as np
from lab01_odomotor import Bonobo
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey(): #you can ignore this function. It's for stopping the robot when press 'Ctrl+C'
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



class PIDcontrol(Bonobo):
    def __init__(self, **kwargs):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
        self.monkee_vel = rospy.Subscriber('monkee_vel', String, self.monkee_vel_callback, queue_size=1)
        self.mvel = 0.1

        self.rate = rospy.Rate(kwargs.get('freq', 10))
        self.cur_pose = None
        self.target = None
        rospy.loginfo("Rate: %s" % self.rate)

        self.integral = None
        self.error = None
        self.last_error = None
        self.data = deque([0]*25, 25)
        self.cur_data = 0

    def monkee_vel_callback(self, data):
        data_in = float(data.data)
        self.mvel = data_in

    def camera_callback(self, data):
        data_in = int(data.data)
        self.cur_data = data_in
        if self.target is not None:
            if self.last_error is not None and data_in == 0:

                    if self.last_error > 0:
                        self.error = self.target
                    else:
                        self.error = - self.target

                    # Assume error is zero if you're not seeing a line, and just drive straight
                    if self.last_error is not None and data_in == 0:
                        self.error = 0


            else:
                self.error = data_in - self.target

    def _check_input(self):
        '''
        Do not run the algo unless we know the camera is working
        '''
        if self.error is None:
            start = time.time()
            while (self.error is None):
                if (time.time() - start) > 60:
                    raise Exception("Time Out")

    def monkee(self, feral=False):
        self.target = 320
        self._check_input()
        if self.integral is None:
            self.integral = self.error
        if self.last_error is None:
            self.last_error = 0

        max_lin_vel = self.mvel
        min_lin_vel = 0.0
        max_ang_vel = 1.82

        kp = 0.83 #8
        ki = 0.0001 #0.0005
        kd = 2.3 #3

        int_bound = 640

        self.integral = self.integral + self.error
        if abs(self.integral) > int_bound:
            if self.integral > 0.0:
                self.integral = int_bound
            else:
                self.integral = -int_bound

        self.derivative = self.error - self.last_error
        correction = -((kp*self.error)/320.0 + (ki * self.integral)/320.0 + (kd * self.derivative)/320.0)

        lc = 1/(abs(correction) + 1) if not feral else 1
        vc = max(min_lin_vel + lc*(max_lin_vel - min_lin_vel), 0.08)

        self.bonobo_move((min(vc, max_lin_vel), 0, 0), (0, 0, min(correction*max_ang_vel, 1.82)))
        self.last_error = self.error
        self.rate.sleep()



if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('Lab3')
    PID = PIDcontrol()

    try:
        deliverable = "feral"
        while(1):
            key = getKey()
            if deliverable == "monkee":
                PID.monkee()
            elif "feral":
                PID.monkee(feral=True)

            if (key == '\x03'): #stop the robot when exit the program
                PID.bonobo_hold(1,(0,0,0), (0,0,0))
                break

    except rospy.ROSInterruptException:
        print("comm failed")


