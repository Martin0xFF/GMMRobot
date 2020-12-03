#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os
import monkee
from probabilistic_neuron_activation import HaramBayesian

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

class final_monkee(monkee.PIDcontrol):
    def online(self):
        if self.cur_data !=0:
            return True
        else:
            return False

    def camera_callback(self, data):
        data_in = int(data.data)
        self.cur_data = data_in
        if self.target is not None:
            if self.last_error is not None and data_in == 0:
                    # This is "sane return to line". For final project, replacing this with "go straight"
                    '''
                    if self.last_error > 0:
                        self.error = self.target
                    else:
                        self.error = - self.target
                    '''

                    # Assume error is zero if you're not seeing a line, and just drive straight
                    if self.last_error is not None and data_in == 0:
                        self.error = 0


            else:
                self.error = data_in - self.target



def getKey():
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


class BayesLoc:
    def __init__(self, P0, colourCodes, colourMap, transProbBack, transProbForward, world_map=None, measurement_model=None, c_to_num=None):

        self.colour_sub = rospy.Subscriber('camera_rgb', String, self.colour_callback)
        #self.line_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        #self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.controller = final_monkee() #taking care of line_idx and cmd_vel in here
        self.hb = HaramBayesian(wmap=world_map, ctnum=c_to_num) #input map

        self.probability = P0 ## initial state probability is equal for all states
        self.colourCodes = colourCodes
        self.colourMap = colourMap
        self.transProbBack = transProbBack
        self.transProbForward = transProbForward
        self.numStates = len(P0)
        self.statePrediction = np.zeros(np.shape(P0))

        self.CurColour = None #most recent measured colour

    def norm(self):
        measurement = []
        for color in self.colourCodes:
            temp=0
            for i, channel in enumerate(color):
                diff = (channel - self.CurColour[i])**2
                temp += diff
            measurement.append(np.sqrt(temp))
        return measurement

    def prob(self, measurement):
        norm_factor = np.sum(measurement)
        for i in range(len(measurement)):
            measurement[i] = measurement[i]/norm_factor
        return measurement



    def colour_callback(self, msg):
        '''
        callback function that receives the most recent colour measurement from the camera.
        '''
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))

        self.CurColour = np.array([r,g,b])
        #rospy.loginfo(self.CurColour)

    def waitforcolour(self):
        while(1):
            if self.CurColour is not None:
                break

    def measurement_model(self):
        if self.CurColour is None:
            self.waitforcolour()
        prob=np.zeros(len(colourCodes))
        '''
        Measurement model p(z_k | x_k = colour) - given the pixel intensity, what's the probability that
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.CurColour)
            and the reference RGB values of each color (self.ColourCodes).
        '''
        return prob

    def statePredict(self,forward):
        rospy.loginfo('predicting state')
        '''
        TODO: Complete the state prediction function
        '''

    def stateUpdate(self):
        rospy.loginfo('updating state')
        '''
        TODO: Complete the state update function
        '''

if __name__=="__main__":

    float_formatter = "{:.2f}".format
    np.set_printoptions(formatter={'float_kind':float_formatter})

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    color_maps = [3, 1 ,0, 3, 1, 2, 0, 2, 1, 0, 1]#project.launch map[3, 0, 2, 1, 1, 0, 2, 1, 3, 0, 2] ## current map starting at cell#2 and ending at cell#12
    color_codes = [[72, 255, 72], #green
                    [255, 200, 0], #orange
                    [200,200,255], #purple
                    [255, 255, 0], #yellow
                    [255,255,255]] #line

    colors =['green', 'orange', 'purple', 'yellow', 'line']
    num_to_color = {}
    color_to_num = {}
    for i, color in enumerate(colors):
        num_to_color[i] = color
        color_to_num[color] = i

    world_map = {}
    for i, ckey in enumerate(color_maps):
        world_map[i] = num_to_color[ckey]

    trans_prob_fwd = [0.1,0.9]
    trans_prob_back = [0.2,0.8]

    rospy.init_node('final_project')
    bayesian=BayesLoc([1.0/len(color_maps)]*len(color_maps), color_codes, color_maps, trans_prob_back,trans_prob_fwd, world_map=world_map, c_to_num=color_to_num )
    prob = []
    rospy.sleep(0.5)
    state_count = 0
    bayesian.controller.mvel = 0.15
    prev_state=None
    flag = True
    rospy.loginfo(world_map)
    raw_address =[12, 3, 5]
    address = [office - 2 for office in raw_address]
    office = -1
    location = [0]
    try:
        while (1):
            if len(address) <1:
                break
            key = getKey()
            if (key == '\x03'):
                rospy.loginfo('Finished!')
                rospy.loginfo(prob)
                break

            #TODO: complete this main loop by calling functions from BayesLoc, and adding your own high level and low level planning + control logic
            bayesian.controller.monkee()

            if bayesian.controller.online() is False:
                if flag is True:
                    uk = 1 if bayesian.controller.mvel > 0 else 0
                    zk = num_to_color[np.argmin(bayesian.prob(bayesian.norm()))]
                    location = bayesian.hb.step(uk,zk)
                    office = np.argmax(location)

                    for s in bayesian.hb.x_k:
                        print s,
                    rospy.loginfo(zk)
                    flag = False
            else:
                if location[office] > 0.8 and office in address:
                        rospy.loginfo("Prime delievery")
                        address.remove(office)
                        startt = time.time()
                        while time.time() - startt < 1:
                            bayesian.controller.mvel = 0
                            bayesian.controller.monkee()
                        startt = time.time()
                        bayesian.controller.bonobo_hold(1, (-0.1, 0.0, 0.0), (0.0, 0.0, 0))
                        bayesian.controller.bonobo_hold(1, (0, 0.0, 0.0), (0.0, 0.0, 0))
                        bayesian.controller.bonobo_hold(4, (0.0, 0.0, 0.0), (0.0, 0.0, 0.41))
                        bayesian.controller.bonobo_hold(2, (0.0, 0.0, 0.0), (0.0, 0.0, 0))
                        bayesian.controller.bonobo_hold(4, (0.0, 0.0, 0.0), (0.0, 0.0, -0.39))
                        bayesian.controller.mvel = 0.15
                flag = True

    except Exception as e:
        print("comm failed:{}".format(e))

    finally: #note to self: causes robot to stop at the end, i think.
        rospy.loginfo(bayesian.probability)
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        cmd_publisher.publish(twist)





