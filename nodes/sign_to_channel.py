#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
import time

import numpy as np

from std_msgs.msg import Float32, Header, String
from phidgets_daq.msg import phidgetsDigitalOutput

from optparse import OptionParser

###############################################################################
###############################################################################
class PhidgetsAnalogOut:

    def __init__(self, magnitude_subscriber=None, sign_subscriber=None):
        ##########################################################
        ###########################################################
        
        self.sign = 0
        self.sign_t = -10000
        
        self.magnitude = 0
        self.magnitude_t = 0
        
        self.timing_difference_allowed = 1
        
        self.magnitude_subscriber = rospy.Subscriber(magnitude_subscriber, Float32, self.magnitude_callback)
        self.sign_subscriber = rospy.Subscriber(sign_subscriber, phidgetsDigitalOutput, self.sign_callback)
        
        self.publisher_0 = rospy.Publisher('chrimson/channel/0', Float32)
        self.publisher_1 = rospy.Publisher('chrimson/channel/1', Float32)
        
        nodename = 'phidgets_chrimson_sign_to_channel'
        rospy.init_node(nodename, anonymous=True)
        
    def sign_callback(self, data):
        print data
        self.sign = data.states[1]-data.states[0]
        self.sign_t = time.time()       
    
    def magnitude_callback(self, data):
        self.magnitude = data.data
        self.magnitude_t = time.time()        
        
    def main(self):
        while not rospy.is_shutdown():
            if np.abs(self.magnitude_t - self.sign_t) < self.timing_difference_allowed:
                if self.sign < 0:
                    self.publisher_0.publish(self.magnitude)
                elif self.sign > 0:
                    self.publisher_1.publish(self.magnitude)
                else:
                    self.publisher_0.publish(self.magnitude)
                    self.publisher_1.publish(self.magnitude)
                self.magnitude_t = 0
                self.sign_t = -1000
    
        
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--magnitude_subscriber", type="str", dest="magnitude_subscriber", default='',
                        help="magnitude topic to subscribe to")
    parser.add_option("--sign_subscriber", type="str", dest="sign_subscriber", default='',
                        help="sign topic to subscribe to")
    (options, args) = parser.parse_args()  
    
    
    
    analog = PhidgetsAnalogOut( magnitude_subscriber=options.magnitude_subscriber,
                                sign_subscriber=options.sign_subscriber)
    
    analog.main()
    
    
    
    # 276995
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

