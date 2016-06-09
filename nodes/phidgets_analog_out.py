#!/usr/bin/env python
from __future__ import division
import roslib
import rospy

from std_msgs.msg import Float32, Header, String

import Phidgets
from Phidgets.Devices.Analog import Analog
from optparse import OptionParser

###############################################################################
###############################################################################
class PhidgetsAnalogOut:

    def __init__(self, topic_subscriber=None, topic_publisher=None, channel=0, pulselength=0, pulseinterval=0, pulse=False, serial_number=-1):
        ##########################################################
        #Create an analog out object
        try:
            self.analog = Analog()
        except RuntimeError as e:
            print("Runtime Exception: %s" % e.details)
            print("Exiting....")
            exit(1)
        try:
            if serial_number != -1:
                self.analog.openPhidget(serial_number)
            else:
                self.analog.openPhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)
        rospy.sleep(1)
        ###########################################################
        
        self.channel = channel
        self.pulselength = pulselength
        self.pulseinterval = pulseinterval
        self.pulse = pulse
        self.voltage = 0
        
        self.minsleeptime = 0.05
        
        self.analog.setEnabled(self.channel, True)
        
        self.subscriber = rospy.Subscriber(topic_subscriber, Float32, self.subscriber_callback)
        self.publisher = rospy.Publisher(topic_publisher, Float32)
        
        nodename = 'phidgets_chrimson_analog_' + str(self.channel)
        rospy.init_node(nodename, anonymous=True)
        
    def subscriber_callback(self, data):
        self.voltage = data.data
        if not self.pulse:
            self.analog.setVoltage(self.channel, self.voltage)
            self.publisher.publish(Float32(self.voltage))
            rospy.sleep(self.minsleeptime)
        
    def pulse_train(self):
        if self.voltage != 0:
            self.analog.setVoltage(self.channel, self.voltage)
            self.publisher.publish(Float32(self.voltage))
            rospy.sleep(self.pulselength)
            self.publisher.publish(Float32(self.voltage))
            
            self.analog.setVoltage(self.channel, 0)
            self.publisher.publish(Float32(0))
            rospy.sleep(self.pulseinterval - self.pulselength)
            self.publisher.publish(Float32(0))
            
        else:
            self.analog.setVoltage(self.channel, 0)
            self.publisher.publish(Float32(0))
            rospy.sleep(self.minsleeptime)
    
        return
        
    def main(self):
        if self.pulse:
            while not rospy.is_shutdown():
                self.pulse_train()
        else:
            rospy.spin()
    
        
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--subscribe", type="str", dest="subscribe", default='',
                        help="topic to subscribe to")
    parser.add_option("--publish", type="str", dest="publish", default='',
                        help="topic to publish to")
    parser.add_option("--channel", type="int", dest="channel", default=0,
                        help="phidgets analog out channel (0,1,2,3)")
    parser.add_option("--pulselength", type="float", dest="pulselength", default=-1,
                        help="pulselength, seconds")
    parser.add_option("--pulseinterval", type="float", dest="pulseinterval", default=-1,
                        help="pulse period, seconds. if you want 1 second pulses with 2 seconds between each pulse, pulselength should be 1, and pulseinterval should be 3.")
    parser.add_option("--pulse", type="int", dest="pulse", default=0,
                        help="0: do not pulse (on/off determined solely by subscribe topic); 1: pulse")
    parser.add_option("--serial_number", type="int", dest="serial_number", default=-1,
                        help="serial number of phidget to connect to")
    (options, args) = parser.parse_args()  
    
    
    
    analog = PhidgetsAnalogOut( topic_subscriber=options.subscribe, 
                                topic_publisher=options.publish, 
                                channel=options.channel, 
                                pulselength=options.pulselength, 
                                pulseinterval=options.pulseinterval, 
                                pulse=options.pulse,
                                serial_number=options.serial_number)
    
    analog.main()
    
    
    
    # 276995
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
