#!/usr/bin/env python
'''
Modified July 2016
@author: Ike Dean

  Edison based differention drive base
  Borrowed heavily from Dr. Rainer Hessmer's ardros/arduino.py code.
  
Created January, 2011

@author: Dr. Rainer Hessmer

  arduino.py - gateway to Arduino based differential drive base
  Copyright (c) 2011 Dr. Rainer Hessmer.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import roslib
import rospy
#import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from SerialDataGateway import SerialDataGateway

class Edison(object):
	
	def __init__(self, port="/dev/ttyMFD1", baudRate=9600):
		self._Counter = 0
		self._PreviousLeftEncTicks = 0
		self._PreviousRighEncTicks = 0

		rospy.init_node('edison')
    
		rospy.loginfo("ed._init_: Starting with serial port: " + port + ", baud rate: " + str(baudRate))

		# subscriptions and publications
		rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
		
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

	def Start(self):
		rospy.loginfo("ed.Start: Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.loginfo("ed.Stop: Stopping")
		self._SerialDataGateway.Stop()
		
	def _HandleVelocityCommand(self, twistCommand):
		""" Handle movement requests. """
		v = twistCommand.linear.x        # m/s
		omega = twistCommand.angular.z   # rad/s
		rospy.loginfo("ed.hvc: Input ( /cmd_vel msg from ROS) : " + str(v) + "," + str(omega))

		message = 's %.2f %.2f\r' % (v, omega)
		rospy.loginfo("ed.hvc: Output > Msg to Kanagroo : " + message)
		'''self._WriteSerial(message)'''

	def _HandleReceivedLine(self,  line):
		rospy.loginfo("ed.hrl: Output Ack/Err < Msgs rcvd from Kanagroo : " + line)

if __name__ == '__main__':
	edison = Edison() #runs __init__ constructor to instance  edison class object
	try:
		edison.Start()  #calls edison.Start() to start serial port listener thread
		rospy.spin()

	except rospy.ROSInterruptException:
		edison.Stop()

