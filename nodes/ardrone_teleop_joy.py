#!/usr/bin/env python
# Copyright (c) 2012, Falkor Systems, Inc.  All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  THIS SOFTWARE IS
# PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('falkor_ardrone')

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
import pid
import time

class ArdroneTeleopJoy:

    def __init__( self ):
        self.cmd_vel_pub = rospy.Publisher( "cmd_vel_interm", Twist )
        self.land_pub = rospy.Publisher( "ardrone/land", Empty )
        self.takeoff_pub = rospy.Publisher( "ardrone/takeoff", Empty )
        self.reset_pub = rospy.Publisher( "ardrone/reset", Empty )
        self.joy_sub = rospy.Subscriber( "joy", Joy, self.callback_joy )

    def callback_joy( self, data ):
        empty_msg = Empty()

        if data.buttons[12] == 1 and self.last_buttons[12] == 0:
            self.takeoff_pub.publish( empty_msg )
            self.pause = 5.0

        if data.buttons[14] == 1 and self.last_buttons[14] == 0:
            self.land_pub.publish( empty_msg )

        if data.buttons[13] == 1 and self.last_buttons[13] == 0:
            self.reset_pub.publish( empty_msg )

        self.last_buttons = data.buttons
    
        # Do cmd_vel
        cmd = Twist()
        
        cmd.angular.x = cmd.angular.y = 0
        cmd.angular.z = data.axes[2] * 3.14/2

        cmd.linear.z = data.axes[3] * 2.0
        cmd.linear.x = data.axes[1] * 1.0
        cmd.linear.y = data.axes[0] * 1.0
        self.cmd_vel_pub.publish( cmd )


def main():
  rospy.init_node( 'ardrone_teleop_joy' )

  teleop = ArdroneTeleopJoy()

  try:
      rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main()
            
    
