#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_ardrone')

import sys
import rospy
import cv
import cv2
import math

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import tracker

class ardroneTracker:

  def __init__(self, tracker):
    self.point_pub = rospy.Publisher("/ardrone_tracker/found_point", Point )

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
    self.image_pub = rospy.Publisher( "/ardrone_tracker/image", Image )

    self.tracker = tracker

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e
  
    numpy_image = np.asarray( cv_image )
    trackData = self.tracker.track( numpy_image )
    if trackData:
      x,y,z = trackData[0],trackData[1],trackData[2]

      point = Point( x,y,z )
      self.point_pub.publish( point )
    else:
      self.point_pub.publish( Point( 0, 0, -1 ) )

    try:
      vis = self.tracker.get_vis()
      image_message = self.bridge.cv_to_imgmsg(cv2.cv.fromarray( vis ), encoding="passthrough")
      self.image_pub.publish( image_message )
    except CvBridgeError, e:
      print e

def main():
  rospy.init_node( 'ardrone_tracker' )
#  ardroneTracker(tracker.LkTracker() )
#  ardroneTracker(tracker.DummyTracker() )
#            '/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml'

  cascade_file = rospy.get_param( '~cascadefile', 'NULL' )
  if cascade_file == 'NULL':
    print "Must set cascadefile parameter!"
    exit

  ardroneTracker( tracker.CascadeTracker( cascade_file ) )
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
#  import cProfile
#  cProfile.run('main()')
    main()
