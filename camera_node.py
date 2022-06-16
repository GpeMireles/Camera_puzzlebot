#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

x = (
  "nvarguscamerasrc sensor-id=%d !"
  "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
  "nvvidconv flip-method=%d ! "
  "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
  "videoconvert ! "
  "video/x-raw, format=(string)BGR ! appsink"
  % (
      0,
      1920,
      1080,
      30,
      0,
      960,
      540,
  )
)
video_capture = cv2.VideoCapture(x, cv2.CAP_GSTREAMER)



class Camera:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher('/video', Image , queue_size=10)
    self.img_msgs = None

  def show_camera(self):  
      # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
      ret, frame = video_capture.read()
      if(not ret or frame is None):
        return
      (h, w) = frame.shape[:2]
      (cX, cY) = (w // 2, h // 2)
      # rotate our img_msg by 45 degrees around the center of the img_msg
      M = cv2.getRotationMatrix2D((cX, cY), -180, 1.0)
      rotated = cv2.warpAffine(frame, M, (w, h))
      img_msg = self.bridge.cv2_to_imgmsg(rotated, "bgr8")
      self.publish(img_msg)

  def publish(self, cap):
    try:
      self.image_pub.publish(cap)
    except CvBridgeError as e:
      print(e)

if __name__ == "__main__":
  obj = Camera()
  rospy.init_node('Camera',anonymous=True)
  while not rospy.is_shutdown():
    obj.show_camera()
    print("yes")
  print("Shutdown")
  video_capture.release()
