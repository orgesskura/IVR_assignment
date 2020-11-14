#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from scipy.optimize import least_squares

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    #receive image from second camera
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    # Synchronize subscriptions into one callback
    ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 1)
    ts.registerCallback(self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    
    #Publisher each of the 3 joints since joint 1 is fixed
    self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    #save current angles so we can use least squares
    self.current_angles = [0,0,0]


    # record the begining time
    self.time_initial= rospy.get_time()

  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)

      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0 :
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
      #this is in case red is blocked by green
      else:
            cx = self.detect_green(image)[0]
            cy = self.detect_green(image)[1]
      return np.array([cx, cy])

      

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if(M['m00'] != 0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
      #in case blue is blocked by green
      else:
            cx = self.detect_blue(image)[0]
            cy = self.detect_blue(image)[1]
      return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if(M['m00'] != 0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
      #in case blue is blocked by green
      else:
            cx = self.detect_green(image)[0]
            cy = self.detect_green(image)[1]
            
      return np.array([cx, cy])

  
  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def pixel2meter(self,image):
    # Obtain the centre of each coloured blob,in particular yellow and blue
     circle1Pos = self.detect_yellow(image)
     circle2Pos = self.detect_blue(image)
      # find the distance between two circles
     dist = np.sum((circle1Pos - circle2Pos)**2)
     return 2.5 / np.sqrt(dist)

  #detect y-coordinate given the image and the color we want to detect
  def detect_pos_y(self,image,color_sphere):
    a = self.pixel2meter(image)
    dist = color_sphere(image) - self.detect_yellow()
    return a * dist[0]


  #detect y-coordinate given an image and the color we want to detect
  def detect_pos_z(self,image,color_sphere):
    a = self.pixel2meter(image)
    dist = color_sphere(image) - self.detect_yellow()
    return a * dist[1]

  # detect x-coordinate given an image and the color we want to detect
  def detect_pos_x(self,image,color_sphere):
    a = self.pixel2meter(image)
    dist = color_sphere(image) - self.detect_yellow()
    return a * dist[0]
  
  #get xyz coordinates for a colour sphere
  def get_coordinates(self,color_sphere):
        x_cord = self.detect_pos_x(self.cv_image2,color_sphere)
        y_cord = self.detect_pos_y(self.cv_image1,color_sphere)
        z_cord = self.detect_pos_z(self.cv_image1,color_sphere)
        return [x_cord,y_cord,z_cord]
  
  #calculate translation matrices by using rotations and translations in x or y
  def calcTrans(self,thetas):
        #calculate translation matrices
       trans23 = np.array([[1,0,0,0],
       [0,np.cos(thetas[0]),-np.sin(thetas(0)),0],
       [0,np.sin(thetas[0]),np.cos(thetas[0]),0],
       [0,0,0,1]])
       trans12 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,2.5],[0,0,0,1]])
       trans34 = np.array([[np.cos(thetas[1]),0,np.sin(thetas[1]),0],
       [0,1,0,0],
       [-np.sin(thetas[1]),0,np.cos(thetas[1]),3.5],
       [0,0,0,1]])
       trans45 = np.array([[1,0,0,0],
       [0,np.cos(thetas[2]),-np.sin(thetas[2]),0],
       [0,np.sin(thetas[2]),np.cos(thetas[2],3)],
       [0,0,0,1]])
       #calculate translation matrices from one frame to another
       trans13 = np.matmul(trans12,trans23)
       trans14 = np.matmul(trans13,trans34)
       trans15 = np.matmul(trans15,trans45)
       return trans14,trans15

  def optimize_func(self,thetas):
       #calculate positon taken from translation matrices vs ones taken by computer vision so that we pass it as argument to least squares
       trans14,trans15 = self.calcTrans(thetas)
       calc_gx = trans14[0,3]
       calc_gy = trans14[1,3]
       calc_gz = trans14[2,3]
       real_gx = self.get_coordinates(self.detect_green)[0]
       real_gy = self.get_coordinates(self.detect_green)[1]
       real_gz = self.get_coordinates(self.detect_green)[2]
       calc_rx = trans15[0,3]
       calc_ry = trans15[1,3]
       calc_rz = trans15[2,3]
       real_rx = self.get_coordinates(self.detect_red)[0]
       real_ry = self.get_coordinates(self.detect_red)[1]
       real_rz = self.get_coordinates(self.detect_red)[2]
       #get error from calculated position and one calculated with computer vision
       return [calc_gx-real_gx,calc_gy-real_gy,calc_gz-real_gz,calc_rx-real_rx,calc_ry - real_ry,calc_rz - real_rz]



  #calculate angles using least squares method
  def calc_angles(self):
        res = least_squares(self.calcTrans, self.current_angles,
        bounds=([-np.pi / 2, -np.pi / 2, -np.pi / 2], [np.pi / 2, np.pi / 2, np.pi / 2]))
        self.current_angles = res.x
        return res.x



        


  

  # Recieve data from camera 1, process it, and use it
  def callback1(self,data1,data2):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    #Set the joint angles  according to the assignment requirements
    joint2_val = Float64()
    joint2_val.data = (np.pi/2) * np.sin((np.pi/15) * (rospy.get_time() - self.time_initial)) 
    joint3_val = Float64()
    joint3_val.data = (np.pi/2) * np.sin((np.pi/18) * (rospy.get_time()-self.time_initial))
    joint4_val = Float64()
    joint4_val.data = (np.pi/2) * np.sin((np.pi/20) * (rospy.get_time()-self.time_initial))

    self.joints = Float64MultiArray()
    x = self.calc_angles()
    self.joints.data = x

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.joint2_pub.publish(joint2_val)
      self.joint3_pub.publish(joint3_val)
      self.joint4_pub.publish(joint4_val)
      self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)
    


# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


