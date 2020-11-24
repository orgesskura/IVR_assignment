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


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    # Synchronize subscriptions into one callback
    timesync = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
    timesync.registerCallback(self.callback2)
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
    #hardcoded them so that i do not calculate ratio every time
    self.pixel2meter_image1_blue = 0.041666666666666664
    self.pixel2meter_image2_blue = 0.041643537790798095
    self.pixel2meter_image1_green = 0.039473684210526314
    self.pixel2meter_image2_green = 0.03947282997743106
    self.pixel2meter_image1_red = 0.039301310043668124
    self.pixel2meter_image2_red = 0.03913006493497153
    self.pixel2meter_es_image1 = 0.03955706129619173
    self.pixel2meter_es_image2 = 0.0391203563104617


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
      #in case blue is blocked by blue
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


  #detect y-coordinate given the image and the color we want to detect
  def detect_pos_y(self,color_sphere,image):
    a = self.pixel2meter_es_image2
    dist = color_sphere(image) - self.detect_yellow(image)
    return a * dist[0]


  #detect z-coordinate given an image and the color we want to detect:take average of 2 images for z
  def detect_pos_z(self,color_sphere,image1,image2):
    a = self.pixel2meter_es_image1
    b = self.pixel2meter_es_image2
    dist1 = -color_sphere(image1) + self.detect_yellow(image1)
    dist2 = -color_sphere(image2) + self.detect_yellow(image2)
    return (a * dist1[1] + b* dist2[1])/2
   # return a* dist1[1]

  # detect x-coordinate given an image and the color we want to detect
  def detect_pos_x(self,color_sphere,image):
    a = self.pixel2meter_es_image1
    dist = color_sphere(image) - self.detect_yellow(image)
    return a * dist[0]
  
  #get xyz coordinates for a colour sphere
  def get_coordinates(self,color_sphere,image1,image2):
        x_cord = self.detect_pos_x(color_sphere,self.cv_image2)
        y_cord = self.detect_pos_y(color_sphere,self.cv_image1)
        z_cord = self.detect_pos_z(color_sphere,self.cv_image1,self.cv_image2)
        return np.array([x_cord,y_cord,z_cord])



  def pixel2meter_blue(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_yellow(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 2.5 / np.sqrt(dist)
  def pixel2meter_green(self,image):
          # Obtain the centre of each coloured blob
      circle1Pos = self.detect_green(image)
      circle2Pos = self.detect_blue(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3.5 / np.sqrt(dist)
  def pixel2meter_green2red(self,image):
      circle1Pos = self.detect_red(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)

  def pixel2meter_red(self,image):
          # Obtain the centre of each coloured blob
      circle1Pos = self.detect_red(image)
      circle2Pos = self.detect_yellow(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 9 / np.sqrt(dist)
  def pixel2meter_estimated(self,image):
        est1 = (self.pixel2meter_blue(image) + self.pixel2meter_green(image) + self.pixel2meter_green2red(image))/3
        return est1


   # calculate angles in naive way
  def calc_angles_naive(self):
        a = self.pixel2meter_es_image1
        b = self.pixel2meter_es_image2
        yellow1 = a*self.detect_yellow(self.cv_image1)
        yellow2 = b*self.detect_yellow(self.cv_image2)
        blue1 = a * self.detect_blue(self.cv_image1)
        blue2 = b * self.detect_blue(self.cv_image2)
        green1 = a*(self.detect_green(self.cv_image1))
        green2 = b*(self.detect_green(self.cv_image2))
        red1 = a* self.detect_red(self.cv_image1)
        red2 = b * self.detect_red(self.cv_image2)
       #calculate rotation around x-axis using camera 1

        j2=np.arctan2(blue1[0]-green1[0],blue1[1]-green1[1])
        #do this since arctan2 return angles in [-pi,pi] range
        if j2 > (np.pi)/2 :
              a = np.pi - j2
              b = (np.pi)/2 - a
              j2 = (np.pi)/2 - b
        elif j2 < -(np.pi)/2 :
              a = np.pi + j2
              b = (np.pi)/2 - a
              j2 = -(np.pi)/2 + b
        else :
              j2 = j2
        #calculate rotation around y-axis using camera 2
        j3 = -(np.arctan2(blue2[0]-green2[0],blue2[1]-green2[1]) )
        if j3 > (np.pi)/2 :
              a = np.pi - j3
              b = (np.pi)/2 - a
              j3 = (np.pi)/2 - b
        elif j3 < -(np.pi)/2 :
              a = np.pi + j3
              b = (np.pi)/2 - a
              j3 = -(np.pi)/2 + b
        else :
              j3 = j3

        #calculate rotation around x-axis using camera 1 for last angles since rotation around y in camera 1 does not change angles
        j4 = np.arctan2(green1[0]-red1[0],green1[1]-red1[1])
        if j4 > (np.pi)/2 :
              a = np.pi - j4
              b = (np.pi)/2 - a
              j4 = (np.pi)/2 - b
        elif j4 < -(np.pi)/2 :
              a = np.pi + j4
              b = (np.pi)/2 - a
              j4 = -(np.pi)/2 + b
        else :
              j4 = j4
        j4 = j4 - j2

        return [j2,j3,j4]


  # Recieve data, process it, and publish
  def callback2(self,data1,data2):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")

    except CvBridgeError as e:
      print(e)
    
    joint2_val = Float64()
    joint2_val.data = (np.pi/2) * np.sin((np.pi/15) * (rospy.get_time() - self.time_initial)) 
    joint3_val = Float64()
    joint3_val.data = (np.pi/2) * np.sin((np.pi/18) * (rospy.get_time()-self.time_initial))
    joint4_val = Float64()    # im2=cv2.imshow('window2', self.cv_image2)
    # cv2.waitKey(1)
    joint4_val.data = (np.pi/3) * np.sin((np.pi/20) * (rospy.get_time()-self.time_initial))
    
    self.joints = Float64MultiArray()
#     rs = self.calc_angles_manually()
#     theta3 = rs
    # theta1,theta2 = self.calc_theta1_theta2()
    # self.joints.data = [theta1,theta2,0]
    self.joints.data = self.calc_angles_naive()

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    # im2=cv2.imshow('window2', self.cv_image2)
    # cv2.waitKey(1)
    # print(self.get_coordinates(self.detect_blue,self.cv_image1,self.cv_image2))
    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.joints_pub.publish(self.joints)
      self.joint2_pub.publish(joint2_val)
      self.joint3_pub.publish(joint3_val)
      self.joint4_pub.publish(joint4_val)
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


