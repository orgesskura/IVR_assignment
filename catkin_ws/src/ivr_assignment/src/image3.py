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
from numpy import sin,cos
import math


 #record last detected colour(maybe implement it later)

last_green_image1 =np.array([0,0])
last_blue_image1 =np.array([0,0])
last_orange_image1 =np.array([0,0])

last_green_image2 =np.array([0,0])
last_blue_image2 =np.array([0,0])
last_orange_image2 =np.array([0,0])
last_red_image1  =[0,0]
last_red_image2  =[0,0]



class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    #receive image from second camera
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    # Synchronize subscriptions into one callback
    timesync = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
    timesync.registerCallback(self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    #initialize a publisher to send position of orage sphere to to a topic name target_pos
    self.target_pub = rospy.Publisher("target_pos",Float64MultiArray, queue_size=10)
    
    #Publisher of all joints
    self.joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    #save current angles so we can use least squares
    self.current_angles = [0,0,0,0]
    #hardcoded them so that i do not calculate ratio every time
    self.pixel2meter_image1 = 0.03955706129619173
    self.pixel2meter_image2 = 0.0391203563104617

    # record the begining time
    self.time_initial= rospy.get_time()


    self.theta3 = np.array([0])
    self.previous_theta1 = 0
    self.previous_theta2 = 0
    #hardcode yellow and blue coordinates since it does not move and this improves accuracy
    self.detect_yellow_image1 = np.array([399,532])
    self.detect_yellow_image2 = np.array([399,532])
    self.detect_blue_image1 = np.array([399,472])
    self.detect_blue_image2 = np.array([399,472])

    # joint angles in last iteration
    self.q_prev_observed = np.array([0.0,0.0,0.0,0.0])
    self.prev_pos = np.array([0.0,0.0,9.0])
     # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64') 
    # initialize error and derivative of error for trajectory tracking  
    self.error = np.array([0.0,0.0,0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
    self.last_green_image1 = [0,0]
    self.last_green_image2 = [0,0]
    self.last_red_image1 = [0,0]
    self.last_red_image2 = [0,0]

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
            if image is self.cv_image1:
                  self.last_red_image1[0] = cx
                  self.last_red_image1[1] = cy
                  return np.array([cx, cy])
            else:
                  self.last_red_image2[0] = cx
                  self.last_red_image2[1] = cy
                  return np.array([cx, cy])
      #this is in case red is blocked by green
      else:
            if image is self.cv_image1:
                  return np.array(self.last_red_image1)
            else :
                  return np.array(self.last_red_image2)
      

      

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0 :
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if self.cv_image1 is image:
                  self.last_green_image1 = [cx,cy]
                  return np.array([cx, cy])
            else:
                  self.last_green_image2 = [cx,cy]
                  return np.array([cx, cy])
      #this is in case red is blocked by green
      else:
            if self.cv_image1 is image:
                  return np.array(self.last_green_image1)
            else :
                  return np.array(self.last_green_image2)
      


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0 :
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if self.cv_image1 is image:
                  self.last_blue_image1 = np.array([cx,cy])
                  return np.array([cx, cy])
            else:
                  self.last_blue_image2 = np.array([cx,cy])
                  return np.array([cx, cy])
      #this is in case red is blocked by green
      else:
            if self.cv_image1 is image:
                  return self.last_blue_image1
            else :
                  return self.last_blue_image2
      

  
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
  def detect_pos_y(self,color_sphere):
    a = self.pixel2meter_image1
    dist = color_sphere(self.cv_image1) - self.detect_yellow_image1
    return a * dist[0]


  #detect z-coordinate given an image and the color we want to detect:take average of 2 images for z
  def detect_pos_z(self,color_sphere):
    a = self.pixel2meter_image1
    b = self.pixel2meter_image2
    dist1 = -color_sphere(self.cv_image1) + self.detect_yellow_image1
    dist2 = -color_sphere(self.cv_image2) + self.detect_yellow_image2
    #return a * dist1[1]
    return (a * dist1[1] + b* dist2[1])/2

  # detect x-coordinate given an image and the color we want to detect
  def detect_pos_x(self,color_sphere):
    a = self.pixel2meter_image2
    dist = color_sphere(self.cv_image2) - self.detect_yellow_image2
    return a * dist[0]
  
  #get xyz coordinates for a colour sphere
  def get_coordinates(self,color_sphere):
        x_cord = self.detect_pos_x(color_sphere)
        y_cord = self.detect_pos_y(color_sphere)
        z_cord = self.detect_pos_z(color_sphere)
        return np.array([x_cord,y_cord,z_cord])

  def length_of_vector(self,vect):
        squared_sum = np.sum(vect ** 2)
        return np.sqrt(squared_sum)


  def vector_angles(self,vect1,vect2):
        dot_vect1_vect2 = np.dot(vect1,vect2)
        length_vect1 = self.length_of_vector(vect1)
        length_vect2 = self.length_of_vector(vect2)
        return np.arccos(dot_vect1_vect2/(length_vect1 * length_vect2))


  def rotate_axis_z(self,theta,vec):
        rotMat = np.array([[cos(theta),-sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])
        return np.dot(rotMat,vec)

  def rotate_axis_x(self,theta,vec):
        rotMat =np.array([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]])
        return np.dot(rotMat,vec)

  def projection(self,vect1, vect2):
    dot_prod = np.dot(vect1,vect2)
    sqrt_length = self.length_of_vector(vect2) * self.length_of_vector(vect2)
    amount_proj = dot_prod/sqrt_length
    return amount_proj * vect2

  # calculate angles in naive way
  def calc_angles(self):
    link3 = self.get_coordinates(self.detect_green) - [0,0,2.5] 
    j1 = np.arctan2(link3[1],link3[0])
    #rotation around x
    j2 = -(np.arctan2(link3[2], link3[1]) - np.pi/2)  
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

    link3 = self.rotate_axis_x(-j2, link3)
    #Rotation around y
    j3 = np.arctan2(link3[2], link3[0]) - np.pi/2 
    if j3 > np.pi :
          j3 = j3 - 2 *np.pi
    elif j3 < np.pi:
          j3 = j3 + 2 *np.pi

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
    link4 = self.get_coordinates(self.detect_red) - self.get_coordinates(self.detect_green) 
    proj= self.projection(link4, link3)
    if (self.length_of_vector(link4 + proj) >= self.length_of_vector(link4)):
       j4 = self.vector_angles(link4, proj)
    else :
       j4 = np.pi /2 - self.vector_angles(link4, proj)
    
    return np.array([j1,j2, j3, j4])
      
  def length_of_vector(self,vect):
        squared_sum = np.sum(vect ** 2)
        return np.sqrt(squared_sum)


  def vector_angles(self,vect1,vect2):
        dot_vect1_vect2 = np.dot(vect1,vect2)
        length_vect1 = self.length_of_vector(vect1)
        length_vect2 = self.length_of_vector(vect2)
        return np.arccos(dot_vect1_vect2/(length_vect1 * length_vect2))

  

        


  

  # Recieve data from camera 1 and camera 2, process them, and use them
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
    joint1_val = Float64()
    joint1_val.data = (np.pi) * np.sin((np.pi/15) * (rospy.get_time() - self.time_initial)) 
    joint2_val = Float64()
    joint2_val.data = (np.pi/2) * np.sin((np.pi/15) * (rospy.get_time() - self.time_initial)) 
    joint3_val = Float64()
    joint3_val.data = (np.pi/2) * np.sin((np.pi/18) * (rospy.get_time()-self.time_initial))
    joint4_val = Float64()
    joint4_val.data = (np.pi/3) * np.sin((np.pi/20) * (rospy.get_time()-self.time_initial))

      #     print(self.pixel2meter(self.cv_image2))
      #     im1=cv2.imshow('window1', self.cv_image1)
      #     cv2.waitKey(1)
      #     print(self.get_coordinates(self.detect_green))
    self.joints = Float64MultiArray()#The published joints detected by the vision are placed in array
    self.joints.data = self.calc_angles()

      #Publish the results
    try: 
            # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.joints_pub.publish(self.joints)
            self.joint1_pub.publish(joint1_val)
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


