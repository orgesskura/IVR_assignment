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
from math import asin

 #record last detected colour position. I have declared it as global

last_green_image1 =[0,0]
last_blue_image1 =[0,0]
last_orange_image1 = [0,0]
last_red_image1  =[0,0]
last_green_image2 =[0,0]
last_blue_image2 =[0,0]
last_orange_image2 =[0,0]
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
    
    #Publisher each of the 3 joints since joint 1 is fixed
    self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    #hardcoded them so that i do not calculate ratio every time
    self.pixel2meter_image1 = 0.03955706129619173
    self.pixel2meter_image2 = 0.0391203563104617

    # record the begining time
    self.time_initial= rospy.get_time()

    #hardcode yellow and blue coordinates since blue and yellow spheres do not move and this improves accuracy
    self.detect_yellow_image1 = np.array([399,532])
    self.detect_yellow_image2 = np.array([399,532])
    self.detect_blue_image1 = np.array([399,472])
    self.detect_blue_image2 = np.array([399,472])


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
                  last_red_image1[0] = cx
                  last_red_image1[1] = cy
                  return np.array([cx, cy])
            else:
                  last_red_image2[0] = cx
                  last_red_image2[1] = cy
                  return np.array([cx, cy])
      #this is in case red is blocked by green
      else:
            if image is self.cv_image1:
                  return np.array(last_red_image1)
            else :
                  return np.array(last_red_image2)
      

      

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
                  last_green_image1[0] = cx
                  last_green_image1[1] = cy
                  return np.array([cx, cy])
            else:
                  last_green_image2[0] = cx
                  last_green_image2[1] = cy
                  return np.array([cx, cy])
      #this is in case red is blocked by green
      else:
            if self.cv_image1 is image:
                  return np.array(last_green_image1)
            else :
                  return np.array(last_green_image2)
      


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
                  last_blue_image1 = np.array[cx,cy]
                  return np.array([cx, cy])
            else:
                  self.last_blue_image2 = [cx,cy]
                  return np.array([cx, cy])
      #this is in case red is blocked by green
      else:
            if self.cv_image1 is image:
                  return np.array(last_blue_image1)
            else :
                  return np.array(last_blue_image2)
      

  
  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  def detect_orange_sphere(self,image):
    #get template in a black and white format as it is better like that for matching
    template =cv2.imread("template.png", 0) 
    #get orange thresholding
    thresh = cv2.inRange(image, (0,50,100), (15,75,150))
    #get width and height of template in order to get center of matching
    width, height = template.shape[::-1] 
    #template matching between threshold and template i have. Code is adapted from opencv python tutorials
    matching = cv2.matchTemplate(thresh, template, 1) 
    #Get result of matching the template with thresholded image
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matching) 

    #Returns the centre of the orange sphere
    return np.array([min_loc[0] + width/2, min_loc[1] + height/2]) 



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


  # calculate angles in naive way
  def calc_angles_naive(self):
        a = self.pixel2meter_image1
        b = self.pixel2meter_image2
        yellow1 = a*self.detect_yellow_image1
        yellow2 = b*self.detect_yellow_image2
        blue1 = a * self.detect_blue_image1
        blue2 = b * self.detect_blue_image2
        green1 = a*(self.detect_green(self.cv_image1))
        green2 = b*(self.detect_green(self.cv_image2))
        red = self.get_coordinates(self.detect_red)
        blue = np.array([0,0,2.5])
        green = self.get_coordinates(self.detect_green)

        green2red = red - green
        blue2green = green - blue
        
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

        #calculate rotation around joint 4 by finding angle between vectors. Could have used method for angle 2 but do not want to depend on angle 2
        j4 = self.vector_angles(blue2green, green2red)
        if (green[1]>0):
           if (red[2]>green[2]):
                j4 *=-1
        else:
           if (red[2]<green[2]):
                j4 *= -1

        return [j2,j3,-j4]
        


  

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
    joint2_val = Float64()
    joint2_val.data = (np.pi/2) * np.sin((np.pi/15) * (rospy.get_time() - self.time_initial)) 
    joint3_val = Float64()
    joint3_val.data = (np.pi/2) * np.sin((np.pi/18) * (rospy.get_time()-self.time_initial))
    joint4_val = Float64()
    joint4_val.data = (np.pi/3) * np.sin((np.pi/20) * (rospy.get_time()-self.time_initial))

    
#     self.joints = Float64MultiArray()
#     self.joints.data = self.calc_angles_naive()
#     im1=cv2.imshow('window1', self.cv_image2)
#     cv2.waitKey(1)
#     print(self.get_coordinates(self.detect_orange_sphere))

    self.target_pos = Float64MultiArray()
    self.target_pos.data = self.get_coordinates(self.detect_orange_sphere)
    #Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.target_pub.publish(self.target_pos)
      # self.joints_pub.publish(self.joints)
      # self.joint2_pub.publish(joint2_val)
      # self.joint3_pub.publish(joint3_val)
      # self.joint4_pub.publish(joint4_val)
      
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
