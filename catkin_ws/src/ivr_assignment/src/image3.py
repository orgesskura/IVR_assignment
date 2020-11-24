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
    
    #Publisher each of the 3 joints since joint 1 is fixed
    self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    #save current angles so we can use least squares
    self.current_angles = np.array([1.0,1.0,1.0])
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
                  self.last_green_image1 = np.array([cx,cy])
                  return np.array([cx, cy])
            else:
                  self.last_green_image2 = np.array([cx,cy])
                  return np.array([cx, cy])
      #this is in case red is blocked by green
      else:
            if self.cv_image1 is image:
                  return self.last_green_image1
            else :
                  return self.last_green_image2
      


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
  
  #calculate translation matrices by using rotations and translations in x or y
  # i calculate 1 by 1 due to np array properties
  def calcTrans12(self,thetas):
        return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,2.5],[0,0,0,1]])

  def calcTrans23(self,thetas):
        return np.array([[1,0,0,0],
       [0,np.cos(thetas[0]),-np.sin(thetas[0]),0],
       [0,np.sin(thetas[0]),np.cos(thetas[0]),0],
       [0,0,0,1]])

  def calcTrans34(self,thetas):
        return np.array([[np.cos(thetas[1]),0,np.sin(thetas[1]),3.5*np.sin(thetas[1])],
       [0,1,0,0],
       [-np.sin(thetas[1]),0,np.cos(thetas[1]),3.5* np.cos(thetas[1])],
       [0,0,0,1]])

  def calcTrans45(self,thetas):
        return np.array([[1,0,0,0],
       [0,np.cos(thetas[2]),-np.sin(thetas[2]),-3 * np.sin(thetas[2])],
       [0,np.sin(thetas[2]),np.cos(thetas[2]),3*np.cos(thetas[2])],
       [0,0,0,1]])

  def calcTrans13(self,thetas):
        return np.matmul(self.calcTrans12(thetas),self.calcTrans23(thetas))

  def calcTrans14(self,thetas):
        return np.matmul(self.calcTrans13(thetas), self.calcTrans34(thetas))

  def calcTrans15(self,thetas):
        return np.matmul(self.calcTrans14(thetas), self.calcTrans45(thetas))


  def optimize_func(self,thetas):
       #calculate positon taken from translation matrices vs ones taken by computer vision so that we pass it as argument to least squares
       #calculate till green as well since we need enough equations to come up with solutions
       calc_gx = self.calcTrans14(thetas)[0,3]
       calc_gy = self.calcTrans14(thetas)[1,3]
       calc_gz = self.calcTrans14(thetas)[2,3]
       real_gx = self.get_coordinates(self.detect_green)[0]
       real_gy = self.get_coordinates(self.detect_green)[1]
       real_gz = self.get_coordinates(self.detect_green)[2]
       calc_rx = self.calcTrans15(thetas)[0,3]
       calc_ry = self.calcTrans15(thetas)[1,3]
       calc_rz = self.calcTrans15(thetas)[2,3]
       real_rx = self.get_coordinates(self.detect_red)[0]
       real_ry = self.get_coordinates(self.detect_red)[1]
       real_rz = self.get_coordinates(self.detect_red)[2]
       calc_bx = self.calcTrans13(thetas)[0,3]
       calc_by = self.calcTrans13(thetas)[1,3]
       calc_bz = self.calcTrans13(thetas)[2,3]
       #get error from calculated position and one calculated with computer vision
       return [calc_gx - real_gx,calc_gy - real_gy,calc_gz - real_gz,calc_rx - real_rx,calc_ry - real_ry, calc_rz - real_rz,calc_bx,calc_by,calc_bz - 2.5]
      # real_green = self.get_coordinates(self.detect_green)
      # real_red = self.get_coordinates(self.detect_red)
      # return sum[np.sum(calc_green-real_green),np.sum(calc_red-real_red)]

  def functions(self,thetas):
        
        f = [0,0,0,0,0,0]
        f[0] = self.calcTrans14(thetas)[0,3] - self.get_coordinates(self.detect_green)[0]
        f[1] = self.calcTrans14(thetas)[1,3] - self.get_coordinates(self.detect_green)[1]
        f[2] = self.calcTrans14(thetas)[2,3] - self.get_coordinates(self.detect_green)[2]
        f[3] = self.calcTrans15(thetas)[0,3] - self.get_coordinates(self.detect_red)[0]
        f[4] = self.calcTrans15(thetas)[1,3] - self.get_coordinates(self.detect_red)[1]
        f[5] = self.calcTrans15(thetas)[2,3] - self.get_coordinates(self.detect_red)[2]

        return f
  def calc_trans_physics(self,thetas):
        b = np.array([[3.5*sin(thetas[1])*cos(thetas[0])],
        [-3.5*sin(thetas[0])],
        [3.5*cos(thetas[0]) * cos(thetas[1])+2.5]])
        c = np.array([[6.5*sin(thetas[1])*cos(thetas[0])],
        [-3*sin(thetas[0])*cos(thetas[2]) -3.5 * sin(thetas[0]) -  3 * sin(thetas[2])* cos(thetas[0]) * cos(thetas[1])],
        [-3 * sin(thetas[0]) * sin(thetas[2])  + 3 * cos(thetas[0]) * cos(thetas[1]) * cos(thetas[2]) + 3.5 * cos(thetas[0]) * cos(thetas[1]) + 2.5 ]])
        return b,c


  def calc_theta1_theta2(self):
         green = self.get_coordinates(self.detect_green)
         if green[1] > 0 :
               green[1] = green[1] - 0.5
         elif green[1] < 0 :
             green[1] = green[1] + 0.5
         theta1 = np.arcsin((-1/3.5) * green[1])
         cos_theta1 = cos(theta1)
      #    if green[0]/(3.5*cos(theta1)) >= 1:
      #          theta2 = np.pi/2
      #    elif green[0]/(3.5*cos(theta1)) <= -1:
      #        theta2 = self.previous_theta2 + 0.1
      #    else:
      #       theta2 = asin(green[0]/(3.5*cos(theta1)))
      #       self.previous_theta2 = theta2
         theta2 =0

         
         return theta1,theta2

  def functions_direct(self,thetas3):
         thetas1,thetas2 = self.calc_theta1_theta2()
         red = self.get_coordinates(self.detect_red)
         f = [0,0]
         f[0] = -3*sin(thetas1)*cos(thetas3[0]) -3.5 * sin(thetas1) -  3 * sin(thetas3[0])* cos(thetas1) * cos(thetas2) - red[1]
         f[1] = -3 * sin(thetas1) * sin(thetas3[0])  + 3 * cos(thetas1) * cos(thetas3[0]) * cos(thetas2) + 3.5 * cos(thetas1) * cos(thetas2) + 2.5 - red[2]
         return f
  def calc_angles_manually(self):
        bounds = (-np.pi/3,np.pi/3)
        res = least_squares(self.functions_direct,self.theta3,bounds = bounds)
        self.theta3 = res.x
        return res.x

  def functions_physics(self,thetas):
        b,c = self.calc_trans_physics(thetas)
        f = [0,0,0,0,0,0]
        f[0] = b[0,0] - self.get_coordinates(self.detect_green)[0]
        f[1] = b[1,0] - self.get_coordinates(self.detect_green)[1]
        f[2] = b[2,0] - self.get_coordinates(self.detect_green)[2]
        f[3] = c[0,0] - self.get_coordinates(self.detect_red)[0]
        f[4] = c[1,0] - self.get_coordinates(self.detect_red)[1]
        f[5] = c[2,0] - self.get_coordinates(self.detect_red)[2]
        
        return f
  #calculate angles using least squares method
  def calc_angles_leastSquares(self):
        bounds = (
        np.array([-np.pi/2, -np.pi/2, -np.pi/3]),
        np.array([ np.pi/2, np.pi/2, np.pi/3]),
                )
        res = least_squares(self.functions_physics, self.current_angles,
        bounds =bounds)
        self.current_angles = res.x
        print(res.cost)
        return res

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
#     rs = self.calc_angles_manually()
#     theta3 = rs
#     theta1,theta2 = self.calc_theta1_theta2()
#     self.joints.data = [theta1,theta2,0]
#     print(rs.cost)
#     self.joints.data = self.calc_angles_naive()
#     print(self.pixel2meter(self.cv_image1))
#     print(self.pixel2meter(self.cv_image2))
    print(last_red_image1)
    print(last_red_image2)
#     im1=cv2.imshow('window1', self.cv_image1)
#     cv2.waitKey(1)
#     print(self.get_coordinates(self.detect_green))
    #Publish the results
    try: 
      # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      # self.joints_pub.publish(self.joints)
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


