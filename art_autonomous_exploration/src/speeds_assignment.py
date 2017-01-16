#!/usr/bin/env python

from __future__ import division  #added by us
import numpy as np  #added by us
import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan
      threshold_scan = 0.5
      L = len(scan)
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance
      furthest_scan = scan.index(max(scan))
      nearest_scan = scan.index(min(scan))
      # angular speed tries to avoid the nearest object and its magnitude
      # depends both on the nearest and the furthest scan
      angular = np.sign(L/2 - nearest_scan) * 0.6/(max(scan)*min(scan))
              
      # linear speed is maximum when there is no object in radius of 1m
      # otherwise the closer the object is, the lower the speed becomes 
      linear = 0.2 * min(scan) * (min(scan)< 1) + 0.3 * (min(scan)>=1)
      
      #~ print "The furthest scan is", max(scan)
      #~ print "The nearest scan is", min(scan)
      #~ print "The linear speed is", linear
      #~ print "The angular speed is", angular
      #~ print "\n"
      ##########################################################################
      
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):
 
      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()
      
      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0
      
      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.
        scan = self.laser_aggregation.laser_scan
        if min(scan) < 0.5:
          self.linear_velocity = 0.30 * l_goal + 0.70 * l_laser
          self.angular_velocity = 0.30 * a_goal + 0.70 * a_laser
          print "Avoid the obstactles is on with distance ", min(scan), "!!!\n"
        else:
          self.linear_velocity = 0.90 * l_goal + 0.10 * l_laser
          self.angular_velocity = 0.90 * a_goal + 0.10 * a_laser
        ##########################################################################
      else:
        self.linear_velocity  = l_laser
        self.angular_velocity = a_laser
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant
        pass
        ##########################################################################
        
      # maximum angular speed 0.3 r/s
      if abs(self.angular_velocity) > 0.3:
        self.angular_velocity = 0.3 * np.sign(self.angular_velocity)
        
      if abs(self.linear_velocity) > 0.3:
        self.linear_velocity = 0.3 * np.sign(self.linear_velocity)
      
      print "The linear velocity is ",self.linear_velocity
      print "The angular velocity is ",self.angular_velocity , "\n"
    
    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
      
    
