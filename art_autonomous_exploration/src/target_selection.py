#!/usr/bin/env python

from __future__ import division
import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from path_planning import PathPlanning


# Class for selecting the next best target
class TargetSelection:
    
    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()


    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
        resolution, force_random = False):
        
        target = [-1, -1]
        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the 
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)
        
        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker(\
            vis_nodes,\
            1, # Type: Arrow
            0, # Action: Add
            "map", # Frame
            "art_topological_nodes", # Namespace
            [0.3, 0.4, 0.7, 0.5], # Color RGBA
            0.1 # Scale
        )
        
        # Calculate the next target
        # Cost-based target selection
        
        # Initialize max and mins
        final_prior = 0
        maxdist = 0
        mindist = 999999999
        maxtopol = 0
        mintopol = 999999999
        target_index = 0
        
        distance_costs = np.zeros(len(nodes))
        topol_costs = np.zeros(len(nodes))
        rotation_costs= np.zeros(len(nodes))
        
        distance_costs_norm = np.zeros(len(nodes))
        topol_costs_norm = np.zeros(len(nodes))
        rotation_costs_norm = np.zeros(len(nodes))

        # Calculate max, mins and costs for each node
        for node_index in range(0, len(nodes)): # TODO check the range
          xtarget = nodes[node_index][0]
          ytarget = nodes[node_index][1]
          [xrobot, yrobot] = [\
              robot_pose['x_px'] - origin['x'] / resolution,\
              robot_pose['y_px'] - origin['y'] / resolution
                      ]
          distance_costs[node_index] = math.sqrt((xtarget-xrobot) \
            * (xtarget-xrobot) + (ytarget-yrobot) * (ytarget-yrobot))
          if distance_costs[node_index] > maxdist:
            maxdist = distance_costs[node_index]
          if distance_costs[node_index] < mindist:
            mindist = distance_costs[node_index]
          
          for i in range(-1,1):
            for j in range(-1,1):
              if i==0 and j==0:
                continue
              topol_costs[node_index] = topol_costs[node_index] \
                + brush[xtarget+i][ytarget+j]
          topol_costs[node_index] = (1/8) * topol_costs[node_index]
          if topol_costs[node_index] > maxtopol:
            maxtopol = topol_costs[node_index]
          if topol_costs[node_index] < mintopol:
            print node_index
            mintopol = topol_costs[node_index]
          
          rotation_costs = self.rect_to_polar_input(nodes[node_index][1]\
          -yrobot + 0.00001, nodes[node_index][0] - xrobot + 0.00001)
        
        # Normalization of costs
        distance_costs_norm = 1 - ((distance_costs - mindist)/( maxdist - mindist))
        topol_costs_norm = 1 - ((topol_costs - mintopol)/( maxtopol - mintopol))
        rotation_costs_norm = 1 - (rotation_costs/360)
        
        # Calculation of smoothing coefficient
        w_coeff = (4 * topol_costs_norm + 2 * distance_costs_norm + \
          1 * rotation_costs_norm)
        
        # Calculation of the Prioritization for each node
        current_final_prior = w_coeff * (4 * topol_costs_norm + \
          2 * distance_costs_norm + 1 * rotation_costs_norm)
        
        # Find the maximum Prioritization
        target_index = np.argmax(current_final_prior)
        final_prior = max(current_final_prior)
        target = nodes[target_index]
                  
        print "The selected target is ", target_index,  \
        " with Prioritization ", final_prior
          
        # Random point
        if self.method == 'random' or force_random == True:
          print "\n"
          print "Ra ra ra random power is on!"
          target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        ########################################################################
        return target

    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
      # The next target in pixels
        tinit = time.time()
        next_target = [0, 0] 
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
              brushogm[x_rand][y_rand] > 5:
            next_target = [x_rand, y_rand]
            found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), \
            Print.ORANGE)
        return next_target
        
    def rect_to_polar_input(self, y, x):
        angle = round(math.degrees(math.atan2(y, x)))
        if angle < 0:
            angle += 360
        return angle
