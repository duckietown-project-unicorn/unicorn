#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Marta Tintore
# Path following controller.py
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
import seaborn
import math
import bezier

from shapely.geometry import Point
from shapely.geometry import LineString

class Car(object):
    def __init__(self,x_pos,y_pos,angle,velocity):
        self.x = x_pos
        self.y = y_pos
        self.a = angle #With x axis (driving direction of the car)
        self.velx = velocity*math.cos(angle)
        self.vely = velocity*math.sin(angle)

def path_generate(direc):

        #direc: l=-1, r=1, s=0

        #Case straight
        if direc == 0:
                path = LineString([(0,-12.25),(56.5,-12.25)])

        #Case left/right
        else:
                #Create bezier curve
                #Left
                nodes_left = np.asfortranarray([[0, 18, 36, 40.5,40.5],[-12.25, -12.25,-7.85,10.25,28.25]])
                #Right
                nodes_rig = np.asfortranarray([[0, 6.75, 13.5, 16,16],[-12.25, -12.25,-14.5,-21,-28.25]])

                #Select nodes
                if direc == -1:
                        nodes = nodes_left
                if direc == 1:
                        nodes = nodes_rig

                #Create curve
                curve = bezier.Curve.from_nodes(nodes)

                #Extract points from curve
                x_points,y_points = curve.evaluate_multi(np.linspace(0,1,50))
                points = zip(x_points,y_points)
                path = LineString(points)

        return path

def orient_volt(tau,vel):

        #Motor parameters
        m_gain = 0.6
        m_trim = 0.0
        m_radius = 3.18 #Radius of the wheel (in cm)
        m_baseline = 10 #Distance between wheels (in cm)
        m_k = 27
        m_limit = 1 #Voltage

        #Adjust k by gain and trim
        k_r_inv = (m_gain + m_trim) / m_k
        k_l_inv = (m_gain - m_trim) / m_k

        omega_r = (vel + 0.5*tau*m_baseline) / m_radius
        omega_l = (vel - 0.5*tau*m_baseline) / m_radius

        #Convert from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        print(u_l)
        print(u_r)

        #Limit output to limit (u_max = 1 V)
        u_r_limited = max(min(u_r, m_limit), -m_limit)
        u_l_limited = max(min(u_l, m_limit), -m_limit)

        return u_l_limited, u_r_limited

def controller(x,y,angle,direc=-1):

        #Parameters to tune
        adm_error = 0.05 #Admissible error (perpendicular distance of future point from the path) (in cm)
        la_dis = 10 #Look ahead distance (in cm)
        t_step = 0.005 #Time interval to calculate future point (in s)

        #Set trajectory to follow
        path = path_generate(direc) #Define intersection type and desired direction to take

        #Set actual position & orientation of the car
        
        #x = 9 #Example (in cm)
        #y = -23 #Example (in cm)
        angle = angle * (math.pi/180) #Transform degrees to radians
        vel = 50 #Example (in cm/s) #TO_DO: Approximate
        car = Car(x,y,angle,vel)

        #Get future point location 
        actual = Point(car.x,car.y)

        future_x = actual.coords[0][0]+(t_step*car.velx)
        future_y = actual.coords[0][1]+(t_step*car.vely)
        future = Point(future_x, future_y)

        #Get projection of future point + distance 

        #Get projected point
        projected = path.interpolate(path.project(actual))
        #Get distance
        vector = (projected.coords[0][0]-future.coords[0][0],projected.coords[0][1]-future.coords[0][1])
        distance = np.linalg.norm(vector)

        if distance < adm_error:
                pass
        else:
                #Find goal point
                in_dis = path.project(actual)
                goal = path.interpolate(in_dis + la_dis)

                #From goal point --> vehicle action (velocity & steering vector)
                sv = (goal.coords[0][0]-actual.coords[0][0],goal.coords[0][1]-actual.coords[0][1]) #Steering_vector
                #New orientation for the car
                cosang = np.dot(sv, (1,0))
                sinang = np.linalg.norm(np.cross(sv, (1,0)))

                ori = np.arctan2(sinang, cosang) #In radians

                #Compute omega (pure pursuit geometry) 
                l = np.linalg.norm(sv) #in cm
                al = (math.pi/2) - (ori - car.a) #Complementary of current orientation and desired orientation 
                xL = l*math.sin(al) #in cm
                yL = l*math.cos(al) #in cm
                r = (xL**2)/(2*yL) + (yL/2) #in cm
                tau = vel/r

                #Output: velocity of each of the wheel motors
                return orient_volt(tau,vel)