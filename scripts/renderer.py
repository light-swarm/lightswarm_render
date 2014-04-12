#!/usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
import numpy as np
import yaml

import shape
from shapely.geometry import Polygon as ShpPolygon
from shapely.geometry import MultiPolygon as ShpMultiPolygon
from shapely.geometry import Point as ShpPoint

from lightswarm_core.msg import Penumbras
from lightswarm_core.msg import Shadow
from lightswarm_core.msg import Polygon
from lightswarm_core.msg import Point
from lightswarm_core.msg import World
from lightswarm_core.msg import Boid

from display import Display

CONFIG_FILENAME = 'src/lightswarm_core/params/config.yaml'

class Renderer:
    def __init__(self):
        self.node_name = 'renderer'
        
        self.proj_id_2idx = {'first':0, 'second':1, 'third':2, 'fourth':3}
        
        config_fname = rospy.get_param('config_file', CONFIG_FILENAME)
        fullscreen = rospy.get_param('~fullscreen', False)
        self.read_in_config(config_fname)
        self.create_homograpy()
        self.calc_resp([])
        self.create_displays2(fullscreen)

        self.sub_pen = rospy.Subscriber('/penumbras', Penumbras, self.penumbras_callback2)
        self.sub_world = rospy.Subscriber('/world', World, self.world_callback2, queue_size = 1)
 
 
    def read_in_config(self, filename):
        f = open(filename)
        config_map = yaml.safe_load(f)
        f.close()
        
        self.num_proj = config_map.get('number_of_projectors')
        self.proj_coord = config_map.get('projector_coordinates')
        self.proj_res = config_map.get('projector_resolution')
        
        
    def create_homograpy(self): #creates homography transformations between coordinates and projector pixels
        proj_pixels = np.float32([[0, 0], 
            [0, self.proj_res[0]], 
            [self.proj_res[1], self.proj_res[0]], 
            [self.proj_res[1], 0]])
        
        self.homog = []
        for i in range(self.num_proj): #one for each projector
            temp_homog, mask = cv2.findHomography(np.float32(self.proj_coord[i]), proj_pixels, 0)
            self.homog.append(temp_homog)


    def create_displays2(self, fullscreen): #create display objects for each projector image
        self.reset_image()
        self.shadows = []
        
        self.display = Display()
        self.display.setup(fullscreen)
        self.draw()
        
        
    def reset_image(self): #reset internal image for projectors
        self.image = []
        for i in range(self.num_proj):
            self.image.append(np.zeros([self.proj_res[1], self.proj_res[0], 3], np.uint8))

        
    def penumbras_callback2(self, penumbras): #update images with new responsibilities for projectors
        rospy.loginfo('got new shadows')
        self.calc_resp(penumbras.projector_shadows)
        self.update_image2()

    
    
    def calc_resp(self, projector_shadows):
    
        shadow_union = [ShpPolygon()]*self.num_proj
        for shadow in projector_shadows:
            proj_idx = self.proj_id_2idx[shadow.projector_id]
            
            poly = []
            for point in shadow.polygon.points:
                poly.append([point.y, point.x])
                
            shadow_union[proj_idx] = shadow_union[proj_idx].union(ShpPolygon(poly))
            
            
        new_resp = [None]*self.num_proj
        prev_coverage = ShpPolygon()
        
        for i in range(self.num_proj):
            curr_poly = ShpPolygon(self.proj_coord[i])
            curr_poly = curr_poly.difference(shadow_union[i])
            new_coverage = curr_poly.difference(prev_coverage)
            new_resp[i] = new_coverage
            prev_coverage.union(curr_poly)
            
        self.resp = new_resp
   
        
    def world_callback2(self, world): #update images with new boid locations
        rospy.loginfo('got world')
        self.world = world
        self.update_image2()
        
        
    def create_boid_poly(self, loc, theta):
        theta *= np.pi/180
        y = np.cos(theta)
        x = np.sin(theta)
        
        poly = np.array([ [loc[1]+y, loc[0]+x], 
            [loc[1]-y+0.5*x, loc[0]-x-0.5*y],
            [loc[1]-y-0.5*x, loc[0]-x+0.5*y]], np.float32)
        return poly
        
        
    def update_image2(self): #updates images with boid locations and responsibility vectors
        self.reset_image()
        
        for boid in self.world.boids:
            poly = self.create_boid_poly([boid.location.y, boid.location.x], boid.theta)#create boid polygon in world coord
            
            #figure out which projector owns it
            proj_own = -1
            for i in range(self.num_proj):
                if self.resp[i].contains(ShpPoint(boid.location.y, boid.location.x)):
                    proj_own = i
                    break
                    
            #transform to proj coordinates
            if (proj_own >= 0):
                poly = np.float32([ poly ]).reshape(-1,1,2)
                boid_pix = cv2.perspectiveTransform(poly, self.homog[proj_own])            
                boid_pix = np.int32(boid_pix).reshape(1,-1,2)
                cv2.fillConvexPoly(self.image[proj_own], np.fliplr(boid_pix[0]), boid.color)

        self.draw()

    def draw(self):
        combined_image = self.combine_image(self.image[0], self.image[1])
        self.display.draw(combined_image)
            
    def combine_image(self, image1, image2):
        return np.hstack((image1, image2))


def main():
    renderer = Renderer()
    
    rospy.init_node(renderer.node_name, anonymous=True)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
    
    
    
    
    
