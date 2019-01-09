import math
import random
import numpy as np
from matplotlib import pyplot as plt

## base functions
##-------------------------------------------
def truncate(n, decimals=0):
    multiplier = 10 ** decimals
    return int(n * multiplier) / multiplier

def is_in_range (num, _min, _max):
    if (_min <= num <= _max):
        return True
    else:
        return False

#efficiently find points inside a circle sector

class rel:
    def __init__ (self, x, y):
        self.x = x
        self.y = y

def isInsideSector(point, center, sectorStart, sectorEnd, radius):
    x = point[0] - center[0]
    y = point[1] - center[1]

    relPoint    = rel(x,y)
    sectorStart = rel(sectorStart[0],sectorStart[1])
    sectorEnd   = rel(sectorEnd[0],sectorEnd[1])
    
    radiusSquared = radius * radius
    
    if ((areClockwise(sectorStart, relPoint))== False\
       and areClockwise(sectorEnd, relPoint) and isWithinRadius(relPoint, radiusSquared)):
        return True
    else:
        return False

def areClockwise(v1, v2):
      if (-v1.x*v2.y + v1.y*v2.x > 0):
          return True
      else:
          return False

def isWithinRadius(v, radiusSquared):
    if (v.x*v.x + v.y*v.y <= radiusSquared):
        return True
    else:
        return False

def angle_bet_2vec(a,b):
    x1, y1 = a
    x2, y2 = b
    dot = x1*x2 + y1*y2      # dot product
    det = x1*y2 - y1*x2      # determinant
    angle = math.atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
    return angle

##classes
##-------------------------------------------
class _2D_square_world_:
    def __init__ (self, size):
        self.width  = size / 2
        self.height = size / 2
        self.size_int_range = [(-1)*(int(size/2)),(int(size/2))]
        self.size_int_range_without_neg = [0, int(size)]
        self.offset = int(size / 2)
        self.full_size = size
        
        left_top     = [((-1) * self.width), ((-1) * self.height)]
        right_top    = [self.width, ((-1) * self.height)]
        right_bottom = [self.width, self.height]
        left_bottom  = [((-1) * self.width), self.height]

        self.corners = [left_top, right_top, right_bottom, left_bottom]
        #x
        self.max_x_range = self.corners[2][0]
        self.min_x_range = self.corners[0][0]
        #y
        self.max_y_range = self.corners[2][1]
        self.min_y_range = self.corners[0][1]
        
        #circle
        self.center = [ 0 , 0 ]
        self.radius = size/2
        
        self.points = []
        
    def add_point (self, x, y):
        err_mess = 'Point [' + str(x) + ',' + str(y) + '] not in range'
        if (is_in_range(x, self.min_x_range, self.max_x_range)):
            if (is_in_range(y, self.min_y_range, self.max_y_range)):
                x = int(truncate(x, 1))
                y = int(truncate(y, 1))
                self.points.append([x,y])
            else:
                print err_mess
        else:
            print err_mess

SIZE_of_world = 500

rep_pattern_w = _2D_square_world_(SIZE_of_world)

##map depth map to proccess second stereo pair
import cv2
## Force DEPTH MAP to the size of array
depth_map = cv2.imread('asd.jpg',0)
depth_map = cv2.resize(depth_map, (SIZE_of_world , SIZE_of_world))

def map_range(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def fill_in_with_circular_pattern (world, amount_of_segments) :

    full_angle_of_circle = float(360)
    radius_of_one_strip = float(full_angle_of_circle) / float(amount_of_segments)
    angle = math.radians(radius_of_one_strip)
    vector_start = [world.radius, 0.0]
    vector_end   = [world.radius * math.cos(angle), world.radius * math.sin(angle)]
    #check angle between vectors
    check_calc = math.degrees(angle_bet_2vec(vector_start, vector_end))
    print check_calc
    print radius_of_one_strip
    print vector_start
    print vector_end

    #add random points within circle sector
    offset = world.offset
    for xx in range (world.size_int_range_without_neg[1]):
        for yy in range (world.size_int_range_without_neg[1]):
            #decide if we place a point or not
            current_decision = random.choice([True, False])
            if (current_decision):
                cx = xx
                cy = yy
                #sectorStart and end expressed in relative to center vectors
                if(isInsideSector([cx,cy],[0,0],vector_start,vector_end,world.radius)):
                    world.add_point(cx,cy)

    #repeat segment across circle
    new_arr = []
    for p in range (len(world.points)):            
        for re in range(amount_of_segments):
            #copy into initial place
            shift = math.radians(radius_of_one_strip*re)
            angle = shift
            X = int((math.cos(angle) * world.points[p][0]) - (math.sin(angle) * world.points[p][1]))
            Y = int((math.sin(angle) * world.points[p][0]) + (math.cos(angle) * world.points[p][1]))
            new_point_coord = [X,Y]
            new_arr.append(new_point_coord)

    #append new segments to array
    for p in new_arr:
        world.points.append(p)
    
    #shift coordinates according to depth map
    shift_arr = []
    for p in range (len(world.points)):
        int_row    = int(world.points[p][1]+offset)
        int_column = int(world.points[p][0]+offset)
        if(int_row != world.full_size and int_column != world.full_size):
            shift_val = depth_map[int_row][int_column]
            normalised_shift_val = map_range(shift_val, 0, 255, 0,  radius_of_one_strip)    
            angle = math.radians(normalised_shift_val)
            XX = int((math.cos(angle) * world.points[p][0]) - (math.sin(angle) * world.points[p][1]))
            YY = int((math.sin(angle) * world.points[p][0]) + (math.cos(angle) * world.points[p][1]))
            if(isInsideSector([world.points[p][0],world.points[p][1]],[0,0],vector_start,vector_end,world.radius)):
                shift_arr.append([world.points[p][0],world.points[p][1]])   
            else:
                for p_2 in range (len(world.points)):
                    if(world.points[p_2] == [XX,YY]):
                        shift_arr.append([XX,YY]) 

    world.points = []
    for p in shift_arr:
        world.points.append(p)
    
                                                
amount_of_segments = 6
fill_in_with_circular_pattern(rep_pattern_w, amount_of_segments)

##plot data
data = np.array(rep_pattern_w.points)
x, y = data.T
plt.scatter(x,y,marker='o',linewidth=None,s=0.5)
plt.axis('equal')
plt.axis('off')
plt.show ( )


