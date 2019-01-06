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
                x = truncate(x, 1)
                y = truncate(y, 1)
                self.points.append([x,y])
            else:
                print err_mess
        else:
            print err_mess

rep_pattern_w = _2D_square_world_(500)

def fill_in_with_circular_pattern (world, density, amount_of_segments) :

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
    for x in range (density):
        #
        cx = current_random_x_num_in_range = random.uniform(world.min_x_range, world.max_x_range)
        cy = current_random_y_num_in_range = random.uniform(world.min_y_range, world.max_y_range)
        #sectorStart and end expressed in relative to center vectors
        if(isInsideSector([cx,cy],[0,0],vector_start,vector_end,world.radius)):
            world.add_point(cx,cy)

    #repeat segment across circle
    new_arr = []
    for p in range (len(world.points)):
        point_radius = math.sqrt(world.points[p][0]*world.points[p][0]+world.points[p][1]*world.points[p][1])             
        for re in range(amount_of_segments):
            shift = math.radians(radius_of_one_strip*re)
            angle = shift
            X = (math.cos(angle) * world.points[p][0]) - (math.sin(angle) * world.points[p][1])
            Y = (math.sin(angle) * world.points[p][0]) + (math.cos(angle) * world.points[p][1])
            new_point_coord = [X,Y]
            new_arr.append(new_point_coord)

    #append new segments to array
    for p in new_arr:
        world.points.append(p)
    
fill_in_with_circular_pattern(rep_pattern_w, 500, 4)

##print rep_pattern_w.points


#add artificial corners to keep size of plot
rep_pattern_w.add_point(rep_pattern_w.corners[0][0],rep_pattern_w.corners[0][1])
rep_pattern_w.add_point(rep_pattern_w.corners[2][0],rep_pattern_w.corners[2][1])

##plot data
data = np.array(rep_pattern_w.points)
x, y = data.T
plt.scatter(x,y)
plt.axis('equal')
plt.show ( )


