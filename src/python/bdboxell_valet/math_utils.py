import math

class Math_Utils:
    class Pose:
        x = 0
        y = 0
        theta = 0
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta

        def add(self, pose):
            self.x = self.x+pose.x
            self.y = self.y+pose.y
            self.theta = self.theta+pose.theta

        def subtract(self, pose):
            self.add(pose.multiply(-1))
        
        def distance(self, pose):
            dist = math.sqrt(math.pow(self.x - pose.x,2)+ math.pow(self.y-pose.y,2))
            delta_theta = self.theta - pose.theta
            return (dist,delta_theta)
        
        def multiply(self, scalar):
            x = self.x*scalar
            y = self.y*scalar
            theta = self.theta*scalar
            
            return Math_Utils.Pose(x,y,theta)
        
        def rotate(self, theta):
            polar_r = math.sqrt(math.pow(self.x,2) + math.pow(self.y,2))
            polar_theta = math.atan2(self.y,self.x)
            polar_theta = polar_theta + theta
            self.x = polar_r * math.cos(polar_theta)
            self.y = polar_r * math.sin(polar_theta)
        
        def print(self):
            print("("+str(self.x)+", "+str(self.y)+", "+str(self.theta)+")")
    
        '''
            Transform
                Transforms the frame of the pose object to the frame 
                of the specified pose
        '''
        def transform(self, pose):
            polar_radius = math.sqrt(math.pow(self.x,2)+ math.pow(self.y,2))
            polar_theta = math.atan2(self.y, self.x)
            polar_theta = polar_theta + pose.theta
            self.x = pose.x + polar_radius * (math.cos(polar_theta))
            self.y = pose.y + polar_radius * (math.sin(polar_theta))
            self.theta = self.theta + pose.theta

        def copy(self):
            pose = Math_Utils.Pose(self.x, self.y, self.theta)
            return pose
        
        def to_vector(self):
            return Math_Utils.Vector(self.x, self.y)

    class Vector:
        x = 0
        y = 0
        def __init__(self, x, y):
            self.x = x
            self.y = y

        def add(self, vec):
            self.x = self.x+vec.x
            self.y = self.y+vec.y

        def subtract(self, vec):
            self.add(vec.multiply(-1))
        
        def distance(self, vec):
            dist = math.sqrt(math.pow(self.x - vec.x,2)+ math.pow(self.y-vec.y,2))
            return dist
        
        def multiply(self, scalar):
            x = self.x*scalar
            y = self.y*scalar
            return Math_Utils.Vector(x,y)
        
        def magnitude(self):
            mag = math.sqrt(math.pow(self.x,2)+ math.pow(self.y,2))
            return mag
        
        def rotate(self, theta):
            polar_r = math.sqrt(math.pow(self.x,2) + math.pow(self.y,2))
            polar_theta = math.atan2(self.y,self.x)
            polar_theta = polar_theta + theta
            self.x = polar_r * math.cos(polar_theta)
            self.y = polar_r * math.sin(polar_theta)
            
        '''
            Project
                Projects this vector onto a given vector
        '''
        def project(self, vec):
            numerator = self.dot_product(vec)
            denominator = math.pow(vec.magnitude(),2)
            result = vec.multiply(numerator/denominator)
            return result
        
        def dot_product(self, vec):
            return self.x*vec.x + self.y*vec.y
        
        def cross_product(self, vec):
            return self.x*vec.y-self.y*vec.x
        
        def print(self):
            print("("+str(self.x)+", "+str(self.y)+")")

        def unit_vector(self):
            if self.magnitude() == 0:
                return self
            else:
                return self.multiply(1/self.magnitude())
        
        def equals(self, vec):
            return abs(self.x - vec.x) < 0.001 and abs(self.y - vec.y) < 0.001 
        def copy(self):
            vec = Math_Utils.Vector(self.x, self.y)
            return vec
        
        def to_tuple(self):
            return (self.x,self.y)