#!/usr/bin/env python
# coding: utf-8

# In[13]:


class Vec3:
    def __init__(self, x, y, z):
        self._x = x
        self._y = y
        self._z = z

    def __iter__(self):
        for item in (self._x, self._y, self._z):
            yield item

    def __neg__(self):
        return Vec3(-self._x, -self._y, -self._z)

    def __repr__(self):
        return "Vec3({}, {}, {})".format(self._x, self._y, self._z)

    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, rhs):
        self._x = rhs

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, rhs):
        self._y = rhs

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, rhs):
        self._z = rhs   

class Convex_Hull:
    """
    A Convex Hull is defined by a list of points forming an arbitrary convex shape.
    Takes as input a list of 2D points (Each defined as a list with 2 elements)
    TODO: Include a check to confirm whether the shape is convex.
        -For now, it is assumed the list of points forms a convex shape.
    
    """
    
    def __init__(self, list2D):

        self.points = list2_to_vec3(list2D)
        
        
    def list2_to_vec3(list2D):
        #Transform list of 2D points (Each defined as a list with 2 elements)
        vec3List = []

        for point in list2D:
            vec3List.append(Vec3(point[0],point[1],0.0))

        return vec3List  


def dot(a, b):
    """Calculate the magnitude of one vector multiplied by another."""
    return a.x * b.x + a.y * b.y + a.z * b.z


def cross(a, b):
    """Calculate a vector that is at right angles to two points passed in."""
    return Vec3(
        a.y * b.z - a.z * b.y,  # i
        a.z * b.x - a.x * b.z,  # j
        a.x * b.y - a.y * b.x,  # k
    )


def support(shape, direction):
    """
    Support(shape, d), which returns the point on shape which has the highest dot product with d.
    - This would be the most 'extreme' or furthest point going in the direction passed in.
    :param shape:
    :param direction:
    :return: furthest point in the direction passed in.
    """
    furthest_in_direction = None
    result = float("-inf")
    for point in shape.points:
        product = dot(point, direction)
        if product > result:
            result = product
            furthest_in_direction = point

    return furthest_in_direction

def triangle_check(simplex):
    contains_origin = False
    
    c, b, a = simplex
    ao = -a
    ab = b - a  # from point A to B
    ac = c - a  # from point A to C

    abc = cross(ab, ac)  # Compute the triangle's normal

    # Test away from triangle in direction of line ac planar normal
    if dot(cross(abc, ac), ao) >= 0:

        if dot(ac, ao) >= 0:
            #Origin is outside triangle closest to line ac
            simplex = [a, c]
            d_tmp = cross(cross(ac, ao), ac)
            if (d_tmp.x == 0) and (d_tmp.y == 0) and (d_tmp.z == 0):
                #Edge case: if origin on the line, then will return cross product of zero, therefore simplex contains origin
                contains_origin = True
            else:
                d = d_tmp
        else:
            if dot(ab, ao) >= 0:
                #Origin is outside triangle closest to line ab
                simplex = [a, b]
                d_tmp = cross(cross(ab, ao), ab)
                if (d_tmp.x == 0) and (d_tmp.y == 0) and (d_tmp.z == 0):
                    #Edge case: if origin on the line, then will return cross product of zero, therefore simplex contains origin
                    contains_origin = True
                else:
                    #Otherwise, carry on as usual
                    d = d_tmp
            else:
                simplex = [a]
                d = ao

    # Test in the other direction ...
    else:
        if dot(cross(ab, abc), ao) >= 0:
            if dot(ab, ao) >= 0:
                #Origin is outside triangle closest to line ab
                simplex = [a, b]
                d_tmp = cross(cross(ab, ao), ab)
                if (d_tmp.x == 0) and (d_tmp.y == 0) and (d_tmp.z == 0):
                    #Edge case: if origin on the line, then will return cross product of zero, therefore simplex contains origin
                    contains_origin = True
                else:
                    #Otherwise, carry on as usual
                    d = d_tmp
            else:
                simplex = [a]
                d = ao
        else:
            #Nowhere else it could be besides in the triangle, whether above or below
            or_dir = dot(abc, ao)
            if or_dir < 0:
                #Should always be the case during the tetrahedral check
                simplex = [a, c, b]
                d = -abc
            elif or_dir > 0:
                simplex = [a, b, c]
                d = abc
            else:
                #Edge case, origin is contained within and on the plane of the triangle
                simplex = [a, b, c]
                d = abc
                contains_origin = True
                
    return simplex, d, contains_origin

def nearest_simplex(simplex, d):
    """
    A simplex is an array of points that may be represented as a:
    - 2 points = line
    - 3 points = triangle
    - 4 points = tetrahedron
    :param simplex: collection of points that can create simple shapes
    :param d:
    :return: updated points, updated direction and a bool of True if  shape contains origin.
    """
    
    contains_origin = False

    # __ Two Point Simplex: Line
    if len(simplex) == 2:
        b, a = simplex

        # a was found in a direction away from origin - its opposite is towards Origin.
        ao = -a

        # Will result in a new vector going in the direction of the newly added point
        ab = b - a

        # if AB is perpendicular (or at a > angle) to Origin (AO):
        #   - calculate the new vector to origin from the simplex
        # else if its a negative number:
        #   - we must be going in the wrong direction
        #   - make the direction the opposite of what we started with
        #   - drop the first entry since the origin is not in that direction and we don't want to recalc later.
        if dot(ab, ao) >= 0:
            d_tmp = cross(cross(ab, ao), ab)
            if (d_tmp.x == 0) and (d_tmp.y == 0) and (d_tmp.z == 0):
                #Edge case: if origin on the line, then will return cross product of zero, therefore simplex contains origin
                contains_origin = True
            else:
                #Otherwise, carry on as usual
                d = d_tmp
        else:
            #Should have already detected no collision if it gets here
            simplex = [a]
            d = ao

    # __ Three point simplex: Triangle
    elif len(simplex) == 3:
        simplex, d, contains_origin = triangle_check(simplex)

    # __ Four point simplex: Tetrahedron
    elif len(simplex) == 4:
        d, c, b, a = simplex

        ao = -a
        ab = b - a
        ac = c - a
        ad = d - a
        
        do = -d
        da = a - d
        db = b - d
        dc = c - d 
        
        
        """
        abc = cross(ab, ac)
        acd = cross(ac, ad)
        adb = cross(ad, ab)
        bcd = cross(bc, bd)
        
        if dot(abc, ao) >= 0 and dot(acd, ao) >= 0 and dot(adb, ao) >= 0:
            # must be behind all four faces so we have surrounded the origin!
            # Do not need to check opposite direction of normal since it should be\
            # the correct direction given a priori information from "triangle" stage
            return simplex, d, True
        """
        acb = cross(ac, ab)
        adc = cross(ad, ac)
        abd = cross(ab, ad)
        dcb = cross(dc, db)
        
        acb_dot = dot(acb, ao)
        adc_dot = dot(adc, ao)
        abd_dot = dot(abd, ao)
        
        if acb_dot <= 0 and adc_dot <= 0 and abd_dot <= 0:
            #Origin behind each triangular face therefore origin is surrounded and collision is detected
            return simplex, dcb, True
        
        elif acb_dot > 0:# and adc_dot <= 0 and abd_dot <= 0:
            #In front of acb face but not the other two
            simplex, d, contains_origin = triangle_check([c,b,a])
            
        elif adc_dot > 0:# and acb_dot <= 0 and abd_dot <= 0:
            #In front of adc face but not the other two
            simplex, d, contains_origin = triangle_check([d,c,a])
            
        elif abd_dot > 0: #acb_dot <= 0 and adc_dot <= 0:
            #In front of abd face but not the other two
            simplex, d, contains_origin = triangle_check([b,d,a])
            
        else: #acb_dot > 0 and adc_dot > 0 and abd_dot > 0:
            #Newest point of tetrahedron not far enough towards origin, most likely no collision
            simplex = [a]
            d = ao
            contains_origin = False

    # print("Direction:", d)
    return simplex, d, contains_origin


def gjk_intersection(p, q, initial_axis):
    """
    Collision detection alogrithm named after Gilbert Johnson Keerthi.
    :param p: Shape
    :param q: Shape
    :param initial_axis: Vec3
    :return:
    """
    # a -> Vec3
    # s -> list(): Simplex result
    # d -> Vec3 - Direction

    a = support(p, initial_axis) - support(q, -initial_axis)
    simplex = []
    simplex.append(a)
    direction = -a

    for i in range(100):  # cube only has 8 verts - should pass/fail before ten
        a = support(p, direction) - support(q, -direction)
        
        if dot(a, direction) < 0:
            # print("Collision NOT detected.")
            return False

        simplex.append(a)
        
        simplex, direction, contains_origin = nearest_simplex(simplex, direction)
        if contains_origin:
            return True
        
    print("Failed to find an answer.")
    return True


"""
# Create Test hulls
hull1 = Convex_Hull([[0,0],[0,10],[5,15],[10,15],[15,5],[10,0]])   
hull2 = Convex_Hull([[5,5],[10,5],[10,10],[5,10]])

direction = Vec3(1.0, 0.0, 0.0)

result = gjk_intersection(hull1, hull2, direction)

print("[{}] test_01_collision".format("passed" if result is True else "FAILED"))

hull1 = Convex_Hull([[0,0],[0,10],[5,15],[10,15],[15,5],[10,0]])   
hull2 = Convex_Hull([[50,50],[100,50],[100,100],[50,100]])

direction = Vec3(1.0, 0.0, 0.0)

result = gjk_intersection(hull1, hull2, direction)

print("[{}] test_02_collision".format("passed" if result is False else "FAILED"))
"""


# In[ ]:




