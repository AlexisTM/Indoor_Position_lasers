"""
This script allows to get the position of the laser in space (static axis based on the origin (0,0,0) on the gravity center of the drone)


First, rotate the point where we were to the point where we are (orinal laser position to the rotated laser position)
Second, rotate the vector from the original position to the rotated position
Third, with M, the distance to the wall meseared, define the point where is the wall, from the laser through the vector direction to the distance measured
"""

from __future__ import division
from numpy import dot, pad
from transformations import *
from dumb_functions import *
from math import sqrt, asin, acos, sin, cos
from time import time

SPEED_TEST_ENABLE = False

# rotate a point, vector or quaternion by a quaternion
# The rotation DO NOT change the size of the distance to the origin
# input p : a tuple (x,y,z) or a quaternion (x,y,z,w)
# Create a matrix from the quaternion q and return the dot product of the vector/point/quaternion & p
def rotate(p, q):
    # if we got only a point/vector, convert it to (x,y,z,w), with w = 0
    if(len(p) == 3):
        p = p + (0,)
    m = quaternion_matrix(q)
    return dot(m, p)

# computes the X point where the laser points to on the well
# input p : point with the laser
# input v : the vector direction
# input M : distance measured
def extrapolate(p,v,M):
    # p is the origin
    # M = 20-5000 [cm]
    # find k, the factor to multiply v to get length(v) = M (interpolation)
    # Then add p to v to get the result
    K = sqrt(M*M/(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]))
    v = (K*v[0]+p[0], K*v[1]+p[1], K*v[2]+p[2])
    return v

# input point  : A point in space on which we applied a quaternion q
# input q      : The quaternion applied to point
# output point : The posistion of the point if there where NO roll/pitch (yaw only)
def getYawPosition(p, q): 
    # if we got only a point/vector, convert it to (x,y,z,w), with w = 0
    if(len(p) == 3):
        p = p + (0,)
    zaxis = (0, 0, 1)
    qi = quaternion_inverse(q)
    p = rotate(p, qi)
    _, _, yaw = list(euler_from_quaternion(q, axes='sxyz'))
    qz = quaternion_about_axis(yaw, zaxis)
    return rotate(p, qz)

def length(point):
    return sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2])


# input q : Quaternion
# output normalized quaternion
def normalizeQ(q):
    l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if(l == 1 or l == 0):
        return q
    return (q[0]/l, q[1]/l, q[2]/l, q[3]/l)

# input q : Quaternion
# output axis (x,y,z) and angle
def Quaternion2AxisAngle(q):
    q = normalizeQ(q)
    angle = 2 * acos(q[3]);
    s = sqrt(1-q[3]*q[3]);
    if(s <= 0.001):
        return (q[0], q[1], q[2]), angle
    return (q[0]/s, q[1]/s, q[2]/s), angle


# input q : Quaternion
# output axis (x,y,z) and angle
def fastQuaternion2AxisAngle(q):
    l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    q3 = q[3]/l
    angle = 2 * acos(q3/l);
    s = sqrt(1-q3*q3);
    if(s <= 0.001):
        return (q[0], q[1], q[2]), angle
    return (q[0]/s, q[1]/s, q[2]/s), angle

# CARE : Counterclock rotation whenn u point towards observer
# input vector3
# input axis (x,y,z) NORMALIZED
# input angle NORMALIZED
def rotationAxisAngle(vector, axis, angle):
    cost = cos(angle)
    omcost = 1-cost
    sint = sin(angle)
    return ((omcost*axis[0]*axis[2] + sint*axis[1])*vector[2] + (-sint*axis[2] + omcost*axis[0]*axis[1])*vector[1] + (omcost*axis[0]*axis[0]  + cost)*vector[0], \
    (omcost*axis[1]*axis[2] - sint*axis[0])*vector[2] + (omcost*axis[1]*axis[1]  + cost)*vector[1] + (omcost*axis[0]*axis[1] + sint*axis[2])*vector[0], \
    (omcost*axis[2]*axis[2]  + cost)*vector[2] + (omcost*axis[1]*axis[2] + sint*axis[0])*vector[1] + (omcost*axis[0]*axis[2] - sint*axis[1])*vector[0])

def quaternionRotation(p, q):
    axis, angle = Quaternion2AxisAngle(q)
    return  rotationAxisAngle(p, axis, angle)

def distance(a,b):
    return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]))

def exportCSV(data, data2, distances):
    for d in range(0,len(data)):
        print data[d][0],data[d][1],data[d][2],data2[d][0],data2[d][1],data2[d][2], distances[d]

### Known values
# Mesure
M = 200
Laser = (10,20,30)
Orientation = (1,0,0)

### PixHawk values
roll = 10
pitch = 20
yaw = 30
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

Eulers = [roll,pitch, yaw]
Eulers = deg2rad(Eulers)
qCreation = quaternion_from_euler(Eulers[0], Eulers[1], Eulers[2], axes="sxyz")
#qWanted = quaternion_from_euler(0, 0, Eulers[2], axes="sxyz")
# equals
qWanted = quaternion_about_axis(Eulers[2], zaxis)

# I apply a transformation
# Then I got a point
pointCreation = rotate(Laser, qCreation)
pointWanted = rotate(Laser, qWanted)
print "Yaw only point: ", pointWanted
print "RollPitchYaw point", pointCreation
# Now, I have Q from the PixHawk & the original point
# I have to transform the transformed point, to go back to => yaw dependent only point
#
# Then I get eulers from the quaternion with another convention => szxy
# Create a new quaternion with that same convention => szxy, but with yaw = 0, pitch = -pitch, roll = -roll
# Apply this to the point => Yaw dependant only
# print rad2deg(list(euler_from_quaternion(q, axes='sxyz')))
yawpoint = getYawPosition(pointCreation, qCreation)
print "result :", yawpoint
print "vector :", rotate(xaxis, qCreation)
print "rot :", rotate(xaxis, qCreation)


#PROGRAM 
print "----------------------------------"
print "-------------PROGRAM--------------"
print "----------------------------------"
print "# KNOWN #"
print "Measure:", M
print "Angles XYZ:", (roll, pitch, yaw)
q = quaternion_from_euler(deg2radf(roll), deg2radf(pitch), deg2radf(yaw), axes="sxyz")
print "Quaternion:", q
print "Laser position:", Laser
print "Laser orientation:", Orientation
print ""

print "# COMPUTED #"
FLP = rotate(Laser, q)
print "Final Laser Position:", FLP
FLO = rotate(Orientation, q)
print "Final Laser Orientation:", FLO
X = extrapolate(FLP, FLO, M)
print "Pointed point on the wall:", X
N = rotate(X, quaternion_inverse(q))
# Possible solution => From Quaternion to Axis Angle => http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
# It gives the possibility to get only an angle to get the whole transformation 
# Then, we would be able to renormalise with cos(angle)
# N IS FALSE, it do not touch the wall, but is much further ! CARE
# It is because we are still on the sphere
print "Original distance : ",N
Yaw = getYawPosition(X, q)
print "Yaw distance : ",Yaw

print "--- TESTS ---"
axis, angle = Quaternion2AxisAngle(q)
print axis, angle, rad2degf(angle), Laser
k = rotationAxisAngle(Laser, axis, -angle)
print k, length(k), length(Laser)

print "--- Quaternion rotation ---"
q = quaternion_from_euler(deg2radf(roll), deg2radf(pitch), deg2radf(yaw), axes="sxyz")
print rad2deg(list(euler_from_quaternion(q, axes="sxyz")))
q = quaternion_from_euler(deg2radf(roll), deg2radf(pitch), 0, axes="sxyz")

result = list()
result2 = list()
distances = list()
p = (1,1,1)
v = (1,0,0)
M = 50
p2 = (10,10,10)
v2 = (1,0,0)
M2 = 55
for x in range(-180,181):
    ro = deg2radf(10)
    pi = deg2radf(20)
    qua = quaternion_from_euler(ro, pi, deg2radf(x), axes="sxyz")
    point = quaternionRotation(p, qua)
    orientation = quaternionRotation(v, qua)
    point2 = quaternionRotation(p2, qua)
    orientation2 = quaternionRotation(v2, qua)
    print point, orientation, M
    print point2, orientation2, M2


    pointOnWall = extrapolate(point, orientation, M)
    pointOnWall2 = extrapolate(point2, orientation2, M2)
    result.append(pointOnWall)
    result2.append(pointOnWall2)
    print distance(pointOnWall, pointOnWall2), distance(point, point2)
    #distances.append(distance(pointOnWall, pointOnWall2))

#print result
#exportCSV(result, result2, distances)



if(SPEED_TEST_ENABLE):
    print "--- SPEED TESTS ---"
    print "--- Quaternion To Axis Angle Without complete normalization ---"
    q = quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
    startTimeSlow = time()
    for x in xrange(10000):
        Quaternion2AxisAngle(q)
    stopTimeSlow = time()

    startTimeFast = time()
    for x in xrange(10000):
        fastQuaternion2AxisAngle(q)
    stopTimeFast = time()

    print stopTimeSlow-startTimeSlow, stopTimeFast-startTimeFast
    print ((stopTimeSlow-startTimeSlow)-(stopTimeFast-startTimeFast))/ (stopTimeSlow-startTimeSlow) *100
    # GAIN 18% speed with reduced normalization

    print "--- Quaternion rotation vs Quaternion via Axis Angle rotation ---"
    q = quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
    p = (10,20,30)
    startTimeSlow = time()
    for x in xrange(1000):
        rotate(p, q)
    stopTimeSlow = time()

    startTimeFast = time()
    for x in xrange(1000):
        quaternionRotation(p,q)
    stopTimeFast = time()

    print stopTimeSlow-startTimeSlow, stopTimeFast-startTimeFast
    print ((stopTimeSlow-startTimeSlow)-(stopTimeFast-startTimeFast))/ (stopTimeSlow-startTimeSlow) *100
    # GAIN 25% speed with axis angle rotation
#print 