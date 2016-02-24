"""
This script allows to get the position of the laser in space (static axis based on the origin (0,0,0) on the gravity center of the drone)


First, rotate the point where we were to the point where we are (orinal laser position to the rotated laser position)
Second, rotate the vector from the original position to the rotated position
Third, with M, the distance to the wall meseared, define the point where is the wall, from the laser through the vector direction to the distance measured
"""

from numpy import dot
from numpy import pad
from transformations import *
from dumb_functions import *
from math import sqrt

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

### Known values
# Mesure
M = 200
Laser = (10,10,20)
Orientation = (1,0,0)

### PixHawk values
roll = 5
pitch = 10
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
q = quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
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

