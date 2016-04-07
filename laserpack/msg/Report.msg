# This message is a complete report for an UAV to be sent via TCP
# Sending a whole message instead of 15 messages reduce the network 
# overhaul/connections
std_msgs/Header header
laserpack/RPYPose mocap
laserpack/RPYPose local
laserpack/RPYPose vision
laserpack/RPYPose setpoint
laserpack/Distance lasers_raw
laserpack/RPYPose lasers_pose
laserpack/RPYPose laser_filtered
Vector3 linear_acceleration
Vector3 linear_speed
Vector3 angular_speed
laserpack/Battery battery
bool connected
bool armed
bool guided
string mode