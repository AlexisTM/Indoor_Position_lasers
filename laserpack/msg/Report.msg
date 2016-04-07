# This message is a complete report for an UAV to be sent via TCP
# Sending a whole message instead of 15 messages reduce the network 
# overhaul/connections
std_msgs/Header header
RPYPose mocap
RPYPose local
RPYPose vision
RPYPose setpoint
Distance lasers_raw
RPYPose lasers_pose
RPYPose laser_filtered
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 linear_speed
geometry_msgs/Vector3 angular_speed
Battery battery
bool connected
bool armed
bool guided
string mode
