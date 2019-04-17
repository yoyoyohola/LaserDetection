# The Landmark 2D msg
# Created in 2019-03-26 by Fan ZHENG, VisionNav Robotics
#   position[2]: [x, y] coordinates in the map
#   covariance[4]: 2x2 covariance matrix of the position [x, y]

string id
float32[2] position
float32[4] covariance
float32 typical_intensity
float32 max_intensity
float32 min_intensity
