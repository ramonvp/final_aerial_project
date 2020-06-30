uint8 TYPE_FLOOR    = 0    # Aruco on floor type
uint8 TYPE_BOX      = 1    # Aruco on box type
uint8 type                 # Type of the detected object
uint8 id                   # Id starts with index 0
geometry_msgs/Vector3 dir  # Direction at which the object has been seen
                           # already expressed in ROS xyz frame
float64 distance           # Distance at which the object has been seen
