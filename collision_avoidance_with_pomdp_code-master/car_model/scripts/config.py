from geometry_msgs.msg import Vector3

# Maximum velocity
MAX_VEL = 1.0

# Publish rate for the nodes
PUBLISH_RATE = 20

# Colors
EMERALD = [46. / 255., 204. / 255., 113. / 255.]
BLUE = [52. / 255., 152. / 255., 219. / 255.]
MIDNIGHT_BLUE = [44. / 255., 62. / 255., 80. / 255.]

# Starting Positions for the Markers
POSITION_CAR = [5.0, 5.0, 0.5]
POSITION_PEDESTRIAN = [10.0, 30.0, 0.5]

# Range for the pedestrian to walk
RANGE_PEDESTRIAN = 1.0

# Time for the pedestrian to wait until new position is generated
WAIT = PUBLISH_RATE * 2

# Goal positions for the pedestrians
GOALS = {'trash': Vector3(5.0, 45.0, 0.5),
         'trash2': Vector3(30, 10, 0.5)}
