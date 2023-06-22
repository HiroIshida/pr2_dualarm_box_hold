from skrobot.models.pr2 import PR2
from skrobot.interfaces.ros.pr2 import PR2ROSRobotInterface

model = PR2()
model.reset_manip_pose()
ri = PR2ROSRobotInterface(model)
ri.angle_vector(model.angle_vector())
