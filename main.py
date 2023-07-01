from skrobot.models.pr2 import PR2
from skrobot.interfaces.ros.pr2 import PR2ROSRobotInterface

model = PR2()
model.reset_manip_pose()
print(model.l_shoulder_pan_joint.joint_angle())
model.l_shoulder_pan_joint.joint_angle(model.l_shoulder_pan_joint.joint_angle() + 0.4)
model.r_shoulder_pan_joint.joint_angle(model.r_shoulder_pan_joint.joint_angle() - 0.4)
ri = PR2ROSRobotInterface(model)
ri.angle_vector(model.angle_vector())
