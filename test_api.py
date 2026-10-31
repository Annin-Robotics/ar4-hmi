from ARrobots.ARrobot import ARRobot
import time

robot = ARRobot(port="COM4")
robot.connect()

print("Robot:", robot.hello())

#
# Z-axis jog test
#
pose = robot.get_pose()
print("Starting Pose:", pose)

for i in range(3):
    print(f"Z Up - Cycle {i+1}")
    pose[2] += 50
    robot.move_pose(pose, speed=20)

    time.sleep(1)

    print(f"Z Down - Cycle {i+1}")
    pose[2] -= 50
    robot.move_pose(pose, speed=20)

    time.sleep(1)

#
# J1 jog test
#
joints = robot.get_joints()
print("Starting Joints:", joints)

for i in range(3):
    print(f"J1 Left - Cycle {i+1}")
    joints[0] -= 10
    robot.move_joints(joints, speed=20)

    time.sleep(1)

    print(f"J1 Right - Cycle {i+1}")
    joints[0] += 10
    robot.move_joints(joints, speed=20)

    time.sleep(1)

robot.disconnect()

print("Done")