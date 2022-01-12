import math

class MapMath():
    """ 
    input: pose of the robot (robot_pose), pose of the human (human_pose)
    ouput: angle robot should be at to face human (rad)

    Calculates the angle the z-rotation of the robot needed in order to face where the person is.
    """
    def angle_to_face_point(self, robot_pose, human_pose):
        rad = math.atan2(human_pose[1] - robot_pose[1], human_pose[0] - robot_pose[0])
        deg = math.degrees(rad)
        print(rad, deg)
        
        return rad

    """ 
    input: pose of the robot (robot_pose)
    ouput: pose of the human, assuming facing the robot (human_pose)

    Simple calculates the oppisite angle of the robot.
    """
    def human_pose_rot(self, robot_pose):
        human_pose = (robot_pose[2] + math.pi) % (2 * math.pi)
        return human_pose

    """ 
    input: pose of the human (human_pose), desired radius from human (radius), desired angle (angle), and frame offset (offset)
    output: target pose of the robot (target_robot_pose)

    Calculates new point using trigonometry.
    """
    def new_point_on_circumference(self, human_pose, radius, angle, offset):
        angle = math.radians(angle)

        angle = (angle + offset) % (2 * math.pi)

        x = human_pose[0] + (radius * math.cos(angle))
        y = human_pose[1] + (radius * math.sin(angle))

        target_robot_pose = [x, y, 0]

        return target_robot_pose

if __name__ == '__main__':
    m = MapMath()

    p1 = [3.414, 0.585, 0.0, 0.0]
    p2 = [2.0, 2.0, 0.0]

    print(p1, p2)

    p1[2] = m.angle_to_face_point(p1, p2)
    p2[2] = m.human_pose_rot(p1[2])

    print(p1, p2)

    x, y = m.new_point_on_circumference(p2, 2.0, 45, p2[2])
    print(x, y)

    