#!/usr/bin/env python3

import rospy
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
from ik_antropomorphic_arm import calculate_ik
from move_joints import JointMover
from rviz_marker import MarkerBasics


class AntropomorphicEndEffectorMover(object):

    def __init__(self):

        # Start the RVIZ marker publisher
        self.markerbasics_object = MarkerBasics()
        self.unique_marker_index = 0

        # Start the mover
        self.robot_mover = JointMover()

        # End effector Pose commands Subscriber
        rospy.Subscriber("/ee_pose_commands", EndEffector, self.ee_pose_commands_callback)
        ee_pose_commands_data = None
        while ee_pose_commands_data is None and not rospy.is_shutdown():
            try:
                ee_pose_commands_data = rospy.wait_for_message("/ee_pose_commands", EndEffector, timeout=0.5)
            except:
                rospy.logwarn("Waiting for EE command Pose @topic /ee_pose_commands")
                pass
        
        self.Pee_x = ee_pose_commands_data.ee_xy_theta.x
        self.Pee_y = ee_pose_commands_data.ee_xy_theta.y
        self.Pee_z = ee_pose_commands_data.ee_xy_theta.z

        self.elbow_pol = ee_pose_commands_data.elbow_policy.data

        # End effector pose Subscriber
        rospy.Subscriber("/end_effector_real_pose", Vector3, self.end_effector_real_pose_callback)
        end_effector_real_pose_data = None
        while end_effector_real_pose_data is None and not rospy.is_shutdown():
            try:
                end_effector_real_pose_data = rospy.wait_for_message("/end_effector_real_pose", Vector3, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE command Pose in @topic /end_effector_real_pose")
                pass
        
        self.Pee_x_real = end_effector_real_pose_data.x
        self.Pee_y_real = end_effector_real_pose_data.y
        self.Pee_z_real = end_effector_real_pose_data.z
    
    def end_effector_real_pose_callback(self,msg):

        self.Pee_x_real = msg.x
        self.Pee_y_real = msg.y
        self.Pee_z_real = msg.z


    def ee_pose_commands_callback(self, msg):

        self.Pee_x = msg.ee_xy_theta.x
        self.Pee_y = msg.ee_xy_theta.y
        self.Pee_z = msg.ee_xy_theta.z

        self.elbow_pol = msg.elbow_policy.data

        r2 = 1.0
        r3 = 1.0 

        DH_parameters={"r2":r2, "r3":r3}

        theta_array, possible_solution = calculate_ik(Pee_x=self.Pee_x, Pee_y=self.Pee_y, Pee_z=self.Pee_z,
            DH_parameters=DH_parameters, elbow_config = self.elbow_pol)

        if possible_solution:
            theta_1 = theta_array[0]
            theta_2 = theta_array[1]
            theta_3 = theta_array[2]

            self.robot_mover.move_all_joints(theta_1, theta_2, theta_3)
            self.markerbasics_object.publish_point(self.Pee_x, self.Pee_y, self.Pee_z, index=self.unique_marker_index)
            self.unique_marker_index += 1 
        else:
            rospy.logerr("no possible solution to reach that pose")

def main():
    rospy.init_node('antropomorphic_end_effector_mover_node')

    antropomorphic_object = AntropomorphicEndEffectorMover()
    rospy.spin()



if __name__ == '__main__':
    main()