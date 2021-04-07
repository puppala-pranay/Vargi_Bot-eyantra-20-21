#!/usr/bin/env python
"""
task5_ur5_2_server manages the movement of ur5_2 robot
"""

import sys
import copy
import rospy
import actionlib

from pkg_task5.msg import msgUr5_2Action
from pkg_task5.msg import msgUr5_2Result
from pkg_task5.msg import msgUr5_2Feedback
from pkg_vb_sim.srv import vacuumGripper

import rospkg
import moveit_commander
import moveit_msgs.msg

import geometry_msgs
import yaml
from std_srvs.srv import Empty

import tf2_ros


class TaskSimpleActionServer:
    """
    class which manages the movement of ur5_2 upon receiving the goal
    """

    # Constructor
    def __init__(self):
        # Initialize Simple Action Server
        self._sas = actionlib.SimpleActionServer('/action_ur5_2',
                                                 msgUr5_2Action,
                                                 execute_cb=self.func_on_rx_goal,
                                                 auto_start=False)

        self.yellow_box = [-0.1059029817135313, -0.3943529815450777, -0.46656184801507106, -0.7106819427996118,
                           -1.5698784239386594, 3.0356176069278824]

        self.green_box = [1.8005493706387412, -1.9592796441597233, -1.0096147157616349, -1.7427427625125897,
                          1.5699236871783162, 1.8000591174115605]

        self.red_box = [-1.5720650040971282, -1.9816924634742996, -0.9773975214726383, -1.7538158257092498,
                        1.5700438544296142, -1.571423929644121]

        box_length = 0.15  # Length of the Package
        vacuum_gripper_width = 0.115  # Vacuum Gripper Width
        # delta = vacuum_gripper_width + (box_length / 2)  # 0.19
        # Teams may use this info in Tasks

        # Get The Home Position

        self.ur5_2_home_pose = geometry_msgs.msg.Pose()
        self.ur5_2_home_pose.position.x = -0.8
        self.ur5_2_home_pose.position.y = 0
        self.ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length / 2) + 0.01

        # This to keep EE parallel to Ground Plane
        self.ur5_2_home_pose.orientation.x = -0.5
        self.ur5_2_home_pose.orientation.y = -0.5
        self.ur5_2_home_pose.orientation.z = 0.5
        self.ur5_2_home_pose.orientation.w = 0.5

        self._robot_ns = '/ur5_2'
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description",
                                                      ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self.scene = self._scene
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns + "/robot_description",
                                                          ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +
                                                             '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns +
                                                                        '/execute_trajectory',
                                                                        moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rosp = rospkg.RosPack()
        self._pkg_path = rosp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))
        self.packages_names = ["00", "01", "02", "10", "11", "12", "20", "21", "22",
                               "30", "31", "32"]

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        self.moveit_hard_play_planned_path_from_file(self._file_path,
                                                     'zero_to_drop.yaml', 0)
        self.max_attempts = 5
        self._sas.start()
        rospy.loginfo("Started Ur5 Simple Action Server.")

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        """
        Function to calculate the TF of arg_frame_2 with respect to arg_frame_1
        :param arg_frame_1: reference frame
        :param arg_frame_2: target frame
        :return: transform
        """
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1,
                                                    arg_frame_2, rospy.Time())

            rospy.loginfo("\n" +
                          "Translation: \n" +
                          "x: {} \n".format(trans.transform.translation.x) +
                          "y: {} \n".format(trans.transform.translation.y) +
                          "z: {} \n".format(trans.transform.translation.z) +
                          "\n" +
                          "Orientation: \n" +
                          "x: {} \n".format(trans.transform.rotation.x) +
                          "y: {} \n".format(trans.transform.rotation.y) +
                          "z: {} \n".format(trans.transform.rotation.z) +
                          "w: {} \n".format(trans.transform.rotation.w))
            return trans

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    def clear_octomap(self):
        """
                clears the octomap...
                :return:
        """
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns +
                                                         "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):
        """
                sets the joint angles of ur5_2 to argument angles
                :param arg_list_joint_angles: joint angles to be set
                :return: return bool if succeed or not
        """

        # list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)

        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        self._group.stop()
        self._group.clear_pose_targets()
        # list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        # pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if flag_plan:
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        """
                set the joint angles of ur5_2 to given joint angles by repeatedly calling
                'set_joint_angles' function till it succeed or till arg_max_attempts
                :param arg_list_joint_angles: joint angles to be set
                :param arg_max_attempts: max attempts to set the joint angles
                :return: None
        """

        number_attempts = 0
        flag_success = False

        while (number_attempts <= arg_max_attempts) and (flag_success is False):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()

    def toggle_vaccum_gripper(self, activation):
        """
                Function to on/off the vacuum gripper
                :param activation: on/off
                :return: true if activated
        """
        gripper_activator = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
                                               vacuumGripper)
        result = gripper_activator(activation)
        return result

        # def cartesian_translate(self, x, y, z):
        #    self.cartesian_path.ee_cartesian_translation(x, y, z)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False,
                              timeout=4):
        """
                Checks if Box is attached to planning scene or not when asked to add box by 'add_box' function
                :param box_is_known:
                :param box_is_attached:
                :param timeout:
                :return: true , if attached else false
        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        #  BEGIN_SUB_TUTORIAL wait_for_scene_update
        #
        # Ensuring Collision Updates Are Received
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        # END_SUB_TUTORIAL

    def add_box(self, wpose, timeout=4):
        """
         Function to add a box to the end effector in planning scene .
        :param wpose: position to add box
        :param timeout:
        :return: true, if attached successfully else false
        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.

        scene = self.scene

        # BEGIN_SUB_TUTORIAL add_box
        #
        # Adding Objects to the Planning Scene
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = wpose.rotation.w
        box_pose.pose.orientation.x = wpose.rotation.x
        box_pose.pose.orientation.y = wpose.rotation.y
        box_pose.pose.orientation.z = wpose.rotation.z
        box_pose.pose.position.x = wpose.translation.x  # above the panda_hand frame
        box_pose.pose.position.y = wpose.translation.y
        box_pose.pose.position.z = wpose.translation.z
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.150, 0.150, 0.150))

        # END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        """
         Function to attach the added box to end effector
         :param timeout:
         :return: true , if attached successfully else false
         """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self._robot
        scene = self.scene
        eef_link = self._eef_link
        # group_names = self._group_names

        # BEGIN_SUB_TUTORIAL attach_object
        #
        # Attaching Objects to the Robot
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        # robot be able to touch them without the planning scene reporting the contact as a
        # collision. By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. For the Panda
        # robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        # you should change this value to the name of your end effector group name.
        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        # END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True,
                                          box_is_known=False,
                                          timeout=timeout)

    def detach_box(self, timeout=4):
        """
        Function to detach the box attached to the end effector
        :param timeout:
        :return: true if detached successfully else false .
        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self._eef_link

        # BEGIN_SUB_TUTORIAL detach_object
        #
        # Detaching Objects from the Robot
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        # END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True,
                                          box_is_attached=False,
                                          timeout=timeout)

    def remove_box(self, timeout=4):
        """
        Function to remove the box added to planning scene
        :param timeout:
        :return: true  if removed else false
        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        # BEGIN_SUB_TUTORIAL remove_object
        #
        # Removing Objects from the Planning Scene
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can remove the box from the world.
        scene.remove_world_object(box_name)

        # **Note:** The object must be detached before we can remove it from the world
        # END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False,
                                          box_is_known=False,
                                          timeout=timeout)

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Function to play the planned path from the file mentioned.
        :param arg_file_path: path to file
        :param arg_file_name: file name
        :return: result of execution
        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        self._group.stop()
        self._group.clear_pose_targets()
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self,
                                                arg_file_path,
                                                arg_file_name,
                                                arg_max_attempts):
        """
        Function to play the planned path from the file mentioned by repeatedly calling the function
        'moveit_play_planned_path_from_file' till arg_max_attempts or till succeed
        :param arg_file_path: path to file
        :param arg_file_name: file to be played
        :param arg_max_attempts: maximum attempts to be played
        :return:
        """

        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            self.clear_octomap()

        return True

    def convert_to_world_from_ee(self, transf):
        """
        Function to calculate the x,y,z distance to translate to package on belt
        :param transf: position of package
        :return: x,y,z distances
        """
        out = geometry_msgs.msg.Point()
        out.x = transf.transform.translation.x + 0.8  # trans.transform.translation.x
        out.y = transf.transform.translation.y  # - trans.transform.translation.y
        out.z = transf.transform.translation.z - 1.2  # trans.transform.translation.z
        return out

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """
        Function for cartesian translation of end effector
        :param trans_x: 'x' distance to translate
        :param trans_y: 'y' distance to translate
        :param trans_z: 'z' distance to translate
        :return: None
        """
        # 1. Create a empty list to hold waypoints

        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + trans_x
        wpose.position.y = waypoints[0].position.y + trans_y
        wpose.position.z = waypoints[0].position.z + trans_z
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)  # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartesian Path can be found here,

        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)
        self._group.stop()
        # self._group.clear_pose_targets()
        # self.clear_octomap()

    def func_on_rx_goal(self, goal):
        """
        Callback function on receiving the goal from the client
        this picks the package on the belt and drops in respective bin
        :param goal: goal sent by the cliant
        :return: None
        """
        rospy.loginfo("received the goal")
        rospy.loginfo(goal)

        color = goal.color
        package = goal.package
        # trans = self.func_tf_print("world", "ur5_2_tf/ur5_ee_link")
        transf = self.func_tf_print("world", package)
        out = self.convert_to_world_from_ee(transf)
        rospy.loginfo("out = ")
        rospy.loginfo(out)
        self.add_box(transf.transform)
        self.ee_cartesian_translation(out.x, out.y, out.z+0.2)
        self.toggle_vaccum_gripper(True)
        self.attach_box()
        # self.hard_set_joint_angles([0,-1.57,0,0,0,0],5)
        # path = "top_to_"+color+".yaml"
        # self.moveit_hard_play_planned_path_from_file(self._file_path,path,5)

        if color == "yellow":
            self.hard_set_joint_angles(self.yellow_box, self.max_attempts)
        elif color == "red":
            self.hard_set_joint_angles(self.red_box, self.max_attempts)
        else:
            self.hard_set_joint_angles(self.green_box, self.max_attempts)

        self.toggle_vaccum_gripper(False)
        self.detach_box()
        feedback = msgUr5_2Feedback()
        feedback.status = "Shipped"
        self._sas.publish_feedback(feedback)
        self.remove_box()
        path = color + "_to_drop.yaml"
        self.moveit_hard_play_planned_path_from_file(self._file_path,
                                                     path, self.max_attempts)

        result = msgUr5_2Result()
        result.res = "success"
        rospy.loginfo("sent goal result  for server2 to client")
        self._sas.set_succeeded(result)


def main():
    """
    Main Function
    Initialize the node
    OInstantiate the TaskSimpleActionServer class
    :return:
    """
    # 1. Initialize ROS Node
    rospy.init_node('node_simple_action_server_ur5_2')

    # 2. Create Simple Action Server object.
    TaskSimpleActionServer()

    # 4. Do not exit and loop forever.
    rospy.spin()


if __name__ == '__main__':
    main()
