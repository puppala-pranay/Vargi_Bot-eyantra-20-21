#!/usr/bin/env python
"""
Simple action server which manages the movement of ur5_1 robot.
"""


from __future__ import print_function

import sys
from pkg_task5.msg import msgUr5_1Action
from pkg_task5.msg import msgUr5_1Result

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs
import geometry_msgs.msg
import actionlib
import rospkg
from pkg_vb_sim.srv import vacuumGripper
import yaml
from std_srvs.srv import Empty



class TaskSimpleActionServer:
    """
    Class to manage ur5_1 robot management upon receiving the goal
    """

    # Constructor
    def __init__(self):
        # Initialize Simple Action Server
        self._sas1 = actionlib.SimpleActionServer('/action_ur5_1',
                                                  msgUr5_1Action,
                                                  execute_cb=self.func_on_rx_goal,
                                                  auto_start=False)

        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=
                                                      self._robot_ns + "/robot_description",
                                                      ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self.scene = self._scene
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=
                                                          self._robot_ns + "/robot_description",
                                                          ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +
                                                             '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory, queue_size=1)
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

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        self.max_attempts = 5
        self._sas1.start()
        rospy.loginfo("Started Ur5 Simple Action Server.")

    def clear_octomap(self):
        """
        clears the octomap...
        :return:
        """
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):
        """
        sets the joint angles of ur5_1 to argument angles
        :param arg_list_joint_angles: joint angles to be set
        :return: return bool if succeed or not
        """

        # list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

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
        set the joint angles of ur5_1 to given joint angles by repeatedly calling
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
        :return: result
        """
        gripper_activator = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        result = gripper_activator(activation)
        return result

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Checks if Box is attached to planning scene or not when asked to add box by 'add_box' function
        :param box_is_known:
        :param box_is_attached:
        :param timeout:
        :return: true , if attached else false
        """

        box_name = self.box_name
        scene = self.scene
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

    def add_box(self, wpose, timeout=4):
        """
         Function to add a box to the end effector in planning scene .
        :param wpose: position to add box
        :param timeout:
        :return: true, if attached successfully else false
        """
        scene = self.scene

        # BEGIN_SUB_TUTORIAL add_box
        # Adding Objects to the Planning Scene
        # First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = wpose.orientation.w
        box_pose.pose.orientation.x = wpose.orientation.x
        box_pose.pose.orientation.y = wpose.orientation.y
        box_pose.pose.orientation.z = wpose.orientation.z
        box_pose.pose.position.x = wpose.position.x  # above the panda_hand frame
        box_pose.pose.position.y = wpose.position.y
        box_pose.pose.position.z = wpose.position.z
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.150, 0.150, 0.150))

        ## END_SUB_TUTORIAL
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
        robot = self._robot
        scene = self.scene
        eef_link = self._eef_link
        # group_names = self._group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, self.box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        """
        Function to detach the box attached to the end effector
        :param timeout:
        :return: true if detached successfully else false .
        """
        box_name = self.box_name
        scene = self.scene
        eef_link = self._eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        """
        Function to remove the box added to planning scene
        :param timeout:
        :return: true  if removed else false
        """
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

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
        rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        """
        Function to play the planned path from the file mentioned by repeatedly calling the function
        'moveit_play_planned_path_from_file' till arg_max_attempts or till succeed
        :param arg_file_path: path of the file
        :param arg_file_name: file name to be played
        :param arg_max_attempts: max attempts to try for execution
        :return:
        """
        number_attempts = 0
        flag_success = False

        while (number_attempts <= arg_max_attempts) and (flag_success is False):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            self.clear_octomap()

        return True

    def func_on_rx_goal(self, goal):
        """
        Callback function on receiving the goal from client
        plays the planned path from file which picks package from shelf and place on the belt
        :param goal: file number to play
        :return: None
        """
        rospy.loginfo("receieved the goal")
        rospy.loginfo(goal)
        if goal.count == -1:
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'zero_to_drop.yaml', 5)
        else:
            file_name = 'drop_to_pose' + str(goal.count) + '.yaml'
            self.moveit_hard_play_planned_path_from_file(self._file_path, file_name, self.max_attempts)
            self.toggle_vaccum_gripper(True)
            file_name = 'pose' + str(goal.count) + '_to_drop.yaml'
            self.moveit_hard_play_planned_path_from_file(self._file_path, file_name, self.max_attempts)
            self.toggle_vaccum_gripper(False)
        result = msgUr5_1Result()
        result.res = "success"
        rospy.loginfo("sending goal result to client")
        self._sas1.set_succeeded(result)


def main():
    """
    Main function
    Initialize the node and
    instantiates the 'TaskSimpleActionServer' class
    :return:  none
    """
    # 1. Initialize ROS Node
    rospy.init_node('node_simple_action_server_ur5_1')

    # 2. Create Simple Action Server object.
    obj_server = TaskSimpleActionServer()

    # 4. Do not exit and loop forever.
    rospy.spin()


if __name__ == '__main__':
    main()
