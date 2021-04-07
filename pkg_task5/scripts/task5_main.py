#!/usr/bin/env python
"""
Task5_main.py is the python module which manages all the action servers and
sensors , cameras and makes the work done.
"""
from __future__ import print_function

import datetime
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_task5.msg import msgUr5_1Action
from pkg_task5.msg import msgUr5_1Goal

from pkg_task5.msg import msgUr5_2Action
from pkg_task5.msg import msgUr5_2Goal

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgRosIotGoal

import rospy
import actionlib
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge, CvBridgeError
import cv2





class Task5Interface(object):
    """
    Class to manage the task5 functions and servers.
    """

    def __init__(self):
        """
        Init function - initializes servers and initial variables
        """

        super(Task5Interface, self).__init__()

        self._ac3 = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)
        self._ac3.wait_for_server()
        rospy.loginfo("Ros Iot bridge server is up")

        self._ac = actionlib.SimpleActionClient('/action_ur5_1', msgUr5_1Action)
        self._ac.wait_for_server()
        rospy.loginfo("Action Ur5_1 server is up")

        self._ac2 = actionlib.SimpleActionClient('/action_ur5_2', msgUr5_2Action)
        rospy.loginfo("checking for server")

        self._goal_handles = {}

        self.next_file = 0
        self.ready_1 = True    # Is ur5_1 ready for next goal
        self.ready_2 = True    # iS UR5_2 ready for next goal
        self.power = 0         # Current power of conveyor built

        self.dispatched = []    # List of Dispatched packages
        self.orders = []        # List of Pending Orders
        self.picked = []        # List of Packages Picked From Shelf
        self.reds = []          # List of Positions with red packages
        self.greens = []        # List of Positions with green packages
        self.yellows = []       # List of Positions with yellow packages
        self.inventory = []     # List of Packages in inventory
        self.shipped = []       # List of Packages shipped

        self.items = {
            "red": ["Medicine", 500, "HP"],
            "yellow": ["Food", 400, "MP"],
            "green": ["Clothes", 300, "LP"]
        }

    def send_goal_1(self, file_no):
        """
        The Function To Send The Goal To '/action_ur5_1' server.
        '/action_ur5_1' server moves ur5_1 to given package and puts on
        the conveyor belt.
        """

        # Create Goal message for Simple Action Server
        goal = msgUr5_1Goal(count=file_no)

        self._ac.send_goal(goal, done_cb=self.completed_callback_1,
                           feedback_cb=self.feedback_callback_1)

        rospy.loginfo("Goal has been sent to ur5_1.")

    def send_goal_2(self, package, color):
        """
        The Function To Send The Goal To '/action_ur5_2' server
        '/action_ur5_2' server moves ur5_2 to given package on conveyor belt
         and puts in the respective belt.
        """

        # Create Goal message for Simple Action Server
        goal = msgUr5_2Goal(package=package, color=color)

        self._ac2.send_goal(goal, done_cb=self.completed_callback_2,
                            feedback_cb=self.feedback_callback_2)

        rospy.loginfo("Goal has been sent to ur5_2.")

    def send_goal_ros_iot(self, sheet, data=None):
        """
        The Function To Send The Goal To '/action_ros_iot' server.
        '/action_ros_iot' server uploads the data given to the spreadsheet and
        publishes to mqtt client
        :param sheet : sheet to which data has to be uploaded
        :param data : data to be uploaded
        """
        # Create a Goal Message object
        goal = msgRosIotGoal()
        goal.sheet = sheet

        goal.entries = []

        if sheet == "Inventory":
            rospy.loginfo("received goal.")
            for inv in self.inventory:
                parameters = msgMqttSub()
                parameters.SKU = inv["SKU"]
                parameters.item = self.items[inv["color"]][0]
                parameters.priority = self.items[inv["color"]][2]
                parameters.storage = "R"+inv["storagenum"][0]+" C"+inv["storagenum"][1]
                parameters.cost = self.items[inv["color"]][1]
                parameters.qty = "1"
                goal.entries.append(parameters)
                rospy.loginfo("received goal.")
                rospy.loginfo(goal.entries)
        elif sheet == "IncomingOrders":
            parameters = msgMqttSub()
            parameters.item = data["item"]
            parameters.priority = self.items[data["color"]][2]
            parameters.cost = self.items[data["color"]][1]
            parameters.qty = data["qty"]
            parameters.order_id = data["order_id"]
            parameters.date_time1 = data["order_time"]
            parameters.city = data["city"]
            parameters.lon = data["lon"]
            parameters.lat = data["lat"]
            goal.entries.append(parameters)
        elif sheet == "OrdersDispatched":
            parameters = msgMqttSub()
            parameters.item = data["item"]
            parameters.priority = self.items[data["color"]][2]
            parameters.cost = self.items[data["color"]][1]
            parameters.qty = data["qty"]
            parameters.order_id = data["order_id"]
            parameters.date_time1 = data["order_time"]
            parameters.city = data["city"]
            parameters.status = "YES"
            goal.entries.append(parameters)
        elif sheet == "OrdersDispatched":
            parameters = msgMqttSub()
            parameters.item = data["item"]
            parameters.priority = self.items[data["color"]][2]
            parameters.cost = self.items[data["color"]][1]
            parameters.qty = data["qty"]
            parameters.order_id = data["order_id"]
            parameters.date_time1 = data["order_time"]
            parameters.city = data["city"]
            parameters.status = "YES"
            goal.entries.append(parameters)
            rospy.loginfo(" order dispatched received")
        elif sheet == "OrdersShipped":
            parameters = msgMqttSub()
            parameters.item = data["item"]
            parameters.priority = self.items[data["color"]][2]
            parameters.cost = self.items[data["color"]][1]
            parameters.qty = data["qty"]
            parameters.order_id = data["order_id"]
            parameters.date_time1 = data["order_time"]
            parameters.city = data["city"]
            parameters.status = "YES"
            dates12 = datetime.datetime.now() + datetime.timedelta(days=data["priority"])
            parameters.date_time2 = dates12.strftime("%Y-%m-%d")
            goal.entries.append(parameters)

        rospy.loginfo("Sending goal.")

        # self.on_transition - It is a function pointer to a function which will be called when
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac3.send_goal(goal, self.on_transition, None)

        return goal_handle

    def on_transition(self, goal_handle):

        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        """
        Function called when the status of the goal sent to action server '/action_ros_iot'.
        :param goal_handle: Handle To the goal whose status is changed
        :return: None
        """

        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Waiting for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())

            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_succesdate_times:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

    def completed_callback_1(self, status, result):
        """
        1)Function called when the goal sent to simple action client
        'action/ur_5_1' is accomplished i.e
         when package is Dispatched

        2)This function sends the goal to 'action/ros_iot' to update the
        OrdersDispatched sheet
        :param status: status of goal
        :param result: result that is sent from server to client
        :return: None
        """
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))
        if self.dispatched:
            order = self.dispatched[len(self.dispatched)-1]
            order["order_time"] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self.send_goal_ros_iot("OrdersDispatched", order)
        self.ready_1 = True

    def completed_callback_2(self, status, result):
        """
        1)Function called when the goal sent to simple action client
        'action/ur_5_2' is accomplished i.e
        when package is Shipped
        :param status: status of goal
        :param result: result that is sent from server to client
        :return: None
        """
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))

        self.ready_2 = True

    def feedback_callback_1(self, feedback):
        """
        Function Called for processing the feedback of goals sent to 'action/ur_5_1'
        :param feedback: feedback from the server
        :return: None
        """
        rospy.loginfo(feedback)

    def feedback_callback_2(self, feedback):
        """
        Function Called for processing the feedback of goals sent to 'action/ur_5_2'.

        When the feedback 'shipped' is received , required data is uploaded to the ShippedOrders spreadsheet
        :param feedback: feedback from the server
        :return: None
        """
        if feedback.status == "Shipped":
            order = self.shipped.pop()
            order["order_time"] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self.send_goal_ros_iot("OrdersShipped", order)
        rospy.loginfo(feedback)

    def toggle_conveyer_belt(self, power):

        """
        Function To change the power of conveyor belt
        :param power: power of conveyor belt to be set
        :return: result whether successful in setting the speed or not
        """
        belt_control = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        result = belt_control(power)
        self.power = power
        return result

    def get_order_callback(self, data):

        """
        Function Called when order is received from topic '/ros_iot_bridge/mqtt/sub'
        Function makes a dictionary item of order and adds to pending orders list and uploads to
        spreadsheet .
        :param data: order data from topic subscribed
        :return: None
        """

        color = ""
        priority = 0
        if data.item == "Medicine":
            color = "red"
            priority = 1
        elif data.item == "Food":
            color = "yellow"
            priority = 3
        else:
            color = "green"
            priority = 5

        order = {
            "city": data.city,
            "order_time" : data.date_time1,
            "order_id": data.order_id,
            "lon": data.lon,
            "qty": data.qty,
            "item": data.item,
            "lat": data.lat,
            "color": color,
            "priority": priority

        }
        self.orders.append(order)
        self.send_goal_ros_iot("IncomingOrders", order)
        self.orders.sort(key=lambda x: x["priority"])
        return

    def camera2_callback(self, data):
        """
        Callback function of subscriber to logical camera 2 with topic : '/eyrc/vb/logical_camera_2'

        This Function checks if package is below the camera or not.
        if package is under the camera then it finds the color of package from dispatched orders list
        and picks and drops the package in respective bin by sending the color to server '/action/ur_5_2'

        :param data: data sent from subscriber
        :return: None
        """
        for model in data.models:
            if len(data.models) == 1 and model.type == "ur5":
                if self.power == 0:
                    self.toggle_conveyer_belt(100)

            elif len(data.models) > 1 and model.type == "ur5":
                continue

            elif model.type != "ur5" and model.pose.position.y <= 0:
                if self.power > 0:
                    self.toggle_conveyer_belt(0)

                if self.ready_2:
                    package = "logical_camera_2_" + model.type + "_frame"
                    order = self.dispatched.pop(0)
                    self.shipped.append(order)
                    self.send_goal_2(package, order["color"])
                    self.ready_2 = False
                break
            elif self.power == 0:
                self.toggle_conveyer_belt(100)

        return None

    def camera1_callback(self, data):
        """
                Callback function of subscriber to logical camera 1 with topic : '/eyrc/vb/logical_camera_1'

                This Function checks if there is no package under the camera or not.
                if no package is under the camera and if there is one or more pending orders,
                then it moves to the package related to first order and picks it and places on conveyor belt.
                by sending the package number to server '/action/ur_5_1'

                :param data: data sent from subscriber
                :return: None
        """
        if len(self.inventory) < 12:
            rospy.logwarn("Color Identification Is Not Done")
        elif len(data.models) == 1 and self.ready_1 and len(self.orders) > 0:
            order = self.orders.pop(0)
            color_collect = order["color"]
            if color_collect == "red":
                self.next_file = self.reds.pop(0)
            elif color_collect == "yellow":
                self.next_file = self.yellows.pop(0)
            elif color_collect == "green":
                self.next_file = self.greens.pop(0)
            self.send_goal_1(self.next_file)
            self.dispatched.append(order)
            self.ready_1 = False

    def get_qr_data(self, arg_image):
        """
        Function to decode the qr code in the image given
        :param arg_image: image to decode the qr code
        :return: result of qrcode.
        """
        qr_result = decode(arg_image)

        if len(qr_result) > 0:
            return qr_result
        return 'NA'

    def camera_callback(self, data):
        """
                Callback function of subscriber to camera_1 with
                topic : '/eyrc/vb/camera_1/image_raw'

                This Function decodes the colors of all the 12 items in the shelf
                and updates the inventory list with these 12 items and then
                send goal to 'action/ros_iot/
                to upload to spread sheet.

                This function continues till the inventory list is full
                later no work is done.

                :param data: data sent from subscriber
                :return: None
        """
        if len(self.inventory) < 12:
            self.inventory = []
            self.reds = []
            self.greens = []
            self.yellows = []
            bridge = CvBridge()
            try:
                cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as error:
                rospy.logerr(error)

            image = cv_image


            try:
                count = 0
                row = 0

                for height in range(300, 751, 150):
                    col = 0
                    for width in range(50, 451, 200):
                        temp_image = image[height:height + 150, width:width + 200]
                        qr_data = self.get_qr_data(temp_image)
                        color = qr_data[0].data
                        print(row, col)
                        rospy.loginfo(color)
                        inv = {
                            "color": "",
                            "storagenum": "",
                            "SKU": ""
                        }
                        if color == "red":
                            inv["color"] = color
                            inv["storagenum"] = str(row)+str(col)
                            inv["SKU"] = "R"+inv["storagenum"] + datetime.datetime.now().strftime("%m") + datetime.datetime.now().strftime("%y")
                            if count != 10:
                                self.reds.append(count)
                            self.inventory.append(inv)
                        elif color == "yellow":
                            inv["color"] = color
                            inv["storagenum"] = str(row) + str(col)
                            inv["SKU"] = "Y" + inv["storagenum"] + datetime.datetime.now().strftime("%m")+datetime.datetime.now().strftime("%y")
                            if count != 10:
                                self.yellows.append(count)
                            self.inventory.append(inv)
                        elif color == "green":
                            inv["color"] = color
                            inv["storagenum"] = str(row) + str(col)
                            inv["SKU"] = "G" + inv["storagenum"] + datetime.datetime.now().strftime("%m")+datetime.datetime.now().strftime("%y")
                            if count != 10:
                                self.greens.append(count)
                            self.inventory.append(inv)

                        col = col + 1
                        count = count+1
                    row = row + 1

                rospy.loginfo("reds :")
                rospy.loginfo(self.reds)
                rospy.loginfo("yellows :")
                rospy.loginfo(self.yellows)
                rospy.loginfo("greens :")
                rospy.loginfo(self.greens)
                rospy.loginfo(self.inventory)
                if len(self.inventory) == 12:
                    self.send_goal_ros_iot("Inventory")
                rospy.loginfo("------")
                self.send_goal_1(-1)
                self.ready_1 = False
            except:
                rospy.logwarn(" Waiting for packages to spawn ")

            resized_image = cv2.resize(image, (720 / 2, 1280 / 2))
            cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)




def main():
    """
    main function .
    subscribes to the topics :
    instantiates Task5Interface class.
    '/eyrc/vb/camera_1/image_raw' : for knowing inventory items
    '/ros_iot_bridge/mqtt/sub' : for getting orders
    '/eyrc/vb/logical_camera_1': for dispatching
    '/eyrc/vb/logical_camera_2' : for shipping
    :return:
    """

    rospy.init_node('task5_node', anonymous=True)
    t5i = Task5Interface()

    t5i.toggle_conveyer_belt(100)
    rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, t5i.camera_callback)

    rospy.loginfo("checking for server")
    # waiting for server 2
    t5i._ac2.wait_for_server()
    rospy.loginfo("action_ur5_2 server is up, we can send new goals for ur5")
    rospy.Subscriber('/ros_iot_bridge/mqtt/sub', msgMqttSub, t5i.get_order_callback)
    rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage, t5i.camera2_callback)
    rospy.Subscriber('/eyrc/vb/logical_camera_1', LogicalCameraImage, t5i.camera1_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
