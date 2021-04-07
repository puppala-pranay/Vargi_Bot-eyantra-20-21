#!/usr/bin/env python
"""
ros_iot_bridge which deals with everything related to
mqtt and google spreadsheet
"""


# ROS Node - Action Server - IoT ROS Bridge

import threading
import json
import rospy
import actionlib
from pkg_ros_iot_bridge.msg import msgRosIotAction  # Message Class used by ROS Actions internally
# from pkg_ros_iot_bridge.msg import msgRosIotGoal  # Message Class used for Goal Messages
# from pkg_ros_iot_bridge.msg import msgRosIotResult  # Message Class used for Result Messages
# from pkg_ros_iot_bridge.msg import msgRosIotFeedback  # Message Class used for Feedback Messages
from pkg_ros_iot_bridge.msg import msgMqttSub  # Message Class for MQTT Subscription Messages
from pyiot import iot  # Custom Python Module to perform MQTT Tasks


class IotRosBridgeActionServer:
    """
    Class used to upload the data to spreadsheet
    and mqtt
    """

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the function pointer which points to a
             function which will be called
             when the Action Server receives a Goal.

            * self.on_cancel - It is the function pointer which points to a
             function which will be called
             when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic =\
            param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_apps_spread_sheet_id = \
            param_config_iot['google_apps']['spread_sheet_id']
        self._config_submission_google_apps_spread_sheet_id = \
            param_config_iot['google_apps']['submission_spread_sheet_id']
        print(param_config_iot)

        self.url = "https://script.google.com/macros/s/" + \
                   self._config_google_apps_spread_sheet_id+"/exec"
        self.submission_url = "https://script.google.com/macros/s/" + \
                              self._config_submission_google_apps_spread_sheet_id+"/exec"
        # self.url = self.submission_url

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                               msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/pLmoKnBJ/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        """
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback, self._config_mqtt_server_url,
                                              self._config_mqtt_server_port, self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        """

        ret1 = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                               self._config_mqtt_server_url,
                                               self._config_mqtt_server_port,
                                               "/eyrc/vb/pLmoKnBJ/orders",
                                               self._config_mqtt_qos)
        if ret1 == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        """
        Callback function of subscription to mqtt topic '/eyrc/vb/pLmoKnBJ/orders'
        gets the message from mqtt topic and publishes on to '/ros_iot_bridge/mqtt/sub'
        :param client: mqtt client
        :param userdata: userdata
        :param message: message from subscription
        :return: none
        """
        payload = json.loads(message.payload)

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.city = payload["city"]
        msg_mqtt_sub.date_time1 = payload["order_time"]
        msg_mqtt_sub.order_id = payload["order_id"]
        msg_mqtt_sub.lon = payload["lon"]
        msg_mqtt_sub.qty = payload["qty"]
        msg_mqtt_sub.item = payload["item"]
        msg_mqtt_sub.lat = payload["lat"]

        self._handle_ros_pub.publish(msg_mqtt_sub)

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        """
        Function call back when goal is received from client
        :param goal_handle: handle for the goal to process it
        :return: None
        """
        # goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client by /action_ros_iot ")

        thread = threading.Thread(name="worker",
                                  target=self.process_goal,
                                  args=(goal_handle,))
        thread.start()

    # Function to publish to mqtt topic
    def publish_to_iot(self, server_url, server_port,
                       topic, message, qos):
        """
        Publish a given message to mqtt client
        :param server_url: mqtt client url
        :param server_port: port of mqtt client
        :param topic: mqtt topic for publishing
        :param message: message that should be published
        :param qos: quality
        :return: None
        """
        ret = iot.mqtt_publish(server_url, server_port,
                               topic, message, qos)

        if ret == 0:
            rospy.loginfo("MQTT Publish Successful.")

        else:
            rospy.logerr("MQTT Failed to Publish")

    # Function to upload as many data points as given to given spread sheet
    def upload_to_spread_sheet(self, url, sheet, entries):
        """
        Function to upload to google spreadsheet
        :param url: url of spreadsheet
        :param sheet: sheet name
        :param entries: array of data to be published
        :return: None
        """
        if sheet == "Inventory":
            for entry in entries:
                params = {
                    "id": sheet,
                    "Team Id": "VB#1653",
                    "Unique Id": "pLmoKnBJ",
                    "SKU": entry.SKU,
                    "Item": entry.item,
                    "Priority": entry.priority,
                    "Storage Number": entry.storage,
                    "Quantity": entry.qty,
                    "Cost": entry.cost

                }
                response = iot.spread_sheet_upload(url, params)
                #response = iot.spread_sheet_upload(self.submission_url, params)
                rospy.loginfo(response.content)
                # message = ""
                # self.publish_to_iot(self._config_mqtt_server_url, self._config_mqtt_server_port,
                #                    self._config_mqtt_pub_topic,
                #                    message, self._config_mqtt_qos)
        elif sheet == "IncomingOrders":
            for entry in entries:
                params = {
                    "id": sheet,
                    "Team Id": "VB#1653",
                    "Unique Id": "pLmoKnBJ",
                    "Order ID": entry.order_id,
                    "Order Date and Time":  entry.date_time1,
                    "Item": entry.item,
                    "Priority": entry.priority,
                    "Order Quantity": entry.qty,
                    "City": entry.city,
                    "Longitude": entry.lon,
                    "Latitude": entry.lat,
                    "Cost": entry.cost

                }
                response = iot.spread_sheet_upload(url, params)
                #response = iot.spread_sheet_upload(self.submission_url, params)
                rospy.loginfo(response.content)
                # message = ""
                # self.publish_to_iot(self._config_mqtt_server_url, self._config_mqtt_server_port,
                #                    self._config_mqtt_pub_topic,
                #                    message, self._config_mqtt_qos)
        elif sheet == "OrdersDispatched":
            for entry in entries:
                params = {
                    "id": sheet,
                    "Team Id": "VB#1653",
                    "Unique Id": "pLmoKnBJ",
                    "Order ID": entry.order_id,
                    "Dispatch Date and Time": entry.date_time1,
                    "Item": entry.item,
                    "Priority": entry.priority,
                    "Dispatch Quantity": entry.qty,
                    "City": entry.city,
                    "Cost": entry.cost,
                    "Dispatch Status": entry.status

                }
                response = iot.spread_sheet_upload(url, params)
                #response = iot.spread_sheet_upload(self.submission_url, params)
                rospy.loginfo(response.content)
                # message = ""
                # self.publish_to_iot(self._config_mqtt_server_url, self._config_mqtt_server_port,
                #                    self._config_mqtt_pub_topic,
                #                    message, self._config_mqtt_qos)

        elif sheet == "OrdersShipped":
            for entry in entries:
                params = {
                    "id": sheet,
                    "Team Id": "VB#1653",
                    "Unique Id": "pLmoKnBJ",
                    "Order ID": entry.order_id,
                    "Shipped Date and Time": entry.date_time1,
                    "Item": entry.item,
                    "Priority": entry.priority,
                    "Shipped Quantity": entry.qty,
                    "City": entry.city,
                    "Cost": entry.cost,
                    "Shipped Status": entry.status,
                    "Estimated Time of Delivery": entry.date_time2

                }
                response = iot.spread_sheet_upload(url, params)
                #response = iot.spread_sheet_upload(self.submission_url, params)
                rospy.loginfo(response.content)
                # message = ""
                # self.publish_to_iot(self._config_mqtt_server_url, self._config_mqtt_server_port,
                #                    self._config_mqtt_pub_topic,
                #                    message, self._config_mqtt_qos)

    # Function to process the incoming goal
    def process_goal(self, goal_handle):
        """
        Function to process the goal
        uploads the data in goal to spreadsheets and publish to mqtt client
        :param goal_handle: goal handle
        :return:
        """
        # result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        self.upload_to_spread_sheet(self.url, goal.sheet, goal.entries)

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        """
        Callback function on cancelling the goal
        :param goal_handle: handle to goal cancelled
        :return:
        """

        rospy.loginfo("Received cancel request.")
        # goal_id = goal_handle.get_goal_id()


# Main
def main():
    """
    Main Function
    Initiates the node
    Instantiates the IotRosBridgeActionServer class
    :return: None
    """
    rospy.init_node('node_iot_ros_bridge_action_server')  # initiate node
    IotRosBridgeActionServer()  # instantiate server
    rospy.spin()


if __name__ == '__main__':
    main()
