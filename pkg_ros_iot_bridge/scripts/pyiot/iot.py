"""
Module to operate iot operations.
"""
import time
import requests
import paho.mqtt.client as mqtt  # import the client1



class print_colour:
    """
    class to print color
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    """
    call back function for subscription of mqtt topic
    :param client: mqtt client
    :param userdata: userdata
    :param message: message received
    :return:
    """
    print("message received ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)


def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url,
                                arg_broker_port, arg_mqtt_topic,
                                arg_mqtt_qos):
    """
    function to start a subscribe thread to a given topic
    :param arg_callback_func: callback function reference
    :param arg_broker_url: mqtt url
    :param arg_broker_port: mqtt port
    :param arg_mqtt_topic: topic to be subscribed
    :param arg_mqtt_qos: qos
    :return: 0 if success else -1
    """
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1)  # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()  # starts a new thread
        return 0
    except ValueError:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    """
    Function to publish to a given mqtt topic
    :param arg_broker_url: mqtt url
    :param arg_broker_port: mqtt port
    :param arg_mqtt_topic: mqtt topic
    :param arg_mqtt_message: message to publish
    :param arg_mqtt_qos: qos
    :return:
    """
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1)  # wait

        mqtt_client.loop_stop()  # stop the loop
        return 0
    except ValueError:
        return -1


def spread_sheet_upload(url, parameters):
    """
    Function to upload to the sheet given
    :param url:
    :param parameters:
    :return:
    """
    response = requests.get(url, params=parameters)
    return response
