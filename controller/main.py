import time
import paho.mqtt.client as mqtt

client = mqtt.Client()


def connect():
    global client
    client.connect('localhost')
    client.loop_start()


def on_connect(client, userdata, code):
    print("Connected with result code {}".format(code))
    client.subscribe('light/+/state')


def on_message(client, userdata, message):
    print("message: {} - {}".format(message.topicm, message.payload))


def rainbow(loops=2, delay=0.033):
    num = 0
    while num < 360 * loops:
        client.publish("light/abdec600/control", "hsi:{}:{}:".format(num % 360, (num / 2 % 50) + 50))
        num += 10
        time.sleep(delay)

connect()
