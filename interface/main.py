"""Main code for the controller box."""

import machine
import network
import ubinascii as binascii
import math

from umqtt.simple import MQTTClient

machine_id = binascii.hexlify(machine.unique_id())
print(b"Machine ID: {}".format(machine_id))

sta_if = network.WLAN(network.STA_IF)
sta_if.ifconfig()
broker = sta_if.ifconfig()[2]

hues = []
saturations = []
intensities = []
pixel_count = 0
powered_on = False

strip = None
client = None


def callback(topic, msg):
    """MQTT callback and command router."""
    if topic == topic_name(b"program") or topic == topic_name(b"program", True):
        set_program(msg)
    elif topic == topic_name(b"config") or topic_name(b"config", True):
        load_config(msg)
    elif topic == topic_name(b'identify') or topic_name(b'identify', True):
        publish_identity()


def publish_identity():
    """Publish this node's identity."""
    client.publish(topic_name(b"identity"), b"interface")
    print("Sent identity: {}".format(machine_id))


def topic_name(topic, all_nodes=False):
    """Construct a topic name."""
    return b'/'.join([b'centerpiece', b'all' if all_nodes else machine_id, topic])


def set_intensity(value, pixel=-1):
    """Set the intensity of one or all pixels."""
    global intensities
    value = value.decode("utf-8") if isinstance(value, bytes) else value
    intensity = max(0.0, min(100.0, int(value))) / 100.0
    if pixel == -1:
        intensities = [intensity] * pixel_count
    else:
        intensities[max(pixel_count - 1, min(0, pixel))] = intensity
    update_strip()


def set_saturation(value, pixel=-1):
    """Set the saturation of one or all pixels."""
    global saturations
    value = value.decode("utf-8") if isinstance(value, bytes) else value
    saturation = max(0.0, min(100.0, float(value))) / 100.0
    if pixel == -1:
        saturations = [saturation] * pixel_count
    else:
        saturations[max(pixel_count - 1, min(0, pixel))] = saturation
    update_strip()


def set_hue(value=-1):
    """Set the hue of one or all pixels."""
    global hues
    value = value.decode("utf-8") if isinstance(value, bytes) else value
    hue = max(0.0, min(360.0, float(value))) / 360.0
    if pixel == -1:
        hues = [hue] * pixel_count
    else:
        hues[max(pixel_count - 1, min(0, pixel))] = hue
    update_strip()


def set_power(msg):
    """Turn the pixels on or off."""
    global powered_on
    msg = msg.decode("utf-8") if isinstance(msg, bytes) else msg
    powered_on = msg == "on"
    update_strip()


def update_strip():
    """Update the neopixel strip."""
    if strip is None:
        print("Strip hasn't been configured yet, can't update.")
        return
    if powered_on:
        r, g, b, w = hsi_to_rgbw(hue, saturation, intensity)
        r, g, b, w = int(r * 255), int(g * 255), int(b * 255), int(w * 255)
        strip.fill((r, g, b, w))
    else:
        strip.fill((0, 0, 0, 0))
    strip.write()


def connect_and_subscribe():
    """Connect to the MQTT server and subscribe to topics."""
    global client

    client = MQTTClient(machine_id, broker)
    client.set_callback(callback)
    client.connect()

    print("Connected to {}".format(broker))

    for topic in (b'config', b'program', b'identify'):
        subscribe(topic)
        subscribe(topic, all_nodes=True)


def subscribe(topic, all_nodes=False):
    """Subscribe to MQTT topics."""
    t = topic_name(topic, all_nodes)
    client.subscribe(t)
    print("Subscribed to {}".format(t))


def setup_neopixels(pin, count):
    """Set up the neopixels."""
    global strip
    import neopixel
    strip = neopixel.NeoPixel(machine.Pin(pin), count, 4)
    update_strip()


def load_config(msg):
    """Load an initializing config"""
    print("Loading config: {}".format(msg))

    import ujson as json
    try:
        config = json.loads(msg)
    except (OSError, ValueError):
        print("Couldn't load config from JSON, bailing out.")
    else:
        global pixel_count
        pixel_count = config['led_count']

        set_hue(config['hue'])
        set_saturation(config['saturation'])
        set_intensity(config['intensity'])
        set_power(config['power'])
        setup_neopixels(config['gpio_pin'], config['led_count'])


def setup():
    """Startup initialization function."""
    connect_and_subscribe()


def main_loop():
    """Microcontroller main loop."""
    while 1:
        client.wait_msg()


def teardown():
    """Shutdown cleanup."""
    try:
        client.disconnect()
        print("Disconnected.")
    except Exception:
        print("Couldn't disconnect cleanly.")


def hsi_to_rgbw(h, s, i):
    """Convert HSI-formatted colors to RGBW for the neopixels."""
    h = 3.14159 * h * 2.0  # Convert unit to radians

    if h < 2.09439:
        cos_h = math.cos(h)
        cos_1047_h = math.cos(1.047196667 - h)
        r = s * i / 3 * (1 + cos_h / cos_1047_h)
        g = s * i / 3 * (1 + (1 - cos_h / cos_1047_h))
        b = 0
        w = (1 - s) * i

    elif h < 4.188787:
        h = h - 2.09439
        cos_h = math.cos(h)
        cos_1047_h = math.cos(1.047196667 - h)
        g = s * i / 3 * (1 + cos_h / cos_1047_h)
        b = s * i / 3 * (1 + (1 - cos_h / cos_1047_h))
        r = 0
        w = (1 - s) * i

    else:
        h = h - 4.188787
        cos_h = math.cos(h)
        cos_1047_h = math.cos(1.047196667 - h)
        b = s * i / 3 * (1 + cos_h / cos_1047_h)
        r = s * i / 3 * (1 + (1 - cos_h / cos_1047_h))
        g = 0
        w = (1 - s) * i

    return r, g, b, w


if __name__ == '__main__':
    setup()
    try:
        main_loop()
    finally:
        teardown()
