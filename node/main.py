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

hue = 0.0
saturation = 0.0
intensity = 0.0
powered_on = False

strip = None
client = None


def callback(topic, msg):
    if topic == topic_name(b"control"):
        try:
            msg_type, payload = msg.split(b":", 1)
            if msg_type == b"h":
                set_hue(payload)
            elif msg_type == b"s":
                set_saturation(payload)
            elif msg_type == b"i":
                set_intensity(payload)
            elif msg_type == b"hsi":
                h, s, i = payload.split(b":")

                if h:
                    set_hue(h)
                if s:
                    set_saturation(s)
                if i:
                    set_intensity(i)
            elif msg_type == b"power":
                set_power(payload)
            else:
                print("Unknown message type, ignoring")
        except Exception:
            print("Couldn't parse/handle message, ignoring.")
    elif topic == topic_name(b"config"):
        load_config(msg)
    elif topic == b'light/identify':
        publish_state()


def publish_state():
    global powered_on
    if powered_on:
        client.publish(topic_name(b"state"), b"on")
    else:
        client.publish(topic_name(b"state"), b"off")

    print("Light state: {}".format("on" if powered_on else "off"))


def topic_name(topic):
    return b"/".join([b"light", machine_id, topic])


def set_intensity(msg):
    global intensity
    intensity = max(0.0, min(100.0, int(msg))) / 100.0
    update_strip()


def set_saturation(msg):
    global saturation
    msg = msg.decode("utf-8") if isinstance(msg, bytes) else msg
    saturation = max(0.0, min(100.0, float(msg))) / 100.0
    update_strip()


def set_hue(msg):
    global hue
    msg = msg.decode("utf-8") if isinstance(msg, bytes) else msg
    hue = max(0.0, min(360.0, float(msg))) / 360.0
    update_strip()


def set_power(msg):
    global powered_on
    msg = msg.decode("utf-8") if isinstance(msg, bytes) else msg
    powered_on = msg == "on"
    publish_state()
    update_strip()


def update_strip():
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
    global client

    client = MQTTClient(machine_id, broker)
    client.set_callback(callback)
    client.connect()

    print("Connected to {}".format(broker))

    for topic in (b'config', b'control'):
        t = topic_name(topic)
        client.subscribe(t)
        print("Subscribed to {}".format(t))

    client.subscribe(b'light/identify')


def setup_neopixels(pin, count):
    global strip
    import neopixel
    strip = neopixel.NeoPixel(machine.Pin(pin), count, 4)
    update_strip()


def load_config(msg):
    print("Loading config: {}".format(msg))

    import ujson as json
    try:
        config = json.loads(msg)
    except (OSError, ValueError):
        print("Couldn't load config from JSON, bailing out.")
    else:
        set_hue(config['hue'])
        set_saturation(config['saturation'])
        set_intensity(config['intensity'])
        set_power(config['power'])
        setup_neopixels(config['gpio_pin'], config['led_count'])


def setup():
    connect_and_subscribe()


def main_loop():
    while 1:
        client.wait_msg()


def teardown():
    try:
        client.disconnect()
        print("Disconnected.")
    except Exception:
        print("Couldn't disconnect cleanly.")


def hsi_to_rgbw(h, s, i):
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
