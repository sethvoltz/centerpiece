"""Main code for the controller box."""

import machine
import network
import ubinascii as binascii
import math
import utime
import urandom as random

from umqtt.robust import MQTTClient
from encoder import Encoder
from config import gpio_pin, led_count

machine_id = binascii.hexlify(machine.unique_id())
print(b"Machine ID: {}".format(machine_id))

sta_if = network.WLAN(network.STA_IF)
sta_if.ifconfig()
broker = sta_if.ifconfig()[2]
broker = '192.168.1.244'  # ################################################################### TEMP

pixels = [[0, 0, 0]] * led_count

strip = None
client = None

# -------------------------------------------------------------------------------------- Programs --
last_refresh = utime.ticks_ms
programs = ('white', 'candle', 'rainbow', 'twinkle', 'night', 'dance')
current_program = programs[0]
display_program = programs[0]


def run_white(force=False):
    """Run the white program."""
    if force or update_delta() > 5000:
        # slow refresh (solid color)
        set_intensity(5)
        set_hue(0)
        set_saturation(0)
        update_strip()
        update_refresh()


def run_candle(force=False):
    """Run the candle program."""
    if force or update_delta() > 1000:
        set_intensity(5)
        set_hue(60)
        set_saturation(100)
        update_strip()
        update_refresh()


def run_rainbow(force=False):
    """Run the rainbow program."""
    global rainbow_offset

    if force:
        rainbow_offset = 0.0
        set_intensity(5)
        set_saturation(100)

    if force or update_delta() > 33:
        for i in range(led_count):
            set_hue((i + rainbow_offset) * (360 / led_count) % 360, i)

        rainbow_offset = (rainbow_offset - 0.5) % led_count
        update_strip()
        update_refresh()


def run_twinkle(force=False):
    """Run the twinkle program."""
    if force or update_delta() > 1000:
        set_intensity(5)
        set_hue(180)
        set_saturation(100)
        update_strip()
        update_refresh()


def run_night(force=False):
    """Run the night program."""
    if force or update_delta() > 1000:
        set_intensity(5)
        set_hue(240)
        set_saturation(100)
        update_strip()
        update_refresh()


def run_dance(force=False):
    """Run the dance program."""
    global dance_current, dance_target, dance_offset

    if force:
        dance_current = [randint(360) for r in range(led_count)]
        dance_target = [randint(360) for r in range(led_count)]
        dance_offset = 0
        set_intensity(5)
        set_saturation(100)

    if force or update_delta() > 33:
        for i in range(led_count):
            offset = (dance_target[i] - dance_current[i]) / 33
            hue = dance_current[i] + (offset * dance_offset)
            set_hue(hue, i)

        dance_offset += 1
        if dance_offset > 33:
            dance_current = dance_target
            dance_target = [randint(360) for r in range(led_count)]
            dance_offset = 0

        update_strip()
        update_refresh()


run_programs = {
    'white': run_white,      # steady light, used to illuminate the plant
    'candle': run_candle,    # random flicker of brightness and duration, maybe tint
    'rainbow': run_rainbow,  # cycle through the color wheel at full saturation
    'twinkle': run_twinkle,  # light blue and white strobes randomly spaced
    'night': run_night,      # shades of blue and purple rolling fade, twinkle mixed in
    'dance': run_dance       # fade between random colors
}

# ------------------------------------------------------------------------------ Core Application --


def randint(max_int):
    """Simple conversion of rand available on ESP to int range."""
    return math.floor(random.getrandbits(16) / 65536 * max_int)


def update_delta():
    """Return the number of ms since the last display refresh."""
    return utime.ticks_diff(utime.ticks_ms(), last_refresh)


def update_refresh():
    """Reset the refresh timer."""
    global last_refresh
    last_refresh = utime.ticks_ms()


def callback(topic, msg):
    """MQTT callback and command router."""
    print("Received topic {}, {}".format(topic, msg))
    if match_topic(b'program'):
        set_program(msg)
    elif match_topic(b'identify'):
        publish_identity()


def publish_identity():
    """Publish this node's identity."""
    client.publish(topic_name(b"identity"), b"interface")
    print("Sent identity: {}".format(machine_id))


def topic_name(topic, all_nodes=False):
    """Construct a topic name."""
    return b'/'.join([b'centerpiece', b'all' if all_nodes else machine_id, topic])


def match_topic(topic):
    """Match a topic targeted at this client or all clients."""
    return topic == topic_name(topic) or topic == topic_name(topic, True)


def set_intensity(value, pixel=-1):
    """Set the intensity of one or all pixels."""
    global pixels
    value = value.decode("utf-8") if isinstance(value, bytes) else value
    intensity = max(0.0, min(100.0, int(value))) / 100.0

    if pixel == -1:
        pixels = [[intensity if i == 2 else v for i, v in enumerate(sub)] for sub in pixels]
    else:
        pixels[max(0, min(led_count - 1, pixel))][2] = intensity


def set_saturation(value, pixel=-1):
    """Set the saturation of one or all pixels."""
    global pixels
    value = value.decode("utf-8") if isinstance(value, bytes) else value
    saturation = max(0.0, min(100.0, float(value))) / 100.0

    if pixel == -1:
        pixels = [[saturation if i == 1 else v for i, v in enumerate(sub)] for sub in pixels]
    else:
        pixels[max(0, min(led_count - 1, pixel))][1] = saturation


def set_hue(value, pixel=-1):
    """Set the hue of one or all pixels."""
    global pixels
    value = value.decode("utf-8") if isinstance(value, bytes) else value
    hue = max(0.0, min(360.0, float(value))) / 360.0

    if pixel == -1:
        pixels = [[hue if i == 0 else v for i, v in enumerate(sub)] for sub in pixels]
    else:
        pixels[max(0, min(led_count - 1, pixel))][0] = hue


def update_strip():
    """Update the neopixel strip."""
    if strip is None:
        print("Strip hasn't been configured yet, can't update.")
        return

    for index, color in enumerate(pixels):
        r, g, b, w = hsi_to_rgbw(color[0], color[1], color[2])
        r, g, b, w = int(r * 255), int(g * 255), int(b * 255), int(w * 255)
        strip[index] = (r, g, b, w)

    strip.write()


def connect_and_subscribe():
    """Connect to the MQTT server and subscribe to topics."""
    global client

    client = MQTTClient(machine_id, broker)
    client.set_callback(callback)
    client.connect()

    print("Connected to {}".format(broker))

    for topic in (b'program', b'identify'):
        subscribe(topic)
        subscribe(topic, all_nodes=True)


def subscribe(topic, all_nodes=False):
    """Subscribe to MQTT topics."""
    t = topic_name(topic, all_nodes)
    client.subscribe(t)
    print("Subscribed to {}".format(t))


def set_display(program_name):
    """Set the current program to run on the display."""
    global display_program
    display_program = program_name
    run_display(True)  # force first frame run


def run_display(force=False):
    """Run the display function for the currently selected display."""
    run_programs[display_program](force)


def setup_neopixels(pin, count):
    """Set up the neopixels."""
    global strip
    import neopixel
    strip = neopixel.NeoPixel(machine.Pin(pin), count, 4)
    update_strip()


def setup_encoder():
    """Initialize the rotary encoder."""
    global rotary, rotary_lastval

    rotary = Encoder(
        pin_clk=12, pin_dt=14, pin_mode=machine.Pin.PULL_UP,
        clicks=4, wrap=True, min_val=0, max_val=len(programs))
    rotary_lastval = rotary.value


def check_encoder():
    """Check for change in the rotary encoder position."""
    global rotary, rotary_lastval, display_program

    val = rotary.value
    if rotary_lastval != val:
        rotary_lastval = val
        print(val)
        set_display(programs[val])


def setup():
    """Startup initialization function."""
    setup_neopixels(gpio_pin, led_count)
    connect_and_subscribe()
    setup_encoder()

    # Let the network know we're online
    publish_identity()


def main_loop():
    """Microcontroller main loop."""
    while 1:
        client.check_msg()
        check_encoder()
        run_display()


def teardown():
    """Shutdown cleanup."""
    rotary.close()

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
