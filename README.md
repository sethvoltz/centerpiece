# Centerpiece Light Control

## Controller and Modes

A quick test with one node shows that it's possible to get a comfortable 10 messages per second and a spotty 20-30 messages per second throughput. This suggests that node control should primarily be to set the current mode, and not to control every change. Responsiveness seems fairly good, however, so doing sound-reactive beats might still be a possibility.

### Current Mode Ideas

- White -- steady light, used to illuminate the plant
- Candle flicker -- random flicker of brightness and duration, maybe tint
- Rainbow fade -- cycle through the color wheel at full saturation
- Synchronized rainbow fade -- same as rainbow fade but all nodes syncronized
- Twinkle -- light blue and white strobes randomly spaced
- Night sky -- shades of blue and purple rolling fade, twinkle mixed in
- Manual color -- rotary encoder to send single HSI to all nodes
- Dance mode -- fade between random colors with rotary BPM select
- Synchronized dance mode -- same as dance mode but all nodes syncronized

### Control Commands

Commands are sent to nodes via MQTT with nodes listening on both individual and mass-control topics. Mass-control topics are `light/all/control` and individual is `light/<nodeid>/control`. Commands are specified as a command name and value separated with a colon (`:`). Command values may have multiple parts, with simple values being comma separated and complex being JSON payloads.

- hsi -- set the full color value, components are comma separated
- h -- set the color hue, value is a float 0-360
- s -- set the color saturation, value is a float 0-100
- i -- set the color intensity, value is a float 0-100
- power -- turn the light on or off, value is `on` or `off`
- mode -- enable a well-known color changing mode
- fade -- fade from current to given HSI color over a given time

### NTP-style Time Synchronization

Since different nodes will come online at different times, there's no perfect way to get each one synchronized with all the others for precise timing needs. However, taking inspiration from the Network Time Protocol, we can get close by setting up a time value which is then stored as an offset of the device's `millis()` value. The node performs a time sync on it's own or at the direction of the controller through the `sync` topic.

From [this stack overflow][so-ntp-answer] answer, the protocol is basically this:

- Client sends request at "wrong" time 100. A=100.
- Server receives request at "true" time 150. X=150.
- The server is slow, so it doesn't send out the response until "true" time 160. Y=160.
- The client receives the request at "wrong" time 120. B=120.
- Client determines the time spend on the network is B-A-(Y-X)=120-100-(160-150)=10 seconds
- Client assumes the amount of time it took for the response to get from the server to the client is 10/2=5 seconds.
- Client adds that time to the "true" time when the server sent the response to estimate that it received the response at "true" time 165 seconds.
- Client now knows that it needs to add 45 seconds to its clock.

This would be modified to take the "true" time, in millis since the UNIX epoc, and calculate an offset based on the device millis since boot, the local epoc.

[so-ntp-answer]: http://stackoverflow.com/a/1230826/772207

## Notes

Example config pub:
`mosquitto_pub -h localhost -t "light/abdec600/config" -m '{"hue":0,"saturation":100,"intensity":5,"power":"on","gpio_pin":15,"led_count":1}'`

Example update pub:
`mosquitto_pub -h localhost -t "light/abdec600/control" -m 'h:120'`

Connect to serial REPL: `sudo cu -l /dev/tty.SLAB_USBtoUART -s 115200`

Load Code: `ampy --port /dev/tty.SLAB_USBtoUART put main.py`

Get a list of currently connected clients by publishing to `light/identify` and looking at the topics that return.

Current time with nanoseconds and millis on Unix: `date +%s%N`, `echo $(($(date +%s%N)/1000000))`
