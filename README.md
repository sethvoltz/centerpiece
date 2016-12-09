# Centerpiece Light Control

## Notes

Example config pub:
`mosquitto_pub -h localhost -t "light/abdec600/config" -m '{"hue":0,"saturation":100,"intensity":5,"power":"on","gpio_pin":15,"led_count":1}'`

Example update pub:
`mosquitto_pub -h localhost -t "light/abdec600/control" -m 'h:120'`

Connect to serial REPL: `sudo cu -l /dev/tty.SLAB_USBtoUART -s 115200`

Load Code: `ampy --port /dev/tty.SLAB_USBtoUART put main.py`

