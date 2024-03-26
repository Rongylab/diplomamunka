# Reports the state of a joysticks axes and buttons.
# std_msgs/Header header  # timestamp in the header is the time the data is received from the joystick
float32[8] axes          # the axes measurements from a joystick
int32[11] buttons         # the buttons measurements from a joystick
uint8 id		# the destination id of the veichle
