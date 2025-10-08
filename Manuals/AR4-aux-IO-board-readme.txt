A secondary Arduino board is used to handle peripheral IO for devices such as grippers.  The arduino boards operate at 5v and are better suited for handling IO and relays (the teensy board only handles robot motion)

Please see this video https://youtu.be/76F6dS4ar8Y

If using a Nano board the IO are as follows:
Inputs  = Pins 2 through 7
Outputs = Pins 8 through 13
Servo   = Pins A0 through A7

If using a Mega board the IO are as follows:
Inputs  = Pins 0 through 27
Outputs = Pins 28 through 53
Servo   = Pins A0 through A7