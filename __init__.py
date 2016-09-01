"""Python tools for working with the Crust Crawler robots.

This module has some tools to start using the Crust Crawler pro-Series arms
found in the Automation and Control labs at Aalborg University.

These tools are made to be used with a RS485 adapter or a UartSBee connected
to a RS485 driver in one end and the pc in the other (not with an XBee).

The DTR pin on the UartSBee works as the driver/receiver enable, so be sure
to hook this up too when you are hooking rx, tx, vcc, and gnd up.

"""

from .dynamixel_mx_driver import dynamixel_mx