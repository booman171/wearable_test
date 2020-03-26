# usage: python stream_acc.py [mac1] [mac2] ... [mac(n)]
from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from time import sleep
from threading import Event
from ctypes import byref, create_string_buffer
import platform
import sys

if sys.version_info[0] == 2:
    range = xrange

class State:
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.callback = FnVoid_VoidP_DataP(self.data_handler)

    def data_handler(self, ctx, data):
        print("%s -> %s" % (self.device.address, parse_value(data)))
        self.samples+= 1

states = []
d = MetaWear("CC:3E:36:3A:4B:50")
d.connect()
print("Connected to " + d.address)
states.append(State(d))

for s in states:
    print("Configuring device")
    libmetawear.mbl_mw_settings_set_connection_parameters(s.device.board, 7.5, 7.5, 0, 6000)
    sleep(1.5)
    an_signal = libmetawear.mbl_mw_gpio_get_analog_input_data_signal(s.device.board, 2, GpioAnalogReadMode.ADC)
    libmetawear.mbl_mw_datasignal_subscribe(an_signal, None, s.callback)
    libmetawear.mbl_mw_gpio_start_pin_monitoring(s.device.board, 2)
    libmetawear.mbl_mw_datasignal_read(an_signal)

sleep(15.0)

for s in states:
    libmetawear.mbl_mw_gpio_stop_pin_monitoring(s.device.board, 2)
    an_signal = libmetawear.mbl_mw_gpio_get_analog_input_data_signal(s.device.board, 2, GpioAnalogReadMode.ADC)
    libmetawear.mbl_mw_datasignal_unsubscribe(an_signal)
    libmetawear.mbl_mw_debug_disconnect(s.device.board)
    
print("Total Samples Received")
for s in states:
    print("%s -> %d" % (s.device.address, s.samples))