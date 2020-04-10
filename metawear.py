
from mbientlab.metawear import *
from mbientlab.metawear.cbindings import *
from datetime import *
import time
import sys

class MWBoard:
   def __init__(self):
      self.s = 0

   def returnVal(self, data):
      self.s = data
      print(self.s)
      return self.s
   
   def getData():
      reutrn self.s
      
   def stream(self):
      #I hard coded the address in, just not displaying it here
      device = MetaWear("CC:3E:36:3A:4B:50")
      device.connect()
      board = device.board
      print("connected to")
      print("Configuring device")
      libmetawear.mbl_mw_settings_set_connection_parameters(board, 7.5, 7.5, 0, 6000)
      time.sleep(1.5)
      euler_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(board, SensorFusionData.EULER_ANGLE)
      euler_callback = FnVoid_VoidP_DataP(lambda context, data: \
                                          self.returnVal(parse_value(data)))
      libmetawear.mbl_mw_datasignal_subscribe(euler_signal, None, euler_callback)
      libmetawear.mbl_mw_sensor_fusion_enable_data(board, SensorFusionData.EULER_ANGLE)
      libmetawear.mbl_mw_sensor_fusion_set_mode(board, SensorFusionMode.NDOF)
      libmetawear.mbl_mw_sensor_fusion_write_config(board)
      libmetawear.mbl_mw_sensor_fusion_start(board)
      input("")
      libmetawear.mbl_mw_sensor_fusion_stop(board)
      libmetawear.mbl_mw_sensor_fusion_clear_enabled_mask(board)
      libmetawear.mbl_mw_datasignal_unsubscribe(euler_signal)
      device.disconnect()
      time.sleep(1)

g = MWBoard()
while True:
   g.stream()
   print(g.getData())
