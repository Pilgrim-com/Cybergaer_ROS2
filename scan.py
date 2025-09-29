#!/usr/bin/python3
#
import os, time,can
from cybergear import *
bus = can.Bus(interface= "canalystii",channel= 0, bitrate=1000000)
cg = CyberGear(bus)
for id in range(0, 0x80):
  if cg.type0(id) != None:
    print(id, ':Find')
cg.shutdown()
