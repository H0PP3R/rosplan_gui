#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from service_node.srv import *

def getKBStateClient(x,y):
  rospy.wait_for_service('getKBState')
  try:
    getKBState = rospy.ServiceProxy('getKBState', getKBState)
    resp1 = getKBState(x, y)
    return resp1.sum
  except rospy.ServiceException as e:
    print(f'Service call failed: {e}')

def usage():
  return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 3:
    x = int(sys.argv[1])
    y = int(sys.argv[2])
  else:
    print(usage())
    sys.exit(1)
  print(f'Requesting {x}+{y}')
  print(f'{x} + {y} = {getKBStateClient(x,y)}')