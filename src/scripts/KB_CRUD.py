#!/usr/bin/env python

from __future__ import print_function
import sys
sys.path.append('/home/rabeden/PRJ/ROSPlan/src/rosplan')
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
# from rosplan.rosplan_knowledge_msgs.msg import DomainFormula
# import rosplan
import rospy

class KnowledgeBaseNode():
  def __init__(self):
    rospy.loginfo("Waiting for service")
    rospy.wait_for_service('rosplan_knowledge_base')
    self.getKBStateClient()

  def getKBStateClient(self):
    try:
      getKBState = rospy.ServiceProxy('/rosplan_knowledge_base/domain/predicates', None)
      return getKBState
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')

if __name__ == "__main__":
  print(f'predicates = {KnowledgeBaseNode.getKBStateClient()}')