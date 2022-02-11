#!/usr/bin/env python

from __future__ import print_function
import sys
# sys.path.append('/home/rabeden/PRJ/ROSPlan/src/rosplan')
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
import rospy
# from prettytable import PrettyTable

class KnowledgeBaseNode():
  def __init__(self):
    rospy.loginfo("Waiting for service")
    rospy.wait_for_service('/rosplan_knowledge_base/domain/predicates')
    try:
      self.getKBState = rospy.ServiceProxy('/rosplan_knowledge_base/domain/predicates', GetDomainAttributeService)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')
    
  def getPredicates(self):
    resp = self.getKBState().items
    return self._parsePredicateResponse(resp)

  def _parsePredicateResponse(self, resp):
    result = {}
    for i in range(len(resp)):
      crntPredicate = resp[i]
      for j in range(len(resp[i].typed_parameters)): 
        item = crntPredicate.typed_parameters[j]
        if crntPredicate.name not in result.keys():
          result[crntPredicate.name] = {}
        result[crntPredicate.name][item.key] = item.value
    return result