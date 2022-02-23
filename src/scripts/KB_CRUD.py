from rosplan_knowledge_msgs.srv import GetDomainAttributeService, GetDomainAttributeServiceResponse, GetAttributeService, GetAttributeServiceResponse, KnowledgeQueryService
from rosplan_knowledge_msgs.msg import KnowledgeItem, diagnostic_msgs
import rospy

class KnowledgeBaseNode():
  def __init__(self):
    rospy.loginfo("Waiting for service")
    basePath = """/rosplan_knowledge_base"""
    self.servicePaths = {
      "predicates": f"{basePath}/domain/predicates",
      "functions": f"{basePath}/domain/functions",
      "propositions": f"{basePath}/state/propositions",
      "numericFluents": f"{basePath}/state/functions"
    }
    self._setUp()
  
  def _setUp(self):
    rospy.wait_for_service(self.servicePaths["predicates"])
    rospy.wait_for_service(self.servicePaths["functions"])
    rospy.wait_for_service(self.servicePaths["propositions"])
    rospy.wait_for_service(self.servicePaths["numericFluents"])
    try:
      self._getKBPredicates = rospy.ServiceProxy(self.servicePaths["predicates"], GetDomainAttributeService)
      self._getKBNumPredicates = rospy.ServiceProxy(self.servicePaths["functions"], GetDomainAttributeService)
      self._getKBPropositions = rospy.ServiceProxy(self.servicePaths["propositions"], GetAttributeService)
      self._getKBCurrentNumPropositions = rospy.ServiceProxy(self.servicePaths["numericFluents"], GetAttributeService)
      # self.statusListener = StatusUpdateListener()
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')

  def _parseResponse(self, resp):
    result = {}
    if isinstance(resp, GetDomainAttributeServiceResponse):
      result = self._parseGetDomainAttributeServiceResponse(resp.items)
    elif isinstance(resp, GetAttributeServiceResponse):
      result = self._parseGetAttributeServiceResponse(resp.attributes)
    return result
  
  def _parseGetDomainAttributeServiceResponse(self, resp):
    result = {}
    for crntPredicate in resp:
      for item in crntPredicate.typed_parameters: 
        if crntPredicate.name not in result.keys():
          result[crntPredicate.name] = {}
        result[crntPredicate.name][item.key] = item.value
    return result
  
  def _parseGetAttributeServiceResponse(self, resp):
    result = {}
    t0 = resp[0].initial_time.secs
    for crntProp in resp:
      crntKeys = list(result.keys())
      if crntProp.attribute_name not in crntKeys:
        result[crntProp.attribute_name] = []
      entry = [crntProp.initial_time.secs - t0]
      for item in crntProp.values:
        entry.append(item.value)
      # Add numeric value to entry if crntProp is a numeric fluent
      if crntProp.knowledge_type == 2:
        entry.append(crntProp.function_value)
      entry.append(crntProp.is_negative)
      result[crntProp.attribute_name].append(entry)
    return result

  def getPredicates(self):
    self.predicates = self._parseResponse(self._getKBPredicates())
    return self.predicates

  def getNumPredicates(self):
    self.numPredicates = self._parseResponse(self._getKBNumPredicates())
    return self.numPredicates

  def getPropositions(self):
    self.propositions = self._parseResponse(self._getKBPropositions())
    return self.propositions
  
  def getCurrentNumPropositions(self):
    self.getCurrentNumPropositions = self._parseResponse(self._getKBCurrentNumPropositions())
    return self.getCurrentNumPropositions

if __name__ == '__main__':
  x = KnowledgeBaseNode()
  print(f'predicates:\n{x.getPredicates()}')
  print(f'functions:\n{x.getNumPredicates()}')
  print(f'propositions:\n{x.getPropositions()}')
  print(f'current numeric fluents:\n{x.getCurrentNumPropositions()}')
