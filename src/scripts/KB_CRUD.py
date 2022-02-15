from rosplan_knowledge_msgs.srv import GetDomainAttributeService, GetDomainAttributeServiceResponse, GetAttributeService, GetAttributeServiceResponse
import rospy

class KnowledgeBaseNode():
  def __init__(self):
    rospy.loginfo("Waiting for service")
    basePath = """/rosplan_knowledge_base"""
    self.servicePaths = {
      "predicates": f"{basePath}/domain/predicates",
      "functions": f"{basePath}/domain/functions",
      "propositions": f"{basePath}/state/propositions"
    }
    self._setUp()
  
  def _setUp(self):
    rospy.wait_for_service(self.servicePaths["predicates"])
    rospy.wait_for_service(self.servicePaths["functions"])
    rospy.wait_for_service(self.servicePaths["propositions"])
    try:
      self.getKBPredicates = rospy.ServiceProxy(self.servicePaths["predicates"], GetDomainAttributeService)
      self.getKBNumPredicates = rospy.ServiceProxy(self.servicePaths["functions"], GetDomainAttributeService)
      self.getKBPropositions = rospy.ServiceProxy(self.servicePaths["propositions"], GetAttributeService)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')
    
  def getPredicates(self):
    resp = self.getKBPredicates()
    return self._parseResponse(resp)

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
      result[crntProp.attribute_name].append(entry)
    return result

  def getNumPredicates(self):
    resp = self.getKBNumPredicates()
    return self._parseResponse(resp)

  def getPropositions(self):
    resp = self.getKBPropositions()
    return self._parseResponse(resp)

if __name__ == '__main__':
  x = KnowledgeBaseNode()
  print(f'predicates:\n{x.getPredicates()}')
  print(f'functions:\n{x.getNumPredicates()}')
  print(f'propositions:\n{x.getPropositions()}')
