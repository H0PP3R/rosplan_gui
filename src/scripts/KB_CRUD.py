from rosplan_knowledge_msgs.srv import GetDomainAttributeService, GetAttributeService
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
    rospy.wait_for_service(self.servicePaths["predicates"])
    rospy.wait_for_service(self.servicePaths["functions"])
    try:
      self.getKBPredicates = rospy.ServiceProxy(self.servicePaths["predicates"], GetDomainAttributeService)
      self.getKBNumPredicates = rospy.ServiceProxy(self.servicePaths["functions"], GetDomainAttributeService)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')
    
  def getPredicates(self):
    resp = self.getKBPredicates().items
    return self._parseResponse(resp)

  def _parseResponse(self, resp):
    result = {}
    for i in range(len(resp)):
      crntPredicate = resp[i]
      for j in range(len(resp[i].typed_parameters)): 
        item = crntPredicate.typed_parameters[j]
        if crntPredicate.name not in result.keys():
          result[crntPredicate.name] = {}
        result[crntPredicate.name][item.key] = item.value
    return result
  
  def getNumPredicates(self):
    resp = self.getKBNumPredicates().items
    return self._parseResponse(resp)

  def getPropositions(self):
    rospy.wait_for_service(self.servicePaths["propositions"])
    try:
      self.getKBPropositions = rospy.ServiceProxy(self.servicePaths["propositions"], GetAttributeService)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')
    resp = self.getKBPropositions().attributes
    print(f'resp: {resp}')
    return resp

if __name__ == '__main__':
  x = KnowledgeBaseNode()
  # print(f'predicates:\n{x.getPredicates()}')
  # print(f'functions:\n{x.getNumPredicates()}')
  print(f'propositions:\n{x.getPropositions()}')