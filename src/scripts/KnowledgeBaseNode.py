import rospy
import numpy as np
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.srv import GetDomainAttributeService, GetDomainAttributeServiceResponse
from rosplan_knowledge_msgs.srv import GetAttributeService, GetAttributeServiceResponse
from rosplan_knowledge_msgs.msg import StatusUpdate, KnowledgeItem

class KnowledgeBaseNode():
  '''
  Class that interacts with specific rosplan_knowledge_base services.
  Able to get proposition, predicate and function data, update the KnowledgeBase
  and delete from the KnowledgeBase
  '''
  def __init__(self, _callback):
    '''
    Constructor, sets the service paths used in the class and the
    initial time when plan execution starts
    @param self: the class itself
    @param _callback: a function to call if there is a status update
    '''
    self.t0 = 0
    self.firstCall = True
    self._callback=_callback
    basePath = """/rosplan_knowledge_base"""
    self.servicePaths = {
      "predicates": f"{basePath}/domain/predicates",
      "functions": f"{basePath}/domain/functions",
      "propositions": f"{basePath}/state/propositions",
      "numericFluents": f"{basePath}/state/functions",
      "statusUpdate":f"{basePath}/status/update",
      "update": f"{basePath}/update_array"
    }
    self.knowledgeTypes = {}
    self.kus = KnowledgeUpdateServiceArrayRequest()
    self._setUp()
  
  def _setUp(self):
    '''
    Procedure to set up the rosservice connections
    @param self: the class itself
    '''
    rospy.wait_for_service(self.servicePaths["predicates"])
    rospy.wait_for_service(self.servicePaths["functions"])
    rospy.wait_for_service(self.servicePaths["propositions"])
    rospy.wait_for_service(self.servicePaths["numericFluents"])
    rospy.wait_for_service(self.servicePaths["update"])
    self._statusUpdateListener(self._callback)
    try:
      self._getKBPredicates = rospy.ServiceProxy(self.servicePaths["predicates"], GetDomainAttributeService)
      self._getKBNumPredicates = rospy.ServiceProxy(self.servicePaths["functions"], GetDomainAttributeService)
      self._getKBPropositions = rospy.ServiceProxy(self.servicePaths["propositions"], GetAttributeService)
      self._getKBNumPropositions = rospy.ServiceProxy(self.servicePaths["numericFluents"], GetAttributeService)
      self._updateServer = rospy.ServiceProxy(self.servicePaths["update"], KnowledgeUpdateServiceArray)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')

  def _parseResponse(self, resp):
    '''
    Function to parse rosservice response objects
    and return it in a useful format
    @param self: the class itself
    @param resp: a response object
    @return the parsed response dictionary
    '''
    result = {}
    if isinstance(resp, GetDomainAttributeServiceResponse):
      result = self._parseGetDomainAttributeServiceResponse(resp.items)
    elif isinstance(resp, GetAttributeServiceResponse):
      result = self._parseGetAttributeServiceResponse(resp.attributes)
    return result
  
  def _parseGetDomainAttributeServiceResponse(self, resp):
    '''
    Function to parse and return domain data
    @param self: the class itself
    @param resp: a GetDomainAttributeServiceResponse object
    @return parsed resp GetDomainAttributeServiceResponse dictionary
    '''
    result = {}
    for crntPredicate in resp:
      for item in crntPredicate.typed_parameters: 
        if crntPredicate.name not in result.keys():
          result[crntPredicate.name] = {}
        result[crntPredicate.name][item.key] = item.value
    return result
  
  def _parseGetAttributeServiceResponse(self, resp):
    '''
    Function to parse and return state data
    @param self: the class itself
    @param resp: a GetAttributeServiceResponse object
    @return parsed resp GetAttributeServiceResponse dictionary
    '''
    result = {}
    if self.firstCall:
      self.t0 = resp[0].initial_time.secs
      self.firstCall = False
    for crntProp in resp:
      crntKeys = list(result.keys())
      if crntProp.attribute_name not in crntKeys:
        result[crntProp.attribute_name] = []
      entry = [crntProp.initial_time.secs - self.t0]
      for item in crntProp.values:
        entry.append(item.value)
      # Add numeric value to entry if crntProp is a numeric fluent
      if crntProp.knowledge_type == 2:
        entry.append(crntProp.function_value)
      elif crntProp.knowledge_type == 1:
        entry.append('True')
      result[crntProp.attribute_name].append(entry)
      self.knowledgeTypes[crntProp.attribute_name] = crntProp.knowledge_type
    return result
  
  def _parseUpdateRequest(self, vals):
    '''
    Function to construct and return a KnowledgeItem from the vals
    @param self: the class itself
    @param vals: a dictionary containing KnowledgeItem values
    @return a KnowledgeItem msg containing data from vals
    '''
    ki = KnowledgeItem()
    ki.attribute_name = vals['attribute_name']
    ki.knowledge_type = ki.FACT
    if 'is_negative' in vals.keys():
      ki.is_negative = vals['is_negative']
    ki.values = vals['values']
    if 'function_value' in list(vals.keys()):
      ki.knowledge_type = ki.FUNCTION
      ki.function_value = float(vals['function_value'])
    return ki

  def getPredicates(self):
    '''
    Function to return predicate data
    @param self: the class itself
    @return a predicate data dictionary
    '''
    self.predicates = self._parseResponse(self._getKBPredicates())
    return self.predicates

  def getNumPredicates(self):
    '''
    Function to return numerical predicate data
    @param self: the class itself
    @return a numerical predicate data dictionary
    '''
    self.numPredicates = self._parseResponse(self._getKBNumPredicates())
    return self.numPredicates

  def getPropositions(self):
    '''
    Function to return proposition data
    @param self: the class itself
    @return a proposition data dictionary
    '''
    self.propositions = self._parseResponse(self._getKBPropositions())
    return self.propositions
  
  def getNumPropositions(self):
    '''
    Function to return numerical proposition data
    @param self: the class itself
    @return a numerical proposition data dictionary
    '''
    self.numPropositions = self._parseResponse(self._getKBNumPropositions())
    return self.numPropositions
  
  def _statusUpdateListener(self, _callback):
    '''
    Procedure that initialises a status update listener
    @param self: the class itself
    @param _callback: a function to call if there is a status update
    '''
    rospy.init_node('listener', anonymous=True)
    try:
      rospy.Subscriber(self.servicePaths['statusUpdate'], StatusUpdate, _callback)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')
  
  def update(self, facts):
    '''
    Procedure to prepare the update request
    @param self: the class itself
    @param facts: dictionary of prepared table record values
    '''
    kus = self.kus
    crntKI = self._parseUpdateRequest(facts['crnt'])
    newKI = self._parseUpdateRequest(facts['new'])
    kus.update_type += np.array(kus.REMOVE_KNOWLEDGE).tostring()
    kus.knowledge.append(crntKI)
    kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
    kus.knowledge.append(newKI)

    self._sendToServer(kus)
    self.kus = KnowledgeUpdateServiceArrayRequest()
  
  def delete(self, facts):
    '''
    Procedure to prepare the delete request
    @param self: the class itself
    @param facts: dictionary of prepared table record values
    '''
    kus = self.kus
    knowledgeItem = self._parseUpdateRequest(facts['crnt'])
    kus.update_type += np.array(kus.REMOVE_KNOWLEDGE).tostring()
    kus.knowledge.append(knowledgeItem)

    self._sendToServer(kus)
    self.kus = KnowledgeUpdateServiceArrayRequest()
  
  def _sendToServer(self, request):
    '''
    Procedure to send the prepared request to the ROSPlan
    update service
    @param self: the class itself
    @param request: an instance of the KnowledgeUpdateServiceArrayRequest
    '''
    try: 
      self._updateServer.call(request)
    except Exception as e:
      rospy.logerr(f"Service call failed: {e}")