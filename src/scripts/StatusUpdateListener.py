from tracemalloc import Statistic
import rospy
from rosplan_knowledge_msgs.msg import KnowledgeItem, StatusUpdate
class StatusUpdateListener():
  def __init__(self):
    self.KB = '/rosplan_knowledge_base/status/update'
    # listener subscription
    # try:
    #   self.kb_update_status_subs = rospy.Subscriber(self.knowledge_base + '/status/update', StatusUpdate, self.kb_update_status)
    # except rospy.ServiceException as e:
    #   print(f'Service call failed: {e}')
    return 0

  def _callback(self, data):
    rospy.loginfo(rospy.get_caller_id() +"I heard %s", data.data)

  def listener(self):
    rospy.init_node('listener', anonymous=True)
    try:
      rospy.Subscriber(self.KB, StatusUpdate, self._callback)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')
    rospy.spin()
  
if __name__=='__main__':
  StatusUpdateListener().listener()
