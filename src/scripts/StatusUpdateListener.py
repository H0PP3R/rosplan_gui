import rospy
from rosplan_knowledge_msgs.msg import StatusUpdate
class StatusUpdateListener():
  def __init__(self, _callback):
    self.KB = '/rosplan_knowledge_base/status/update'
    self._listener(_callback)
    # self._callback = _callback

  def _listener(self, _callback):
    rospy.init_node('listener', anonymous=True)
    try:
      rospy.Subscriber(self.KB, StatusUpdate, _callback)
    except rospy.ServiceException as e:
      print(f'Service call failed: {e}')
    rospy.spin()

class parentNode():
  def __init__(self):
    StatusUpdateListener(self._callback)

  def _callback(self, data):
    print('I awm jwust a widdle callbwack fwunction')

if __name__=='__main__':
  parentNode()
