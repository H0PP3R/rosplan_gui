from app import App
import rospy

if __name__ == '__main__':
  rospy.init_node('rosplan_gui', anonymous=False)
  App()
