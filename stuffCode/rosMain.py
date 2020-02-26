import rospy
from std_msgs.msg import String
import linearAct

relevantDataType
relevantDataA
relevantDataB

def transferData(data):
    relevantDataA = data
    relevantDataB = data
    relevantDataType = data

def listentingMain():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('sometpicToGetFrom', int, transferData)

if __name__ == '__main__':
    listeningMain();

pub = rospy.Publisher('someTopicToGiveTo',Data,queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    dataPoint = Data(relevantDataA,relevantDataB,relevantDataType)
    pub.publish(dataPoint)
    rate.sleep()
