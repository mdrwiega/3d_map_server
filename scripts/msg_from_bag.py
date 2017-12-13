import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('2018-11-27-18-44-57.bag', 'r')

for topic, msg, t in bag.read_messages(topics=['/tf_static']):
    print msg
    print "END OF MESSAGE"
bag.close()