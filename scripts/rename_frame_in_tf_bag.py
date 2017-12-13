import rosbag
from std_msgs.msg import Int32, String

input_bag = '/home/mdrwiega/2019-02-04-13-23-28.bag'
output_bag = '/home/mdrwiega/2019-02-04-13-23-28_filtered.bag'

bag = rosbag.Bag(input_bag, 'r')

for topic, msg, t in bag.read_messages(topics=['/tf_static']):
    print msg
    print "END OF MESSAGE"
bag.close()