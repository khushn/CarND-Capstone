import rosbag

bag = rosbag.Bag('../udacity_succesful_light_detection.bag')
i=0
for topic, msg, t in bag.read_messages(topics=['/image_color', '/traffic_waypoint']):
	print i, topic, t
	i+=1
bag.close()

