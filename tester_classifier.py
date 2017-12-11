from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import tensorflow as tf

#path="/home/student/Desktop/kn/training_files/carla_images/green/"
path="/home/student/Desktop/kn/training_files/simulator_images/stop/"
#file = path + "frame000148.jpg"
go_count=0
stop_count=0

for file in os.listdir(path):
	if file.endswith(".jpg"):
		image = cv2.imread(path+file)
		light_classifier = TLClassifier(False)
		result= light_classifier.get_classification(image)
		if result== "go":
			go_count+=1
			print file
		if result== "stop":
			stop_count+=1
			#print file
		print stop_count+go_count
print "go =",go_count
print "stop =",stop_count
print "total =",go_count+stop_count
