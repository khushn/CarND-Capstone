### Steps to train the model using MobileNet

#### 1. traffic light Bag
Used the traffic_light_bag_files/just_traffic_light.bag to extract images using bag_to_images.py (custom code got from Internet)

#### 2. labeling of images
Since the images for a traffic light (red/green/yellow) etc typically happen in sequence, was easily able to classify it. 

#### 3. JPEG conversion
Converted them to JPG format as needed by TensorFlow's retrain code. 

#### 4. Training
Did the retraining using: 

<code>
    python tensorflow/examples/image_retraining/retrain.py --image_dir ../training_images/data/ --architecture mobilenet_1.0_224_quantized
</code>

It finished within 10 minutes or so, on my local laptop. And the model generated was around 16 MB.

#### 5. Validation accuracy
Tensorboard showed good progress. 
  Was able to get validation accuracy 95% +

#### 6. Testing of the model
  Update: Now basic testing works on individual files, using command line

<code>
	python tensorflow/examples/label_image/label_image.py --graph=/home/khush/self_driving_car/CarND-Capstone/models/output_graph.pb --labels=/home/khush/self_driving_car/CarND-Capstone/models/output_labels.txt  --input_layer=Placeholder --output_layer=final_result --input_height=224 --input_width=224 --image=/home/khush/self_driving_car/CarND-Capstone/training_images/data/red/frame000542.jpg
</code>

  It gives output like: 

<code>
  red 0.994886

  yellow 0.0051143

  green 2.99788e-08
  
</code>

### Training of simulator data

#### 1. Training
Did the retraining using: 

<code>
    python tensorflow/examples/image_retraining/retrain.py --image_dir ../data/sim_data/ --architecture mobilenet_1.0_224_quantized
</code>

##### Results of training
<image src="tensorboard_sim_data.png"/>

##### End logs
<blockquote>

INFO:tensorflow:2017-12-07 15:20:13.655450: Step 3970: Train accuracy = 100.0%
INFO:tensorflow:2017-12-07 15:20:13.655661: Step 3970: Cross entropy = 0.012413
INFO:tensorflow:2017-12-07 15:20:13.702566: Step 3970: Validation accuracy = 69.0% (N=100)
INFO:tensorflow:2017-12-07 15:20:14.176840: Step 3980: Train accuracy = 100.0%
INFO:tensorflow:2017-12-07 15:20:14.177066: Step 3980: Cross entropy = 0.016685
INFO:tensorflow:2017-12-07 15:20:14.224112: Step 3980: Validation accuracy = 84.0% (N=100)
INFO:tensorflow:2017-12-07 15:20:14.694175: Step 3990: Train accuracy = 100.0%
INFO:tensorflow:2017-12-07 15:20:14.694363: Step 3990: Cross entropy = 0.013385
INFO:tensorflow:2017-12-07 15:20:14.742932: Step 3990: Validation accuracy = 84.0% (N=100)
INFO:tensorflow:2017-12-07 15:20:15.219282: Step 3999: Train accuracy = 100.0%
INFO:tensorflow:2017-12-07 15:20:15.219640: Step 3999: Cross entropy = 0.008534
INFO:tensorflow:2017-12-07 15:20:15.280765: Step 3999: Validation accuracy = 83.0% (N=100)

INFO:tensorflow:Final test accuracy = 77.6% (N=49)
INFO:tensorflow:Froze 2 variables.
Converted 2 variables to const ops.
</blockquote>

#### 2. Testing of the model
  Using the below command line

<code>
	python tensorflow/examples/label_image/label_image.py --graph=/home/khush/self_driving_car/CarND-Capstone/models/sim_model/output_graph.pb --labels=/home/khush/self_driving_car/CarND-Capstone/models/sim_model/output_labels.txt  --input_layer=input --output_layer=final_result --input_height=224 --input_width=224 --image=/home/khush/self_driving_car/CarND-Capstone/data/sim_data/go/g9.jpg
</code>

Gives out output like: 

<blockquote>

	go 0.999956

	stop 4.41143e-05

</blockquote>
