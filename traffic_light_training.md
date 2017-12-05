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

#### 6. Testing is pending
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

