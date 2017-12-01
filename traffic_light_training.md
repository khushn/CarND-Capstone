### Steps to train the model using MobileNet

#### 1. Used the traffic_light_bag_files/just_traffic_light.bag to extract images using bag_to_images.py (custom code got from Internet)

#### 2. Since the images for a traffic light (red/green/yellow) etc typically happen in sequence, was easily able to classify it. 

#### 3. Converted them to JPG format as needed by TensorFlow's retrain code. 

#### 4. Training
Did the retraining using: 
<code>
    python tensorflow/examples/image_retraining/retrain.py --image_dir ../training_images/data/ --architecture mobilenet_1.0_224_quantized
</code>

It finished within 10 minutes or so, on my local laptop. And the model generated was around 16 MB.

#### 5. Tensorbroad showed good progress. 
  Was able to get validation accuracy 95% +

#### 6. Testing is pending
  But could not test it so far. When tried running 
<code>
	python tensorflow/examples/label_image/label_image.py --graph=/tmp/output_graph.pb --labels=/tmp/output_labels.txt --image=/home/khush/self_driving_car/CarND-Capstone/training_images/data/red/frame000542.jpg
</code>
  It was giving an error like: 
  KeyError: "The name 'import/input' refers to an Operation not in the graph.

  So need to know what is the name of the input layer here. Or figure it out in some way.

