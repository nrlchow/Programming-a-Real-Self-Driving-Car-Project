from styx_msgs.msg import TrafficLight
import rospy
import rospkg
import os
import sys
import numpy as np
import tensorflow as tf
from glob import glob
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
import tensorflow as tf
from collections import defaultdict
from object_detection import label_map_util,visualization_utils


class TLClassifier(object):    
    def __init__(self):                                          
        #TODO load classifier
        self.current_light = TrafficLight.UNKNOWN
        model_sim = '/home/nrlc/Desktop/Udacity-CarND-Capstone/ros/src/tl_detector/light_classification/Carla_Traffic_Light_Detection/output_inference_graph_sim/frozen_inference_graph.pb'
        model_real = '/home/nrlc/Desktop/Udacity-CarND-Capstone/ros/src/tl_detector/light_classification/Carla_Traffic_Light_Detection/output_inference_graph_real/frozen_inference_graph.pb'
        labels_file = '/home/nrlc/Desktop/Udacity-CarND-Capstone/ros/src/tl_detector/light_classification/Carla_Traffic_Light_Detection/label_map.pbtxt'
        num_classes = 4
        label_map = label_map_util.load_labelmap(labels_file)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes)
        self.category_index = label_map_util.create_category_index(categories)
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_sim, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')

            with tf.Session(graph=self.detection_graph) as sess:
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
#                 image_np = np.array(image)   #('/home/nrlc/Desktop/Udacity-CarND-Capstone/ros/src/tl_detector/left0030.jpg')
#                 # Expand dimensions since the model expects images to have shape: [1, None, None, 3]		
#                 image_np_expanded = np.expand_dims(image_np, axis=0)
#                 # Actual detection.
#                 (boxes, scores, classes, num) = sess.run(
#                   [detection_boxes, detection_scores, detection_classes, num_detections],
#                   feed_dict={image_tensor: image_np_expanded})

#                 boxes = np.squeeze(boxes)
#                 scores = np.squeeze(scores)
#                 classes = np.squeeze(classes).astype(np.int32)

#                 # Visualization of the results of a detection.
#                 vis_util.visualize_boxes_and_labels_on_image_array(
#                     image_np, boxes, classes, scores,
#                     category_index,
#                     use_normalized_coordinates=True,
#                     line_thickness=6)

#                 plt.figure(figsize=IMAGE_SIZE)
#                 plt.imshow(image_np)
#                 plt.show()

#                 min_score_thresh = .30
#                 for i in range(boxes.shape[0]):
#                     if scores is None or scores[i] > min_score_thresh:

#                         class_name = category_index[classes[i]]['name']
#                         fx =  0.97428
#                         fy =  1.73205
#                         perceived_width_x = (boxes[i][3] - boxes[i][1]) * 700
#                         perceived_width_y = (boxes[i][2] - boxes[i][0]) * 500

#                         # ymin, xmin, ymax, xmax = box
#                         perceived_depth_x = ((.1 * fx) / perceived_width_x)
#                         perceived_depth_y = ((.3 * fy) / perceived_width_y )


    def get_classification(self, image):                                          
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        self.current_light = TrafficLight.UNKNOWN

        return self.current_light

