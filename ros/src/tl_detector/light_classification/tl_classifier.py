from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import rospy
import datetime
import numpy as np

class TLClassifier(object):
    def __init__(self):

        PATH_TO_GRAPH= 'light_classification/models/graph_optimized.pb'

        self.graph=tf.Graph()
        self.threshold=0.5

        with self.graph.as_default():
            graph_def= tf.GraphDef()
            with tf.gfile.Gfile(PATH_TO_GRAPH, 'rb') as fid:
                graph_def.ParseFromString(fid.read())
                tf.import_graph_def(graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes_tensor = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores_tensor = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes_tensor = self.graph.get_tensor_by_name('detection_classes:0')
            self.detections_tensor = self.graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.graph)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            img_expand=np.expand_dims(image, axis=0)
            time_start= datetime.datetime.now()

            (boxes,scores,classes,num_detections)=self.sess.run([self.boxes_tensor, self.scores_tensor,self.classes_tensor,self.detections_tensor], feed_dict={self.image_tensor:img_expand})
            time_end= datetime.datetime.now()
            delta_t= time_end-time_start
            print(delta_t.total_seconds())

        boxes= np.squeeze(boxes)
        scores= np.squeeze(scores)
        classes= np.squeeze(classes).astype(np.int32)

        print('SCORES: ', scores[0])
        print('CLASSES: ', classes[0])

        if scores[0] > self.threshold:
            if classes[0] == 1:
                print('GREEN')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                print('RED')
                return TrafficLight.RED
            elif classes[0] == 3:
                print('YELLOW')

        return TrafficLight.UNKNOWN
