from styx_msgs.msg import TrafficLight
import cv2
import tensorflow as tf
import os.path
import numpy as np
import time

SCRIPT_FOLDER = os.path.dirname(__file__)
INFERENCE_GRAPH = os.path.join(SCRIPT_FOLDER, "training/training_results.pb")

MIN_SCORE_VALUE = 0.2
MIN_NORMED_HEIGHT = 0.08

def label_to_traffic_light(cls):
    if 3 == cls:
        return TrafficLight.YELLOW
    elif 1 == cls:
        return TrafficLight.GREEN
    elif 2 == cls:
        return TrafficLight.RED

    return TrafficLight.UNKNOWN

def draw_bboxes(image, boxes, classes, scores):
    max_boxes_to_draw = 10

    for i in range(min(max_boxes_to_draw, boxes.shape[0])):
        if scores[i] < MIN_SCORE_VALUE:
            continue

        ymin_normed, xmin_normed, ymax_normed, xmax_normed = boxes[i]
        if ymax_normed - ymin_normed < MIN_NORMED_HEIGHT:
            continue

        ymin = int(ymin_normed * image.shape[0])
        ymax = int(ymax_normed * image.shape[0])
        xmin = int(xmin_normed * image.shape[1])
        xmax = int(xmax_normed * image.shape[1])

        tl = label_to_traffic_light(classes[i])
        if TrafficLight.RED == tl:
            color = (0, 0, 255)
        elif TrafficLight.YELLOW == tl:
            color = (0, 255, 255)
        elif TrafficLight.GREEN == tl:
            color = (0, 255, 0)
        else:
            color = (255, 255, 255)

        cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 3)


class TLClassifier(object):
    def __init__(self):
        config = tf.ConfigProto()

        config.graph_options.optimizer_options.global_jit_level = \
            tf.OptimizerOptions.ON_1

        detection_graph = tf.Graph()
        self.session = tf.Session(graph=detection_graph, config=config)

        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(INFERENCE_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        print "Number of graph operations", len(detection_graph.get_operations())

        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    def __del__(self):
        self.session.close()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_expanded = np.expand_dims(image, axis=0)

        time0 = time.time()

        (boxes, scores, classes) = self.session.run(
            [self.detection_boxes,
             self.detection_scores,
             self.detection_classes],
            feed_dict={self.image_tensor: image_expanded})

        time1 = time.time()

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        scores_per_tl = [0]*5 # Because TrafficLight.UNKNOWN has value 4
        for b, s, c in zip(boxes, scores, classes):
            if s < MIN_SCORE_VALUE:
                continue

            ymin_normed, xmin_normed, ymax_normed, xmax_normed = b
            if ymax_normed - ymin_normed < MIN_NORMED_HEIGHT:
                continue

            scores_per_tl[label_to_traffic_light(c)] += s

        max_total_score = MIN_SCORE_VALUE * 1.5
        max_result = TrafficLight.UNKNOWN
        for i, s in enumerate(scores_per_tl):
            if s > max_total_score:
                max_total_score = s
                max_result = i

        # Set True to output detection and classification results. Will work
        # only if you have graphical environment on your ROS Ubuntu machine,
        # like XWindow
        if False:
            print "Classification time", (time1 - time0) * 1000.0, "ms"
            print "score, result", max_total_score, max_result
            print
            #draw_bboxes(image, boxes, classes, scores,)
            #cv2.imshow("camera", image)
            #cv2.waitKey(1)

        return max_result
