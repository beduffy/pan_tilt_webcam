# ----------------------------------------------
# --- Author         : Pushpraj Katiyar
# --- Mail           : pushprajkatiyar@gmail.com
# --- Date           : 26th April 2019
# ----------------------------------------------
import numpy as np
#import math
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import json
import argparse
import time
import random

#from collections import defaultdict
#from io import StringIO
import cv2

if tf.__version__ < '1.13.0':
    raise ImportError(
        'Please upgrade your tensorflow installation to v1.13.* or later!')

sys.path.insert(0, 'utils')
import label_map_util
import people_class_util as class_utils
import visualization_utils as vis_util

from send_servo_val_requests import post_servo_value

# parser = argparse.ArgumentParser()
# parser.add_argument('--path', required=True,
#                     help='Path to the video')
#
# opt = parser.parse_args()

# What model to download. KEEP IN MIND ssd_mobilenet_v1_coco_2018_01_28 is fastest model but low accuracy
MODEL_NAME = 'ssd_mobilenet_v1_coco_2018_01_28'
# MODEL_NAME = 'faster_rcnn_nas_lowproposals_coco_2017_11_08'
# MODEL_NAME = 'faster_rcnn_resnet50_coco_2018_01_28'
#MODEL_NAME = 'faster_rcnn_resnet50_lowproposals_coco_2018_01_28'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used
# for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = 'utils/person_label_map.pbtxt'

#cap = cv2.VideoCapture('http://devimages.apple.com/iphone/samples/bipbop/bipbopall.m3u8')
# checking if file exist or not and if yes then downloading specified model
if not os.path.exists(MODEL_FILE):
    opener = urllib.request.URLopener()
    opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
    tar_file = tarfile.open(MODEL_FILE)
    for file in tar_file.getmembers():
        file_name = os.path.basename(file.name)
        if 'frozen_inference_graph.pb' in file_name:
            tar_file.extract(file, os.getcwd())

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
NUM_CLASSES = 50
# loading specified class/category description
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(
    label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


# some helper code
def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)



curr_servo_val = 90
post_servo_value(curr_servo_val)

# start providing video
# you can use live CCTV video URL in place of "test_video.mp4"
cap = cv2.VideoCapture(4)
# cap = cv2.VideoCapture(1)
#get framerate of given video
frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
print('>>>>>>>>>FRAME RATE>>>>>>>' + str(frame_rate))
frame_count = 0
second_completed = 0
#detection of video
with detection_graph.as_default():
  with tf.Session(graph=detection_graph) as sess:
        while(True):
            start = time.time()

            #read every frame
            # success, image_np = cap.read()
            success, image_np = cap.read()
            if not success:
                print ('>>>>>  End of the video file...')
                break

            frame_count += 1
            #check if this frame is closed to time interval provided
            # if frame_count == (2 * frame_rate):
            if True:
                second_completed += 2
                print('>>>>>' + str(second_completed))
                frame_count = 0

                #flaten image using numpy
                image_np_expanded = np.expand_dims(image_np, axis=0)
                image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                scores = detection_graph.get_tensor_by_name('detection_scores:0')
                classes = detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = detection_graph.get_tensor_by_name('num_detections:0')

                (boxes, scores, classes, num_detections) = sess.run(
                    [boxes, scores, classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
                # Visualization of the results of a detection.
                # vis_util.visualize_boxes_and_labels_on_image_array(
                #     image_np,
                #     np.squeeze(boxes),
                #     np.squeeze(classes).astype(np.int32),
                #     np.squeeze(scores),
                #     category_index,
                #     use_normalized_coordinates=True,
                #     line_thickness=8)
                

                #creating annotation array which can be sent to API for further operations
                annotations = {}
                annotations['class_annotations'], count = (
                    class_utils.get_class(np.squeeze(classes).astype(np.int32),
                                          category_index, np.squeeze(boxes),
                                          np.squeeze(scores)))
                annotations['person_count'] = count
                print(json.dumps(annotations))

                im_width = 640
                im_height = 480
                im_half_width = int(640 / 2)
                if annotations['class_annotations']:
                    for annotation in annotations['class_annotations']:
                        xmin = annotation['bounding_box']['xmin']
                        xmax = annotation['bounding_box']['xmax']
                        ymin = annotation['bounding_box']['ymin']
                        ymax = annotation['bounding_box']['ymax']
                        (left, right, top, bottom) = (int(round(xmin * im_width)), int(round(xmax * im_width)),
                                        int(round(ymin * im_height)), int(round(ymax * im_height)))
                        
                        cv2.rectangle(image_np, (left, top), (right, bottom), (0, 255, 0), 2)
                        center_x = int(round((right - left) / 2) + left)
                        center_y = int(round((bottom - top) / 2) + top)
                        # center_x = int(round((right - left) / 2)) + left
                        # center_y = int(round((bottom - top) / 2)) + bottom

                        cv2.circle(image_np, (center_x, center_y), 5, (0, 0, 255), 4)   
                        # cv2.circle(image_np, (center_x, im_height - center_y), 5, (0, 0, 255), 4)
                        # print('center: ', center_x, center_y)
                        horizontal_distance_to_center_x_bbox = im_half_width - center_x
                        print('distance: ', horizontal_distance_to_center_x_bbox)

                        amount_to_rotate_by = 1
                        if horizontal_distance_to_center_x_bbox > 0:
                            print('distance positive, object to left')
                            curr_servo_val += amount_to_rotate_by
                        else:
                            print('distance negative, object pospost_servo_value(curr_servo_val)t_servo_value(curr_servo_val)to right')
                            curr_servo_val -= amount_to_rotate_by

                        if curr_servo_val < 0:
                            curr_servo_val = 0
                        elif curr_servo_val > 180:
                            curr_servo_val = 180

                        cv2.line(image_np, (center_x, center_y), 
                                 (im_half_width, center_y), (0, 255, 0), thickness=3)


                        post_servo_value(curr_servo_val)

                # import pdb;pdb.set_trace()
                #writting to json file for now, this block will contain API/DB code to handle data.
                # with open('annotation_'  + str(second_completed) + '.json', 'w') as file:
                #     json.dump(annotations, file)

                #show annotated image on desktop
                cv2.imshow('object detection', image_np)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break

            print('Time taken: {}'.format(time.time() - start))

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()