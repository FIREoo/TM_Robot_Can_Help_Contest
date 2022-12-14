#!/usr/bin/env python3
from glob import glob
from re import I
import string
from turtle import distance
import cv2
import rospy
import cv2
import os
import numpy as np
from rospy.core import rospywarn
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
# from vision_msg.msg import YoloBoundingBox

square_image_rect_TL = (0, 0)
square_image_rect_BR = (0, 0)
line_image_rect_TL = (0, 0)
line_image_rect_BR = (0, 0)
roboteye_rect_click = [0, 0]
yolo_detect_rect = [[0, 0], [640, 480]]

hole_image_pos = np.zeros((19, 2), dtype=int)
hole_image_roboteye_pos = np.zeros((10, 2), dtype=int)

hole_status = np.zeros(19, dtype=int)
hole_line_status = np.zeros(10, dtype=int)

LOCKER_model_processing = False


def distance(P1, P2):
    return ((P1[0] - P2[0]) * (P1[0] - P2[0]) + (P1[1] - P2[1]) * (P1[1] - P2[1]))**0.5


def center(box):
    return [int(box[0] + box[2] / 2), int(box[1] + box[3] / 2)]


def img_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr('bridge error {e}')

    ###YOLO###
    img_detect = cv_image[yolo_detect_rect[0][1]:yolo_detect_rect[1][1], yolo_detect_rect[0][0]:yolo_detect_rect[1][0], :].copy()
    img_detect = cv2.resize(img_detect, (detect_width, detect_height), interpolation=cv2.INTER_AREA)
    global LOCKER_model_processing
    try:
        if (LOCKER_model_processing == False):
            classIds, scores, boxes = model.detect(img_detect, confThreshold=0.6, nmsThreshold=0.4)
        else:
            print('locker locked')
            return
    except:
        print('client using model')
        return

    for (classId, score, box) in zip(classIds, scores, boxes):
        # cv2.rectangle(img_detect, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), color=(139, 38, 212), thickness=1)
        cv2.circle(img_detect, center(box), 8, color=(139, 38, 212), thickness=1)
        cv2.putText(img_detect, str(classes[int(classId)]), (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(139, 38, 212), thickness=1)

        #??????hole?????????  #??????????????? YOLO?????? ?????????????????? ????????????
        for i, pos in enumerate(hole_image_pos):
            if (distance(center(box), pos) < 8):
                if (classId == 2):  #good pin
                    hole_status[i] = 1
                elif (classId == 1):  #hole
                    hole_status[i] = 0
                break  #???????????????????????? ????????????
        #???????????????????????? ?????????????????????????????????????????????????????? ?????????
        #>>end for loop for YOLO

    # draw hole pose
    for i, p in enumerate(hole_image_pos):
        if (i <= 8):  # square
            if (hole_status[i] == 0):
                cv2.circle(img_detect, p.astype('int32'), 4, (37, 240, 34), 1)
            elif (hole_status[i] == 1):
                cv2.circle(img_detect, p.astype('int32'), 4, (37, 240, 34), -1)
            cv2.putText(img_detect, str(i), (p[0], p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(37, 240, 34), thickness=1)
        elif (i >= 9):  #line
            if (hole_status[i] == 0):
                cv2.circle(img_detect, p.astype('int32'), 4, (44, 222, 160), 1)
            elif (hole_status[i] == 1):
                cv2.circle(img_detect, p.astype('int32'), 4, (44, 222, 160), -1)
            index_line = i - 9
            cv2.putText(img_detect, str(index_line), (p[0] + 6, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(44, 222, 160), thickness=1)

    cv2.imshow('detect_image', img_detect)

    #state to string
    hole_status_msg = ''
    for s in hole_status:
        hole_status_msg += str(s)
    # pub status message
    pub_hole_status_line.publish(hole_status_msg)

    cv2.rectangle(cv_image, yolo_detect_rect[0], yolo_detect_rect[1], [50, 200, 200], 2)
    cv2.imshow('get_image', cv_image)

    cv2.waitKey(2)


def handle_NG_detection_service(req):
    global LOCKER_model_processing
    LOCKER_model_processing = True

    img_roboteye = cv2.imread('/home/fire/Desktop/showimage.png')  #2592*1944
    # img_roboteye = cv2.resize(img_roboteye, (432, 324))
    # cv2.imshow('robot cam', img_roboteye)
    # print("Service called")

    img_detect = img_roboteye.copy()
    img_detect = cv2.resize(img_detect, (432, 324), interpolation=cv2.INTER_AREA)

    classIds, scores, boxes = model.detect(img_detect, confThreshold=0.6, nmsThreshold=0.4)

    ng_status = np.zeros(10, dtype=int)
    for (classId, score, box) in zip(classIds, scores, boxes):
        cv2.rectangle(img_detect, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), color=(139, 38, 212), thickness=1)
        # cv2.circle(img_detect, center(box), 8, color=(139, 38, 212), thickness=1)
        cv2.putText(img_detect, str(classes[int(classId)]), (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(139, 38, 212), thickness=1)

        # ??????hole?????????  #??????????????? YOLO?????? ?????????????????? ????????????
        for i, pos in enumerate(hole_image_roboteye_pos):
            if (distance(center(box), pos) < 8):
                if (classId == 4):  #bad pin
                    ng_status[i] = 2
                break  #???????????????????????? ????????????
        # >>end for loop for YOLO
    # draw hole pose
    for i, p in enumerate(hole_image_roboteye_pos):
        if (ng_status[i] == 2):
            cv2.circle(img_detect, p.astype('int32'), 4, (23, 34, 179), -1)
        else:
            cv2.circle(img_detect, p.astype('int32'), 4, (44, 222, 160), 1)
        cv2.putText(img_detect, str(i), (p[0], p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(44, 222, 160), thickness=1)

    cv2.imshow('robot cam', img_detect)
    # #state to string
    ng_status_msg = ''
    for s in ng_status:
        ng_status_msg += str(s)

    LOCKER_model_processing = False
    return TriggerResponse(True, ng_status_msg)


def mouse_event_on_get_image(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global yolo_detect_rect
        yolo_detect_rect[0] = [x, y]
        yolo_detect_rect[1] = [x + detect_width, y + detect_height]


def mouse_event_on_detect_image(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global square_image_rect_TL
        square_image_rect_TL = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        global square_image_rect_BR
        square_image_rect_BR = (x, y)

        hole_image_pos[0] = square_image_rect_TL
        hole_image_pos[1] = (square_image_rect_TL[0], (square_image_rect_BR[1] + square_image_rect_TL[1]) / 2)
        hole_image_pos[2] = (square_image_rect_TL[0], square_image_rect_BR[1])
        hole_image_pos[3] = ((square_image_rect_BR[0] + square_image_rect_TL[0]) / 2, square_image_rect_TL[1])
        hole_image_pos[4] = ((square_image_rect_BR[0] + square_image_rect_TL[0]) / 2, (square_image_rect_BR[1] + square_image_rect_TL[1]) / 2)
        hole_image_pos[5] = ((square_image_rect_BR[0] + square_image_rect_TL[0]) / 2, square_image_rect_BR[1])
        hole_image_pos[6] = (square_image_rect_BR[0], square_image_rect_TL[1])
        hole_image_pos[7] = (square_image_rect_BR[0], (square_image_rect_BR[1] + square_image_rect_TL[1]) / 2)
        hole_image_pos[8] = square_image_rect_BR

    elif event == cv2.EVENT_RBUTTONDOWN:
        global line_image_rect_TL
        line_image_rect_TL = (x, y)
    elif event == cv2.EVENT_RBUTTONUP:
        global line_image_rect_BR
        line_image_rect_BR = (x, y)
        #Y line
        distance = line_image_rect_BR[1] - line_image_rect_TL[1]
        for i in range(10):
            index_line = i + 9
            hole_image_pos[index_line] = (line_image_rect_TL[0], line_image_rect_TL[1] + i * distance / 9)


def mouse_event_on_roboteye_image(event, x, y, flags, param):
    if event == cv2.EVENT_RBUTTONDOWN:
        global roboteye_rect_click
        roboteye_rect_click = [x, y]
    elif event == cv2.EVENT_RBUTTONUP:
        roboteye_rect_release = [x, y]
        #Y line
        distance_x = roboteye_rect_release[0] - roboteye_rect_click[0]
        distance_y = roboteye_rect_release[1] - roboteye_rect_click[1]
        for i in range(10):
            hole_image_roboteye_pos[i] = (roboteye_rect_click[0] + i * distance_x / 9, roboteye_rect_click[1] + i * distance_y / 9)


#__Main__
if __name__ == "__main__":
    rospy.init_node('yolo_node', anonymous=True)

    bridge = CvBridge()
    path_of_data = os.path.abspath('../../../data/')
    path_of_data += '/'

    # rospy.loginfo('path of data : {path_of_data}')
    rospy.loginfo('path of data :' + path_of_data)

    file_names = rospy.get_param('~yolo_names')
    with open(path_of_data + file_names, 'r') as f:
        classes = f.read().splitlines()

    #GPU settings
    rospy.loginfo('setting preferable backend and target to CUDA')
    file_cfg = rospy.get_param('~yolo_cfg')
    file_weights = rospy.get_param('~yolo_weights')
    net = cv2.dnn.readNetFromDarknet(path_of_data + file_cfg, path_of_data + file_weights)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    model = cv2.dnn_DetectionModel(net)

    # origin_width = 2048
    # origin_height = 1536
    #640x480
    #512x384
    #384x288
    detect_width = 384
    detect_height = 288
    # model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)
    model.setInputParams(scale=1 / 255, size=(detect_width, detect_height), swapRB=True)

    sub_image = rospy.Subscriber('/camera/image', Image, img_callback, queue_size=1)
    pub_hole_status_plate = rospy.Publisher('/hole_status/square', String, queue_size=1)
    pub_hole_status_line = rospy.Publisher('/hole_status', String, queue_size=1)
    s = rospy.Service('/ng_detect', Trigger, handle_NG_detection_service)

    cv2.namedWindow('get_image')
    cv2.setMouseCallback('get_image', mouse_event_on_get_image)
    cv2.namedWindow('detect_image')
    cv2.setMouseCallback('detect_image', mouse_event_on_detect_image)
    cv2.namedWindow('robot cam')
    cv2.setMouseCallback('robot cam', mouse_event_on_roboteye_image)

    rospy.spin()
    # cv2.destroyAllWindows()
