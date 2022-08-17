'''
不要在function中間 加入好長一段分行
function順序問題  同類的擺在一起
function命名只有功能  但可以加入是誰觸發的 例如callback_... / ..._handler / handle_...
好多意義不明的數字 0.001, 200這些在計算的時候都要說明
global 與 local 不要同名稱,  其實一大堆都應該寫在main裡面,減少使用global參數
高端寫法很酷，我懂我懂，但...帥是一時的 維護是永遠的阿。看懂優先，其次減少行數。（例如 ｃ的　？：這類的...以後舉例

#座標轉換
有實體座標時，圖不能直接就轉換，除非是攝影機內部參數校正，這樣會把舊的圖片的座標直接忽略，（例如這次的底板角度問題。
所以每個轉換都要清楚有沒有影響,這也是為什麼我有給人看的,跟實際的,我還是保留實際的給後續debug用,因為實際的才是與手臂有轉換關係。 這次我直接在轉回去一次（有些好處,但不正式。
所以一般都是用TF來處理這個問題。
'''

from ast import Del
from email.mime import image
from operator import truediv
import string
from time import time

from cv2 import ROTATE_90_CLOCKWISE
import cv2
import numpy as np
import random, time, math, json
import rospy
from tm_msgs.srv import *
from std_msgs.msg import String
import tf
import geometry_msgs.msg
from std_srvs.srv import Empty, Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frameSize = (300, 300, 3)

unit_vector = np.array([0, 1], dtype=np.int32)
robot_img_pos = np.array([0, 300])  #image base
robot_img_target = np.array([0, 0])
handpos = np.array([300, 300], dtype=np.int32)
handshape = np.array([[0, 0], [0, 0], [0, 0]])
status = "pause"

FLAG_realMove = True
FLAG_rotate = True

####座標相關####
# 1. 到image(0,300)_in real的座標, 給定original_robot_pos
# 2. 依序給定points_in_robot_pos裡面的座標軸
place_points = np.array([(75, 225), (75, 150), (75, 75), (150, 225), (150, 150), (150, 75), (225, 225), (225, 150), (225, 75)], dtype=np.int32)  #square hole #image base
place_points_in_robot_pos = np.array([(75, 225), (75, 150), (75, 75), (150, 75), (150, 150), (150, 225), (225, 75), (225, 150), (225, 225)], dtype=np.float64)  #robot base # unit: M
reached_place_points = np.zeros(len(place_points), dtype=np.bool_)

h_line = -43
pick_points = np.array([(h_line, 20), (h_line, 40), (h_line, 60), (h_line, 80), (h_line, 100), (h_line, 120), (h_line, 140), (h_line, 160), (h_line, 180), (h_line, 200)], dtype=np.int32)
# pick_points = np.array([(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0)], dtype=np.int32)
pick_points_in_robot_pos = np.array([(0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180)], dtype=np.float64)  #robot base # unit: M
pick_points_status = np.zeros(len(pick_points), dtype=np.int8)
pick_point_index = 0  #可以夾的，第一順位 index
UPDATE_pick_hole = False

##座標轉換##
original_image_pos = [0, 300]
original_robot_pos = [0.500, -0.180]  #Unit: M
robot_z_high = 0.305  #unit: M
robot_z_down = 0.277  #unit: M

original_image_rect = np.float32([(0, 300), (300, 300), (300, 0), (0, 0)])
# original_robot_rect = np.float32([(0.500, -0.180), (0.800, -0.180), (0.800, 0.120), (0.500, 0.120)])  #Unit: M
original_robot_rect_mm = np.float32([(540.3, -197.3), (838.8, -206.2), (850.6, 92.6), (552.9, 103.1)])  #Unit: mm
trans_R2I = []
trans_I2R = []

###parameters###
hand_safe_distance = 75  #pixel

bridge = CvBridge()
pub_image = rospy.Publisher('/pathplan/image', Image, queue_size=1)


def tm_send_script_client(cmd: str):
    global FLAG_realMove
    if FLAG_realMove == True:
        rospy.wait_for_service('tm_driver/send_script')
        try:
            tm_send_script = rospy.ServiceProxy('tm_driver/send_script', SendScript)
            resp1 = tm_send_script("demo", cmd)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    else:
        print(cmd)


def tm_pose_to(x, y, z, speed, Rx=90, Ry=0, Rz=90):  #Unit mm / deg
    tm_send_script_client("PTP(\"CPP\",%f,%f,%f,%f,%f,%f,%d,200,0,false)" % (x, y, z, Rx, Ry, Rz, speed))
    while not rospy.is_shutdown():
        try:
            t = listener.getLatestCommonTime("/base", "/tool0")
            (trans, rot) = listener.lookupTransform('/base', '/tool0', t)
            if (abs(trans[0] - x / 1000) < 0.001 and abs(trans[1] - y / 1000) < 0.001 and abs(trans[2] - z / 1000) < 0.001):
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tm_client pose to command block error")
            pass
        UpdateImage()
        cv2.waitKey(1)

    pass


def tm_joint_to(j1, j2, j3, j4, j5, j6, speed):  #Unit deg
    # tm_send_script_client("PTP(\"JPP\",%f,%f,%f,%f,%f,%f,%d,200,0,false)" % (j1, j2, j3, j4, j5, j6, speed))
    # while not rospy.is_shutdown():
    #     try:
    #         t = listener.getLatestCommonTime("/base", "/tool0")
    #         (trans, rot) = listener.lookupTransform('/base', '/tool0', t)
    #         if ((trans[0] - j1 ) < 0.001 and (trans[1] - j2) < 0.001 and (trans[2] - z / 1000) < 0.001):
    #             break
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         rospy.logerr("tm_client pose to command block error")
    #         pass

    pass


def tm_send_gripper_client(cmd: bool):
    global FLAG_realMove
    if FLAG_realMove == True:
        rospy.wait_for_service('tm_driver/set_io')
        '''
        SetIORequest_()
        : module(0)
        , type(0)
        , pin(0)
        , state(0.0)  {
        }
        '''
        try:
            tm_send_io = rospy.ServiceProxy('tm_driver/set_io', SetIO)
            if cmd == True:
                resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_ON)
            elif cmd == False:
                resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_OFF)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    else:
        print(cmd)


def ng_detect_client():
    rospy.wait_for_service('ng_detect')
    try:
        trigger_request = rospy.ServiceProxy('ng_detect', Trigger)
        return trigger_request()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def set_points_to_robot_points():
    global place_points, place_points_in_robot_pos, pick_points_in_robot_pos
    for i, p in enumerate(place_points):
        place_points_in_robot_pos[i] = np.float64(Image2Robot(place_points[i]) / 1000)
        #直接補償
        place_points_in_robot_pos[i][0] += 0.003  #unit M
        place_points_in_robot_pos[i][1] += -0.000  #unit M
    for i, p in enumerate(pick_points):
        pick_points_in_robot_pos[i] = np.float64(Image2Robot(pick_points[i]) / 1000)


#mm to pixel
def Robot2Image(robot_point_mm):
    global original_image_pos, original_robot_pos
    # print("robot: (%f, %f)" % (robot_point[0], robot_point[1]))
    #robot_point => Unit: M
    #image (0, 300) = Robot arm (x, y)  ##1 pixel = 1mm
    #image x_axis = robot x_axis
    #image y_axis = robot -y_axis
    # return np.array((original_image_pos[0] + (robot_point[0] - original_robot_pos[0]) * 1000, (original_image_pos[1] - (robot_point[1] - original_robot_pos[1]) * 1000)), dtype=np.int32)
    trans = trans_R2I
    #轉換公式
    x = robot_point_mm[0]
    y = robot_point_mm[1]
    d = trans[2, 0] * x + trans[2, 1] * y + trans[2, 2]
    return np.array([
        ((trans[0, 0] * x + trans[0, 1] * y + trans[0, 2]) / d),  # x
        ((trans[1, 0] * x + trans[1, 1] * y + trans[1, 2]) / d)
    ]  # y
                   )


#pixel to mm
def Image2Robot(image_point):
    global original_image_pos, original_robot_pos
    #image (0, 300) = Robot arm (x, y)  ##1 pixel = 1mm
    #image x_axis = robot x_axis
    #image y_axis = robot -y_axis
    # image to robot
    # trans = cv2.getPerspectiveTransform(original_robot_rect, original_image_rect)
    trans = trans_I2R
    #轉換公式
    x = image_point[0]
    y = image_point[1]
    d = trans[2, 0] * x + trans[2, 1] * y + trans[2, 2]
    return np.float32([
        ((trans[0, 0] * x + trans[0, 1] * y + trans[0, 2]) / d),  # x
        ((trans[1, 0] * x + trans[1, 1] * y + trans[1, 2]) / d)
    ]  # y
                     )
    dst = cv2.warpPerspective(image, trans, (300, 300))
    return np.array(((original_robot_pos[0] * 1000 + (image_point[0] - original_image_pos[0])) / 1000, (original_robot_pos[1] * 1000 - (image_point[1] - original_image_pos[1])) / 1000), dtype=np.float64)  #Unit M


def FixCoordinate_vector(vector):  #我不知道怎麼算的...只能先這樣修改
    # switch x y
    # return np.array([unit_vector[1], unit_vector[0]])
    return np.array([vector[0], -vector[1]])


def get_safe_dst(com, extraLen):

    def intersect(p1, p2, p3, p4):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0:
            return None
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        if ua < 0 or ua > 1:
            return None
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
        if ub < 0 or ub > 1:
            return None
        x = x1 + ua * (x2 - x1)
        y = y1 + ua * (y2 - y1)
        return (x, y)

    hand = handshape + handpos
    for i in range(len(hand)):
        inter = intersect(hand[i - 1], hand[i], handpos, com)
        if inter != None:
            return np.linalg.norm(handpos - inter) + extraLen
    return 100


def is_inside(polygon, point):
    length = len(polygon) - 1
    dy2 = point[1] - polygon[0][1]
    intersections = 0
    ii = 0
    jj = 1
    while ii < length:
        dy = dy2
        dy2 = point[1] - polygon[jj][1]
        if dy * dy2 <= 0.0 and (point[0] >= polygon[ii][0] or point[0] >= polygon[jj][0]):
            if dy < 0 or dy2 < 0:
                F = dy * (polygon[jj][0] - polygon[ii][0]) / (dy - dy2) + polygon[ii][0]
                if point[0] > F:
                    intersections += 1
                elif point[0] == F:
                    return 2
            elif dy2 == 0 and (point[0] == polygon[jj][0] or (dy == 0 and (point[0] - polygon[ii][0]) * (point[0] - polygon[jj][0]) <= 0)):
                return 2
        ii = jj
        jj += 1
    return intersections & 1


def wtf_unit_vec(vec):
    _unit_vector = vec / (math.sqrt(np.linalg.norm(vec) + 0.0001) * 2)
    dir = ((vec > 0) * 2 - np.ones(2)).astype(np.int32)
    _unit_vector = np.ceil(abs(_unit_vector)) * dir
    _unit_vector = _unit_vector.astype(np.int32)

    return _unit_vector


def check_collision():  #命名怪怪的耶
    # collision = np.zeros(len(points), dtype=np.bool)
    for i, p in enumerate(place_points):
        if np.linalg.norm(robot_img_pos - p) < 8:
            # collision[i] = True
            return i
    # return collision
    return -1


# ============================================================================================== #
# 夾取函數 index 0-8 為盤面 9-18 為小盤PIN
def grasp_pin(index: int) -> bool:
    global pick_points_status
    #if 9-18 grip only
    #0-8 place only
    if (index <= 8):
        print("place in index %d" % index)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_down * 1000))
        Delay(2)
        tm_send_gripper_client(False)
        Delay(3)
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        Delay(1)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("ContinueVLine(300, 500)")
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
    elif (index >= 9):
        index_line = index - 9
        print("pick in index %d" % (index_line))

        #grip!!
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (pick_points_in_robot_pos[index_line][0] * 1000, pick_points_in_robot_pos[index_line][1] * 1000, robot_z_high * 1000))
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (pick_points_in_robot_pos[index_line][0] * 1000, pick_points_in_robot_pos[index_line][1] * 1000, robot_z_down * 1000))
        Delay(2)
        tm_send_gripper_client(True)
        Delay(3)
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (pick_points_in_robot_pos[index_line][0] * 1000, pick_points_in_robot_pos[index_line][1] * 1000, robot_z_high * 1000))
        Delay(1)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("ContinueVLine(300, 500)")
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")

        #set to empty in status
        pick_points_status[index_line] = 0

    return True


# 選擇放置目標
def robot_place(targets):
    global reached_place_points, status

    collision = check_collision()
    if collision >= 0:
        if not reached_place_points[collision]:  #無意義的判斷state == "place/put", 唯一入口在main的判斷裡面
            # reached[collision] = True
            if grasp_pin(collision):  ##if中的判斷式 盡量不要寫功能型的function, if在閱讀上就是判斷,這樣會感覺重點是有沒有成功夾取, 而不是夾取這個行為
                status = "pick"

    target = np.array([0, 0])
    minDst = np.Infinity
    found = False
    for i, p in enumerate(targets):
        safeDst = get_safe_dst(p, hand_safe_distance)
        if reached_place_points[i]:
            continue
        elif is_inside(handshape + handpos, p):
            continue
        elif np.linalg.norm(handpos - p) < safeDst:  #is_inside(handshape + handpos, p):
            continue
        elif np.linalg.norm(robot_img_pos - p) < minDst:
            target = p
            minDst = np.linalg.norm(robot_img_pos - p)
            found = True

    if (found == False):  #沒找到就結束
        target = original_image_pos

    bot_move(target)


# 路徑規劃
def bot_move(target):
    global robot_img_pos, unit_vector, robot_img_target
    robot_img_target = target
    safeDst = get_safe_dst(robot_img_pos, hand_safe_distance)
    v = []
    # 防邊緣超出
    # v.append(np.array((botpos[0],0)))
    # v.append(np.array((botpos[0]-frameSize[0],0)))
    # v.append(np.array((0,botpos[0])))
    # v.append(np.array((0,botpos[0]-frameSize[1])))
    v.append(robot_img_pos - handpos)
    sv = np.array((0, 0), dtype=np.float32)
    for x in v:
        x_len = np.linalg.norm(x)
        x.astype(np.float32)
        if x_len < safeDst:
            sv += (x / x_len) * (safeDst - x_len)

    if np.linalg.norm(sv) > 1:  #如果overlap手的話
        direction = np.array((-50, 0))  #圖片方向不一樣阿，明明就是向左！？
    else:  #沒有overlap任何東西, 正常走
        direction = target - robot_img_pos + sv

    unit_vector = wtf_unit_vec(direction)  #direction: vector of P1 to P2
    global FLAG_realMove
    if FLAG_realMove == True:
        # TM指令區 #
        try:
            t = listener.getLatestCommonTime("/base", "/tool0")
            (trans, rot) = listener.lookupTransform('/base', '/tool0', t)
            robot_img_pos = Robot2Image((trans[0] * 1000, trans[1] * 1000))  #trans unit M ;function is mm #np.array(((trans[0] - 0.3) * 1000, (trans[1] + 0.15) * 1000), dtype=np.int32)
            # print("robot:(%d,%d)" % (trans[0] * 1000, trans[1] * 1000))
            # print("robot_image_pos:(%d,%d)" % (robot_img_pos[0], robot_img_pos[1]))

            unit_vector = FixCoordinate_vector(unit_vector)
            # print("Vline: (%f, %f)" % (unit_vector[0], unit_vector[1]))
            tm_send_script_client("SetContinueVLine(%f, %f, 0, 0, 0, 0)" % (float(unit_vector[0]) / 100, float(unit_vector[1]) / 100))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    elif FLAG_realMove == False:
        robot_img_pos += unit_vector  #simulate
        print("Vline: (%f, %f)" % (unit_vector[0], unit_vector[1]))
        time.sleep(0.1)


# 回原點取卯
def robot_pick(index: int):
    global status
    target = np.array([0, 0])  #給予初始值 避免全部都拿完了,target會找不到
    found = False
    for i, p in enumerate(pick_points_status):
        if (p == 1):
            target = pick_points[i]
            pick_point_index = i
            found = True
            break

    # print("pick_point_index: ", pick_point_index)
    if (found == False):  #沒找到就結束
        status = "end"
        print("Pick empty!")
        return

    if np.linalg.norm(robot_img_pos - target) < 8:
        if grasp_pin(9 + pick_point_index):
            status = "place"

    bot_move(target)


#abandon
def get_tangents_vec(a, p):

    def sqDist(p1, p2):
        return ((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

    def orientation(a, b, c):
        res = ((b[1] - a[1]) * (c[0] - b[0]) - (c[1] - b[1]) * (b[0] - a[0]))
        if (res == 0):
            return 0
        if (res > 0):
            return 1
        return -1

    ind = 0
    n = len(a)
    for i in range(1, n):
        if (sqDist(p, a[i]) < sqDist(p, a[ind])):
            ind = i
    up = ind
    while (orientation(p, a[up], a[(up + 1) % n]) >= 0):
        up = (up + 1) % n
    low = ind
    while (orientation(p, a[low], a[(n + low - 1) % n]) <= 0):
        low = (n + low - 1) % n
    return up, low


# 接收手部位置與圖像
def handle_hand_detector(data):
    global handpos, handshape
    # print(data.data)
    data = json.loads(data.data)
    hand = np.array(data["convex"], dtype=np.int32)
    if hand.size != 0:  #hand is detected
        handpos = np.array([np.sum(hand[:, 0]) / len(hand), np.sum(hand[:, 1]) / len(hand)], dtype=np.int32)
        handshape = hand - handpos
    else:  #no hand detected
        handpos = np.array([300, 300], dtype=np.int32)
        hand = np.array([[0, 0], [0, 0], [0, 0]], dtype=np.int32)
        handshape = hand - handpos


# 接收洞口更新資訊
def handle_hole_status(msg):
    # 0100000101111112101
    global reached_place_points, pick_points_status
    global UPDATE_pick_hole

    if len(msg.data) != 19:
        return
    for i, ch in enumerate(msg.data[:9]):
        if ch == "0":
            reached_place_points[i] = False
        else:
            reached_place_points[i] = True

    #update once when init
    if UPDATE_pick_hole == True:
        for i, ch in enumerate(msg.data[9:]):
            pick_points_status[i] = int(ch)
        UPDATE_pick_hole = False
        print("Pick hole status: ", msg.data[9:])


# 鼠標控制(測試用)
def mouse_move(event, x, y, flags, param):
    global handpos
    handpos = np.array((x, y), dtype=np.int32)


# 繪製圖面
def UpdateImage():
    global status
    global FLAG_realMove
    global unit_vector
    img_process = np.ones(frameSize, dtype=np.uint8) * 100
    img_process_sub = np.ones((300, 100, 3), dtype=np.uint8) * 150  #宣告頭頂一塊空間
    cv2.rectangle(img_process_sub, (30, 0), (80, 220), (100, 100, 100), -1)  #把小板的位置畫出來

    hand = handshape + handpos
    cv2.fillPoly(img_process, [hand], (100, 0, 200))

    #color
    color_good_pin = (86, 173, 35)
    color_bad_pin = (58, 25, 179)
    color_hole = (180, 202, 217)
    # draw 9 square point
    for i, p in enumerate(place_points):
        if reached_place_points[i]:
            cv2.circle(img_process, p, 5, color_good_pin, -1)
            cv2.putText(img_process, str(i), (p[0], p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color=color_good_pin, thickness=1)
        else:
            cv2.circle(img_process, p, 5, color_hole, 2)
            cv2.putText(img_process, str(i), (p[0], p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color=color_hole, thickness=1)

    # 繪製小板的十個點
    for i, p in enumerate(pick_points):
        if pick_points_status[i] == 1:  #good pin
            cv2.circle(img_process_sub, p - np.array([-100, 0]), 6, color_good_pin, -1)
        elif pick_points_status[i] == 2:  #bad pin
            cv2.circle(img_process_sub, p - np.array([-100, 0]), 6, color_bad_pin, -1)
        else:  #none
            cv2.circle(img_process_sub, p - np.array([-100, 0]), 6, color_hole, -1)

    #concatenate image
    img_concate = np.concatenate((img_process_sub, img_process), axis=1)

    ##Draw robot##
    if (status != "end" and status != "pause"):
        offset = np.array([100, 0])  #img_process 被向下位移100
        draw_robot_pos = (int(robot_img_pos[0]), int(robot_img_pos[1])) + offset
        if status == "pick":
            if FLAG_realMove:
                cv2.circle(img_concate, draw_robot_pos, 10, (181, 24, 37), 3)
            else:
                cv2.circle(img_concate, draw_robot_pos, 10, (174, 214, 51), 3)
        elif status == "place":
            if FLAG_realMove:
                cv2.circle(img_concate, draw_robot_pos, 10, (181, 24, 37), -1)
            else:
                cv2.circle(img_concate, draw_robot_pos, 10, (174, 214, 51), -1)

        # 高端繪圖法 一行解千愁
        # cv2.circle(img_process if robotPos[0] >= 0 else img_process_sub, robotPos if robotPos[0] >= 0 else robotPos - np.array([-100, 0]), 10, (181, 24, 37) if FLAG_realMove else (174, 214, 51), -1 if status == "place" else 3)

        #draw vector and path line
        if (FLAG_realMove == True):
            unit_vector = FixCoordinate_vector(unit_vector)
        cv2.line(img_concate, draw_robot_pos, (robot_img_pos + offset + unit_vector * 20).astype(np.int32), (162, 33, 176), 3)
        cv2.line(img_concate, draw_robot_pos, (robot_img_target + offset).astype(np.int32), (69, 8, 13), 1)

    #沒什麼意義複製,暫時先用來區分 畫一些無意義的文字
    # img_show = np.concatenate((img_process_sub, img_process), axis=1)
    img_show = img_concate.copy()
    if (FLAG_rotate == True):
        img_show = cv2.rotate(img_show, ROTATE_90_CLOCKWISE)
        y_put = 10
        cv2.putText(img_show, 'Rotate in view(r)', (100, y_put), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_show, 'Real(z)' if FLAG_realMove else 'Simulate(z)', (10, y_put), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (181, 24, 37) if FLAG_realMove else (174, 214, 51), 1, cv2.LINE_AA)
        cv2.putText(img_show, status, (250, y_put), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 82, 200), 1, cv2.LINE_AA)
    else:
        cv2.putText(img_show, 'Rotate in real(r)', (100, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_show, 'Real(z)' if FLAG_realMove else 'Simulate(z)', (10, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (181, 24, 37) if FLAG_realMove else (174, 214, 51), 1, cv2.LINE_AA)
        cv2.putText(img_show, status, (250, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 82, 200), 1, cv2.LINE_AA)

    cv2.imshow("map", img_show)
    image_message = bridge.cv2_to_imgmsg(img_show, "bgr8")
    pub_image.publish(image_message)


def Delay(sec):
    for _ in range(5 * sec):
        UpdateImage()
        time.sleep(0.2)
        cv2.waitKey(1)


def handle_commands(req):
    global status
    global FLAG_realMove, FLAG_rotate, UPDATE_pick_hole
    global pick_points_status
    global ng_status_msg

    _cmd = req.script  # use TM sendScript.srv
    if _cmd == "detect":
        #find NG pin
        print("Check NG pin")
        tm_send_script_client("PTP(\"JPP\",0.0,-20.0,90.0,150.0,-90.0,180.0,80,200,0,false)")  #camera pos #TM5-700
        tm_pose_to(356.2, -70.2, 276.0, 80, Rx=129.8, Ry=0.0, Rz=90.2)
        print("Take a picture")
        tm_send_script_client('ScriptExit(0)')
        Delay(5)
        ng_status_msg = ng_detect_client()
        print(ng_status_msg)
        tm_send_script_client("PTP(\"JPP\",-16.7,-44.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  #out pos#TM5-700
        tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700
    elif _cmd == "remove_ng_pin":
        print("Remove NG pin")
        pick_points_status = np.zeros(len(pick_points), dtype=np.int8)
        for i, ch in enumerate(ng_status_msg.message[:]):
            if ch == "2":
                #trash it!
                tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
                tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_down * 1000, 80)
                tm_send_gripper_client(True)
                Delay(3)
                tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
                #在旁邊7cm的地方放置
                tm_pose_to(pick_points_in_robot_pos[0][0] * 1000, pick_points_in_robot_pos[0][1] * 1000 + 70, robot_z_high * 1000, 80)
                # tm_send_script_client("PTP(\"JPP\",73.5,-27.3,117.7,89.3,-16.2,179.4,80,200,0,false)")  # trash pos #TM5-700
                Delay(3)
                tm_send_gripper_client(False)
                Delay(3)

                #set to can not reached
                pick_points_status[i] = 0
        #switch to "end" when finish
        status = "end"
    elif _cmd == "start":
        #initial pos
        tm_send_script_client("StopAndClearBuffer(0)")
        tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700
        Delay(1)
        tm_pose_to(original_robot_pos[0] * 1000, original_robot_pos[1] * 1000, robot_z_high * 1000, 50)
        UPDATE_pick_hole = True
        Delay(2)
        #Start Vline
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("ContinueVLine(300, 500)")
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        status = "start"

    elif _cmd == "pause":
        status = "pause"
    elif _cmd == "end":
        status = "end"


def key_commands(k):
    global status
    global FLAG_realMove, FLAG_rotate, UPDATE_pick_hole
    global pick_points_status
    global ng_status_msg

    speed_goto_hole = 80
    if k == 255:
        return
    key = k
    if key == ord('i'):
        tm_send_script_client("StopAndClearBuffer(0)")
        tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700
        Delay(1)
        tm_pose_to(original_robot_pos[0] * 1000, original_robot_pos[1] * 1000, robot_z_high * 1000, 50)
        UPDATE_pick_hole = True
        status = "init"
    elif key == ord('p'):
        status = "pause"
    elif key == ord('s'):
        status = "start"
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("ContinueVLine(300, 500)")
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
    elif key == ord('e'):
        status = "end"
    elif key == ord('a'):
        #find NG pin
        print("Check NG pin")
        tm_send_script_client("PTP(\"JPP\",0.0,-20.0,90.0,150.0,-90.0,180.0,80,200,0,false)")  #camera pos #TM5-700
        tm_pose_to(356.2, -70.2, 276.0, 80, Rx=129.8, Ry=0.0, Rz=90.2)
        print("Take a picture")
        tm_send_script_client('ScriptExit(0)')
        Delay(5)
        ng_status_msg = ng_detect_client()
        print(ng_status_msg)
        tm_send_script_client("PTP(\"JPP\",-16.7,-44.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  #out pos#TM5-700
        tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700

    elif key == ord('b'):
        #去除NG pin
        print("Remove NG pin")
        pick_points_status = np.zeros(len(pick_points), dtype=np.int8)
        for i, ch in enumerate(ng_status_msg.message[:]):
            if ch == "2":
                #trash it!
                tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
                tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_down * 1000, 80)
                tm_send_gripper_client(True)
                Delay(3)
                tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
                tm_send_script_client("PTP(\"JPP\",73.5,-27.3,117.7,89.3,-16.2,179.4,80,200,0,false)")  # trash pos #TM5-700
                Delay(3)
                tm_send_gripper_client(False)
                Delay(3)

                #set to can not reached
                pick_points_status[i] = 0

    elif key == ord('z'):
        FLAG_realMove = not FLAG_realMove
    elif key == ord('r'):
        FLAG_rotate = not FLAG_rotate
    elif key == ord('m'):
        tm_send_script_client("StopAndClearBuffer(0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_gripper_client(True)
    elif key == ord('n'):
        tm_send_script_client("StopAndClearBuffer(0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_gripper_client(False)
    elif key == ord('0'):
        #go around
        tm_send_script_client("StopAndClearBuffer(0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_script_client("PTP(\"JPP\",-16.7,-14.1,115.1,79.2,-106.7,180.2,35,200,0,false)")  #ready pos #TM5-700
        Delay(1)
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (original_robot_rect_mm[0][0] * 1000, original_robot_rect_mm[0][1] * 1000, robot_z_high * 1000))
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (original_robot_rect_mm[1][0] * 1000, original_robot_rect_mm[1][1] * 1000, robot_z_high * 1000))
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (original_robot_rect_mm[2][0] * 1000, original_robot_rect_mm[2][1] * 1000, robot_z_high * 1000))
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (original_robot_rect_mm[3][0] * 1000, original_robot_rect_mm[3][1] * 1000, robot_z_high * 1000))

    elif key == ord('1'):
        print('pick points status: index1 -> 1')
        pick_points_status = np.zeros(len(pick_points), dtype=np.int8)
        pick_points_status[0] = 1
    elif key == 170:  #numpad*
        #go reset pin
        print('clear pick points status')
        pick_points_status = np.zeros(len(pick_points), dtype=np.int8)

    elif key == 176:  #numpad0
        print('Robot go to index 0 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[0][0] * 1000, place_points_in_robot_pos[0][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 177:  #numpad1
        print('Robot go to index 1 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[1][0] * 1000, place_points_in_robot_pos[1][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 178:  #numpad2
        print('Robot go to index 2 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[2][0] * 1000, place_points_in_robot_pos[2][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 179:  #numpad3
        print('Robot go to index 3 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[3][0] * 1000, place_points_in_robot_pos[3][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 180:  #numpad4
        print('Robot go to index 4 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[4][0] * 1000, place_points_in_robot_pos[4][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 181:  #numpad5
        print('Robot go to index 5 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[5][0] * 1000, place_points_in_robot_pos[5][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 182:  #numpad6
        print('Robot go to index 6 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[6][0] * 1000, place_points_in_robot_pos[6][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 183:  #numpad7
        print('Robot go to index 7 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[7][0] * 1000, place_points_in_robot_pos[7][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 184:  #numpad8
        print('Robot go to index 8 pos')
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,%d,200,0,false)" % (place_points_in_robot_pos[8][0] * 1000, place_points_in_robot_pos[8][1] * 1000, robot_z_high * 1000, speed_goto_hole))
    elif key == 185:  #numpad9
        print('Robot go to OUTSIDE pos')
        tm_send_script_client("PTP(\"JPP\",-16.7,-44.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  #out pos#TM5-700
    elif key == 171:  #numpad+
        print('Round all pos (square')
        for i in range(9):
            tm_pose_to(place_points_in_robot_pos[i][0] * 1000, place_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
            tm_pose_to(place_points_in_robot_pos[i][0] * 1000, place_points_in_robot_pos[i][1] * 1000, robot_z_down * 1000, 80)
            tm_pose_to(place_points_in_robot_pos[i][0] * 1000, place_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
        tm_pose_to(original_robot_rect_mm[0][0], original_robot_rect_mm[0][1], robot_z_high * 1000, 80)
    elif key == 173:  #numpad-
        print('Round all pos (line')
        for i in range(10):
            tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
            tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_down * 1000, 80)
            tm_pose_to(pick_points_in_robot_pos[i][0] * 1000, pick_points_in_robot_pos[i][1] * 1000, robot_z_high * 1000, 80)
        tm_pose_to(original_robot_rect_mm[0][0], original_robot_rect_mm[0][1], robot_z_high * 1000, 80)
    else:
        print('key=', key)


# ============================================================================================== #

# ROS1實裝 #
#___Main___
if __name__ == "__main__":
    rospy.init_node('pathplan_node', anonymous=True)
    rospy.Subscriber("/hand_detector", String, handle_hand_detector)
    rospy.Subscriber("/hole_status", String, handle_hole_status)
    rospy.Service('/pathplan/command', SendScript, handle_commands)

    listener = tf.TransformListener()

    # ============================================================================================== #
    cv2.namedWindow('map')
    # cv2.namedWindow('robot cam')
    # cv2.setMouseCallback('map', mouse_move)

    ###座標轉換###
    trans_I2R = cv2.getPerspectiveTransform(original_image_rect, original_robot_rect_mm)
    trans_R2I = cv2.getPerspectiveTransform(original_robot_rect_mm, original_image_rect)
    set_points_to_robot_points()

    while not rospy.is_shutdown():
        if status == "init":  #robot back to initial point
            print('Initial')
            #do initial in key press function
            status = "pause"
        elif status == "pause":
            #do notiong
            pass
        elif status == "start":  #robot start pick
            print('Start')
            status = "pick"
        elif status == "pick":
            # print('Pick')
            robot_pick(0)
        elif status == "place":
            # print('Place')
            robot_place(place_points)
        elif status == "end":
            print('End')
            tm_send_script_client("StopAndClearBuffer(0)")
            tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
            tm_send_script_client("StopContinueVmode()")
            tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700
            Delay(1)
            tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700
            status = "pause"

        ###draw image###
        UpdateImage()

        k = cv2.waitKey(1) & 0xFF
        key_commands(k)

    ###node exit###
    tm_send_script_client("StopContinueVmode()")
    cv2.destroyAllWindows()