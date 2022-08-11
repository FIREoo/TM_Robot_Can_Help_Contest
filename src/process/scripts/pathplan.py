'''
不要在function中間 加入好長一段分行
function順序問題  同類的擺在一起
function命名只有功能  但可以加入是誰觸發的 例如callback_... / ..._handler / handle_...
好多意義不明的數字 0.001, 200這些在計算的時候都要說明
global 與 local 不要同名稱,  其實一大堆都應該寫在main裡面,減少使用global參數
'''

from ast import Del
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

frameSize = (300, 300, 3)
handpos = np.array([random.randint(100, frameSize[0] - 100), random.randint(100, frameSize[1] - 100)], dtype=np.int32)

unit_vector = np.array([0, 1], dtype=np.int32)
robotPos = [0, 300]  #image base
robot_target = np.array([0, 0])
handshape = np.array([[76, -28], [38, 50], [-40, 54], [-72, -24], [-2, -52]])
status = "pause"

FLAG_realMove = True
FLAG_rotate = True

####座標相關####
# 1. 到image(0,300)_in real的座標, 給定original_robot_pos
# 2. 依序給定points_in_robot_pos裡面的座標軸
place_points = np.array([(75, 225), (75, 150), (75, 75), (150, 225), (150, 150), (150, 75), (225, 225), (225, 150), (225, 75)], dtype=np.int32)  #square hole #image base
place_points_in_robot_pos = np.array([(75, 225), (75, 150), (75, 75), (150, 75), (150, 150), (150, 225), (225, 75), (225, 150), (225, 225)], dtype=np.float64)  #robot base # unit: M
reached_place_points = np.zeros(len(place_points), dtype=np.bool_)

pick_points = np.array([(0, 300), (1, 300), (2, 300), (3, 300), (4, 300), (5, 300), (6, 300), (7, 300), (8, 300), (9, 300)], dtype=np.int32)  #at 10,10 #line hole #image base
pick_points_in_robot_pos = np.array([(0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180), (0.500, -0.180)], dtype=np.float64)  #robot base # unit: M
reached_pick_points = np.ones(len(pick_points), dtype=np.bool_)
pick_point_index = 0  #可以夾的，第一順位 index

##座標轉換##
original_image_pos = [0, 300]
original_robot_pos = [0.500, -0.180]  #Unit: M
robot_z_high = 0.32  #unit: M
robot_z_down = 0.3  #unit: M

###parameters###
hand_safe_distance = 35  #pixel


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
    global place_points, place_points_in_robot_pos
    for i, p in enumerate(place_points):
        place_points_in_robot_pos[i] = Image2Robot(place_points[i])


def Robot2Image(robot_point):
    global original_image_pos, original_robot_pos
    # print("robot: (%f, %f)" % (robot_point[0], robot_point[1]))
    #robot_point => Unit: M
    #image (0, 300) = Robot arm (x, y)  ##1 pixel = 1mm
    #image x_axis = robot x_axis
    #image y_axis = robot -y_axis
    return np.array((original_image_pos[0] + (robot_point[0] - original_robot_pos[0]) * 1000, (original_image_pos[1] - (robot_point[1] - original_robot_pos[1]) * 1000)), dtype=np.int32)


def Image2Robot(image_point):
    global original_image_pos, original_robot_pos
    #image (0, 300) = Robot arm (x, y)  ##1 pixel = 1mm
    #image x_axis = robot x_axis
    #image y_axis = robot -y_axis
    return np.array(((original_robot_pos[0] * 1000 + (image_point[0] - original_image_pos[0])) / 1000, (original_robot_pos[1] * 1000 - (image_point[1] - original_image_pos[1])) / 1000), dtype=np.float64)  #Unit M


def FixCoordinate_vector(vector):  #我不知道怎麼算的...只能先這樣修改
    # switch x y
    # return np.array([unit_vector[1], unit_vector[0]])
    return np.array([vector[0], -vector[1]])


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


def get_safe_dst(com, extraLen):
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
        if np.linalg.norm(robotPos - p) < 8:
            # collision[i] = True
            return i
    # return collision
    return -1


# ============================================================================================== #
# 夾取函數 index 0-8 為盤面 9-18 為小盤PIN
def grasp_pin(index: int) -> bool:
    global reached_pick_points
    #if 9-18 grip only
    #0-8 place only
    if (index <= 8):
        print("place in index %d" % index)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_down * 1000))
        tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        Delay(5)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("ContinueVLine(300, 500)")
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
    elif (index >= 9):

        print("pick in index %d" % (index - 9))
        #set to empty in reached
        reached_pick_points[index - 9] = False

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
        elif np.linalg.norm(robotPos - p) < minDst:
            target = p
            minDst = np.linalg.norm(robotPos - p)
            found = True

    if (found == False):  #沒找到就結束
        status = "end"

    bot_move(target)


# 路徑規劃
def bot_move(target):
    global robotPos, unit_vector, robot_target
    robot_target = target
    safeDst = get_safe_dst(robotPos, hand_safe_distance)
    v = []
    # 防邊緣超出
    # v.append(np.array((botpos[0],0)))
    # v.append(np.array((botpos[0]-frameSize[0],0)))
    # v.append(np.array((0,botpos[0])))
    # v.append(np.array((0,botpos[0]-frameSize[1])))
    v.append(robotPos - handpos)
    sv = np.array((0, 0), dtype=np.float32)
    for x in v:
        x_len = np.linalg.norm(x)
        x.astype(np.float32)
        if x_len < safeDst:
            sv += (x / x_len) * (safeDst - x_len)

    if np.linalg.norm(sv) > 1:  #如果overlap手的話
        # hand = handpos + handshape
        # if is_inside(hand, botpos):
        # direction = sv
        direction = np.array((-10, 0))  #圖片方向不一樣阿，明明就是向左！？
        # else:
        # up, down = get_tangents_vec(hand, botpos)
        # # = np.dot((botpos-hand)[tangents[0]], (target-botpos)), np.dot((botpos-hand)[tangents[1]], (target-botpos))
        # # print(up, down)
        # # if up > down and abs(up-down)>abs(up+down)/10:
        # # direction = sv + (botpos-hand)[tangents[0]]/2
        # # else:
        # # direction = sv + (hand[up] - hand[down]) / 10
        # if hand[up][1] < hand[down][1]:
        # direction = sv + hand[up] / 10
        # else:
        # direction = sv + hand[down] / 10
    else:  #沒有overlap任何東西, 正常走
        direction = target - robotPos + sv

    unit_vector = wtf_unit_vec(direction)  #direction: vector of P1 to P2
    global FLAG_realMove
    if FLAG_realMove == True:
        # TM指令區 #
        try:
            t = listener.getLatestCommonTime("/base", "/tool0")
            (trans, rot) = listener.lookupTransform('/base', '/tool0', t)
            robotPos = Robot2Image(trans)  #np.array(((trans[0] - 0.3) * 1000, (trans[1] + 0.15) * 1000), dtype=np.int32)
            # print("Vline: (%f, %f)" % (float(unit_vector[0]) / 200, float(unit_vector[1]) / 200))
            unit_vector = FixCoordinate_vector(unit_vector)
            # print(robotPos)
            tm_send_script_client("SetContinueVLine(%f, %f, 0, 0, 0, 0)" % (float(unit_vector[0]) / 200, float(unit_vector[1]) / 200))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    elif FLAG_realMove == False:
        robotPos += unit_vector  #simulate
        print("Vline: (%f, %f)" % (unit_vector[0], unit_vector[1]))
        time.sleep(0.1)


# 回原點取卯
def robot_pick(index: int):
    global status
    target = np.array([0, 0])  #給予初始值 避免全部都拿完了,target會找不到
    for i, p in enumerate(reached_pick_points):
        if (p == True):
            target = pick_points[i]
            pick_point_index = i
            break

    if np.linalg.norm(robotPos - target) < 8:
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
    handpos = np.array([np.sum(hand[:, 0]) / len(hand), np.sum(hand[:, 1]) / len(hand)], dtype=np.int32)
    handshape = hand - handpos


# 接收洞口更新資訊
def handle_hole_status(msg):
    # 0100000101111112101
    # global pick_point_index
    if len(msg.data) != 19:
        return
    for i, ch in enumerate(msg.data[:9]):
        if ch == "0":
            reached_place_points[i] = False
        else:
            reached_place_points[i] = True

    #拿的部份, 有我就選, 不管順序
    # for i, ch in enumerate(msg.data[9:]):
    #     if ch == "1":
    #         pick_point_index = i
    #         break


# 鼠標控制(測試用)
def mouse_move(event, x, y, flags, param):
    global handpos
    handpos = np.array((x, y), dtype=np.int32)


# 繪製圖面
def UpdateImage():
    global status
    global FLAG_realMove
    global unit_vector
    hand = handshape + handpos
    img_process = np.ones(frameSize, dtype=np.uint8) * 100
    cv2.fillPoly(img_process, [hand], (100, 0, 200))

    if status == "pick":
        if FLAG_realMove:
            cv2.circle(img_process, robotPos, 10, (181, 24, 37), 3)
        else:
            cv2.circle(img_process, robotPos, 10, (174, 214, 51), 3)
    elif status == "place":
        if FLAG_realMove:
            cv2.circle(img_process, robotPos, 10, (181, 24, 37), -1)
        else:
            cv2.circle(img_process, robotPos, 10, (174, 214, 51), -1)
    if (FLAG_realMove == True):
        unit_vector = FixCoordinate_vector(unit_vector)
    cv2.line(img_process, robotPos, robotPos + unit_vector * 20, (162, 33, 176), 3)
    cv2.line(img_process, robotPos, robot_target, (69, 8, 13), 1)

    # draw 9 square point
    for i, p in enumerate(place_points):
        if reached_place_points[i]:
            cv2.circle(img_process, p, 5, (100, 0, 100), -1)
            cv2.putText(img_process, str(i), (p[0], p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color=(100, 0, 100), thickness=1)
        else:
            cv2.circle(img_process, p, 5, (0, 200, 200), -1)
            cv2.putText(img_process, str(i), (p[0], p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color=(0, 200, 200), thickness=1)

    #沒什麼意義複製,暫時先用來區分 畫一些無意義的文字
    img_show = img_process.copy()
    if (FLAG_rotate == True):
        cv2.rotate(img_process, ROTATE_90_CLOCKWISE, img_show)
        cv2.putText(img_show, 'Rotate in view(r)', (100, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    else:
        cv2.putText(img_show, 'Rotate in real(r)', (100, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    if (FLAG_realMove == True):
        cv2.putText(img_show, 'Real(z)', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (181, 24, 37), 1, cv2.LINE_AA)
    else:
        cv2.putText(img_show, 'Simulate(z)', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (174, 214, 51), 1, cv2.LINE_AA)

    cv2.putText(img_show, status, (250, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 82, 200), 1, cv2.LINE_AA)
    cv2.imshow("map", img_show)
    cv2.imshow("map", img_show)


def Delay(sec):
    for _ in range(5 * sec):
        UpdateImage()
        cv2.waitKey(200)


# ============================================================================================== #

# ROS1實裝 #
#___Main___
if __name__ == "__main__":
    rospy.init_node('pathplan_node', anonymous=True)
    rospy.Subscriber("/hand_detector", String, handle_hand_detector)
    rospy.Subscriber("/hole_status", String, handle_hole_status)

    listener = tf.TransformListener()

    # ============================================================================================== #
    cv2.namedWindow('map')
    # cv2.namedWindow('robot cam')
    # cv2.setMouseCallback('map', mouse_move)
    set_points_to_robot_points()

    while not rospy.is_shutdown():
        if status == "init":  #robot back to initial point
            print('Initial')
            # tm_send_script_client("PTP(\"JPP\",0,0,90,0,90,0,35,200,0,false)")
            tm_send_script_client("StopAndClearBuffer(0)")
            tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700
            Delay(1)
            tm_send_script_client("PTP(\"JPP\",-16.7,-14.1,115.1,79.2,-106.7,180.2,35,200,0,false)")  #ready pos #TM5-700
            Delay(1)
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,35,200,0,false)" % (original_robot_pos[0] * 1000, original_robot_pos[1] * 1000, robot_z_high * 1000))
            Delay(2)
            tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
            tm_send_script_client("ContinueVLine(300, 500)")
            tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
            status = "pause"
        elif status == "pause":
            # print('Pause')
            #do notiong
            # time.sleep(0.1)
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
            tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
            tm_send_script_client("StopContinueVmode()")
            tm_send_script_client("PTP(\"JPP\",-16.7,-14.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  #TM5-700
            status = "pause"

        ###draw image###
        UpdateImage()

        ###key bind###
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('i'):
            status = "init"
        elif key == ord('p'):
            status = "pause"
        elif key == ord('s'):
            status = "start"
        elif key == ord('e'):
            status = "end"
        elif key == ord('a'):
            #find NG pin
            tm_send_script_client("PTP(\"JPP\",0.0,-20.0,90.0,150.0,-90.0,180.0,80,200,0,false)")  #camera pos #TM5-700
            tm_send_script_client('ScriptExit(0)')
            Delay(5)
            ng_status_msg = ng_detect_client()
            print(ng_status_msg)
            tm_send_script_client("PTP(\"JPP\",-16.7,-44.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  #out pos#TM5-700
            tm_send_script_client("PTP(\"JPP\",-16.7,-24.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  # higher pos #TM5-700
        elif key == ord('b'):
            #去除NG pin
            reached_pick_points = np.ones(len(pick_points), dtype=np.bool_)
            for i, ch in enumerate(ng_status_msg.data[:]):
                if ch == "2":
                    #trash it!
                    #set to can not reached
                    reached_pick_points[i] = False

        elif key == ord('z'):
            FLAG_realMove = not FLAG_realMove
        elif key == ord('r'):
            FLAG_rotate = not FLAG_rotate
        elif key == ord('m'):
            tm_send_gripper_client(True)
        elif key == ord('n'):
            tm_send_gripper_client(False)
        elif key == 176:  #numpad0
            print('Robot go to index 0 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[0][0] * 1000, place_points_in_robot_pos[0][1] * 1000, robot_z_high * 1000))
        elif key == 177:  #numpad1
            print('Robot go to index 1 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[1][0] * 1000, place_points_in_robot_pos[1][1] * 1000, robot_z_high * 1000))
        elif key == 178:  #numpad2
            print('Robot go to index 2 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[2][0] * 1000, place_points_in_robot_pos[2][1] * 1000, robot_z_high * 1000))
        elif key == 179:  #numpad3
            print('Robot go to index 3 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[3][0] * 1000, place_points_in_robot_pos[3][1] * 1000, robot_z_high * 1000))
        elif key == 180:  #numpad4
            print('Robot go to index 4 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[4][0] * 1000, place_points_in_robot_pos[4][1] * 1000, robot_z_high * 1000))
        elif key == 181:  #numpad5
            print('Robot go to index 5 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[5][0] * 1000, place_points_in_robot_pos[5][1] * 1000, robot_z_high * 1000))
        elif key == 182:  #numpad6
            print('Robot go to index 6 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[6][0] * 1000, place_points_in_robot_pos[6][1] * 1000, robot_z_high * 1000))
        elif key == 183:  #numpad7
            print('Robot go to index 7 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[7][0] * 1000, place_points_in_robot_pos[7][1] * 1000, robot_z_high * 1000))
        elif key == 184:  #numpad8
            print('Robot go to index 8 pos')
            tm_send_script_client("PTP(\"CPP\",%f,%f,%f,90.0,0,90,50,200,0,false)" % (place_points_in_robot_pos[8][0] * 1000, place_points_in_robot_pos[8][1] * 1000, robot_z_high * 1000))
        elif key == 185:  #numpad9
            print('Robot go to OUTSIDE pos')
            tm_send_script_client("PTP(\"JPP\",-16.7,-44.1,115.1,79.2,-106.7,180.2,80,200,0,false)")  #out pos#TM5-700

    ###node exit###
    tm_send_script_client("StopContinueVmode()")
    cv2.destroyAllWindows()
