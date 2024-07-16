import sensor, image, time, math
import Maix
from fpioa_manager import fm
from machine import UART
from Maix import GPIO
from micropython import const

#LED
fm.register(9, fm.fpioa.GPIO0)
fm.register(17, fm.fpioa.GPIO1)
led_g=GPIO(GPIO.GPIO0, GPIO.OUT)
led_r=GPIO(GPIO.GPIO1, GPIO.OUT)
led_g.value(1)

print("Current CPU Frequency: ", Maix.freq.get_cpu())
print("Current KPU Frequency: ", Maix.freq.get_kpu())

#定数定義
UART_SPEED = const(230400)
ANGLE_CONVERSION = 0.28125
GOAL_ANGLE_CONVERSION = 3.55555
WIDTH = const(320)
HEIGHT = const(184)
PROXIMITY_HEIGHT = const(110)

#ホモグラフィー変換行列
homography_matrix = [[ 6.25000, -4.38017, -1.04083 ],
                     [ 0.00000,  9.19117,  0.00000 ],
                     [ 2.37169, -5.51470,  1.00000 ]]

#センサーの設定
sensor.reset(freq = 24000000, set_regs = True, dual_buff = True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.set_hmirror(1)
sensor.set_windowing((WIDTH, HEIGHT))

sensor.set_auto_gain(False, gain_db = 0)
sensor.set_auto_whitebal(False, rgb_gain_db = (25, 20, 40))
sensor.set_auto_exposure(False)
sensor.set_contrast(0)
sensor.set_saturation(0)
sensor.set_brightness(0)
sensor.run(1)

camera_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, camera_gain)

#UART設定
fm.register(10, fm.fpioa.UART1_TX, force = True)
fm.register(11, fm.fpioa.UART1_RX, force = True)

uart = UART(UART.UART1, UART_SPEED, 8, None, 1, timeout = 1000, read_buf_len = 4096)

#各閾値
ball_thresholds = [(0, 100, 30, 60, 30, 60)]
y_goal_thresholds = [(0, 100, -20, 15, 21, 69)]
b_goal_thresholds = [(0, 100, 16, 64, -106, -58)]
court_thresholds = [(0, 100, -35, -6, -14, 10)]

ball_roi_cut_top = const(14)
court_roi_cut_top = const(14)
goal_roi_cut_bottom = const(84)
ball_roi = [0, ball_roi_cut_top, WIDTH, HEIGHT - ball_roi_cut_top]
court_roi = [0, court_roi_cut_top, WIDTH, HEIGHT - ball_roi_cut_top]
goal_roi = [0, 0, WIDTH, HEIGHT - goal_roi_cut_bottom]

led_g.value(0)

def MatrixMultiply(matrix, vector): #行列の積を計算する関数
    result = [[sum(a * b for a, b in zip(row, col)) for col in zip(*vector)] for row in matrix]
    return result

def CheckGreen(x, y):
    def Check(thresholds, lab):
        l_min, l_max, a_min, a_max, b_min, b_max = thresholds[0]
        return 1 if l_min < lab[0] < l_max and a_min < lab[1] < a_max and b_min < lab[2] < b_max else 0

    bit = []
    for dx in [-3, 0, 3]:
        for dy in [-3, 0, 3]:
            bit.append(Check(court_thresholds, image.rgb_to_lab(img.get_pixel(x + dx, y + dy))))

    return 0 if any(bit) else 1

def HomographyProjection(center_x, center_y): #ホモグラフィー変換をする関数
    #カメラ座標でのコートベクトル
    camera_vector = [[center_x],
                     [center_y],
                     [1       ]]

    #ボールベクトルをワールド座標に変換
    coordinate = MatrixMultiply(homography_matrix, camera_vector)
    world_vector = [[coordinate[0][0] / coordinate[2][0]],
                    [coordinate[1][0] / coordinate[2][0]]]
    return world_vector

while True:
    img = sensor.snapshot() #映像の取得

    #ボールを見つける
    ball_rectarray = []
    ball_x = 0
    ball_y = 0
    is_ball_area = 0
    for blob in img.find_blobs(ball_thresholds, roi = ball_roi, pixel_threshold = 10, area_threshold = 10, merge = True, margin = 10):
        if blob[2] < 150:
            ball_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納
            rect = list(blob.rect())
            if 135 < rect[0] < 185 and 170 < rect[1]:
                is_ball_area = 1

    try:
        ball_maxrect = max(ball_rectarray, key = lambda x: x[1])    #配列の中から一番画面の下にあるものを選定
        ball_x = ball_maxrect[0] + (ball_maxrect[2] * 0.5)  #中心のx座標の算出
        ball_y = ball_maxrect[1] + (ball_maxrect[3] * 0.5)  #中心のy座標の算出
        #img.draw_circle(int(ball_x), int(ball_y), int((ball_maxrect[2] * 0.5 + ball_maxrect[3] * 0.5) * 0.5))

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #黄色ゴールを見つける
    y_goal_rectarray = []
    y_goal_x = 0
    y_goal_width = 0
    y_goal_hight = 0

    for blob in img.find_blobs(y_goal_thresholds, roi = goal_roi, pixel_threshold = 100, area_threshold = 100, merge = True, margin = 50):
        y_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        y_goal_maxrect = max(y_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        y_goal_x = y_goal_maxrect[0] + (y_goal_maxrect[2] * 0.5)  #中心のx座標の算出
        y_goal_width = y_goal_maxrect[2]
        y_goal_hight = y_goal_maxrect[3]
        #img.draw_rectangle(y_goal_maxrect)     #オブジェクトを囲う四角形の描画
        #img.draw_string(y_goal_maxrect[0], y_goal_maxrect[1] - 12, "yellow goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #青色ゴールを見つける
    b_goal_rectarray = []
    b_goal_x = 0
    b_goal_width = 0
    b_goal_hight = 0

    for blob in img.find_blobs(b_goal_thresholds, roi = goal_roi, pixel_threshold = 100, area_threshold = 100, merge = False):
        b_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        b_goal_maxrect = min(b_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        b_goal_x = b_goal_maxrect[0] + (b_goal_maxrect[2] * 0.5)  #中心のx座標の算出
        b_goal_width = b_goal_maxrect[2]
        b_goal_hight = b_goal_maxrect[3]
        #img.draw_rectangle(b_goal_maxrect)     #オブジェクトを囲う四角形の描画
        #img.draw_string(b_goal_maxrect[0], b_goal_maxrect[1] - 12, "blue goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    court_rectarray = []
    court_x = 0
    court_y = 0

    for blob in img.find_blobs(court_thresholds, roi = court_roi, pixel_threshold = 1000, area_threshold = 1000, merge = True, margin = 1000):
        court_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        court_maxrect = max(court_rectarray, key = lambda x: x[2] * x[3])  # Y座標が一番大きい要素を選定
        court_x = court_maxrect[0] + (court_maxrect[2] * 0.5)  #中心のx座標の算出
        court_y = court_maxrect[1]
        #img.draw_line(0, court_maxrect[1], 320, court_maxrect[1])     #オブジェクトを囲う四角形の描画
        #img.draw_string(court_maxrect[0], court_maxrect[1] - 12, "court")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    world_court_vector = HomographyProjection(court_x - 160, 184 - court_y)

    #ボールベクトルの大きさ
    court_dis = int(world_court_vector[1][0] * 0.5)

    #範囲保証
    if court_dis > 50:
        court_dis = 50
    if court_y == 0:
        court_dis = 0

    #取得した値を変換
    ball_dir = int(ball_x * ANGLE_CONVERSION) #ボールの角度を求める

    #カメラ座標でのボールベクトル
    world_ball_vector = HomographyProjection(ball_x - 160, 184 - ball_y)

    #ボールベクトルの大きさ
    ball_dis = int(math.sqrt((world_ball_vector[0][0] - (ball_x - 160) * 0.0625) ** 2 + world_ball_vector[1][0] ** 2) * 2)

    #範囲保証
    if(ball_dis > 200):
        ball_dis = 200
    if ball_x == 0 or ball_y == 0:
        ball_dis = 0

    y_goal_dir = int(y_goal_x / GOAL_ANGLE_CONVERSION)
    y_goal_hight = int(y_goal_hight)

    b_goal_dir = int(b_goal_x / GOAL_ANGLE_CONVERSION)
    b_goal_hight = int(b_goal_hight)

    is_goal_front = 0
    own_dir = 0

    if(uart.any()):#UART受信
        own_dir = uart.readchar() * 2 - 180
    robot_center = 160 + own_dir * GOAL_ANGLE_CONVERSION

    if(y_goal_hight > b_goal_hight): #黄ゴールが見つかった時
        is_y_goal = 1
        goal_dir = y_goal_dir
        goal_size = y_goal_hight
        if(y_goal_x + (y_goal_width * 0.5) < robot_center + 10 or y_goal_x - (y_goal_width * 0.5) > robot_center - 10):
            is_goal_front = 0
        else:
            is_goal_front = 1
    else: #青ゴールが見つかった時
        is_y_goal = 0
        goal_dir = b_goal_dir
        goal_size = b_goal_hight
        if(b_goal_x + (b_goal_width * 0.5) < robot_center + 10 or b_goal_x - (b_goal_width * 0.5) > robot_center - 10):
            is_goal_front = 0
        else:
            is_goal_front = 1

    bool_data = (court_dis << 2) | (is_goal_front << 1) | is_y_goal

    #擬似LiDAR
    proximity_data = sum(CheckGreen(x, PROXIMITY_HEIGHT) << (6 - i) for i, x in enumerate(range(40, 301, 40)))
    #for x in range(40, 301, 40):
        #img.draw_cross(x, PROXIMITY_HEIGHT)

    #uart送信
    send_data = bytearray([0xFF, ball_dir, ball_dis, goal_dir, goal_size, bool_data, proximity_data, 0xAA])
    uart.write(send_data)
