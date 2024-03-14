from fpioa_manager import fm
from machine import UART
from Maix import GPIO
import sensor, image, time, math

#センサーの設定
sensor.reset(freq = 24000000, set_regs = True, dual_buff = True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.set_hmirror(1)
sensor.set_windowing((320, 184))

sensor.set_auto_gain(False, gain_db = 0)
sensor.set_auto_whitebal(False, rgb_gain_db = (25, 20, 40))
#sensor.set_auto_whitebal(False, rgb_gain_db = (25, 20, 40))
sensor.set_auto_exposure(False)
sensor.set_contrast(0)
sensor.set_saturation(0)
sensor.set_brightness(0)
sensor.run(1)

camera_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, camera_gain)

#定数定義
UART_SPEED = 115200
ANGLE_CONVERSION = 3.55555
ANGLE_Y_CONVERSION = 2.66666

# 画像座標
camera_x = 320  # 画像の中心からのX座標オフセット (ピクセル)
camera_y = 360  # 画像の中心からのY座標オフセット (ピクセル)
camera_height = 15
camera_tilt_angle = math.radians(62.5)


#UART設定
fm.register(10, fm.fpioa.UART1_TX, force = True)
fm.register(11, fm.fpioa.UART1_RX, force = True)

uart = UART(UART.UART1, UART_SPEED, 8, None, 1, timeout = 1000, read_buf_len = 4096)

#LED
fm.register(9, fm.fpioa.GPIO0)
fm.register(17, fm.fpioa.GPIO1)
led_g=GPIO(GPIO.GPIO0, GPIO.OUT)
led_r=GPIO(GPIO.GPIO1, GPIO.OUT)

#各閾値
ball_thresholds = [(0, 100, 10, 60, 20, 70)]
y_goal_thresholds = [(0, 100, -30, 10, 25, 75)]
b_goal_thresholds = [(0, 100, 20, 70, -110, -60)]
g_cort_thresholds = [(38, 7, -35, -6, -14, 10)]

#ball_thresholds = [(0, 100, 43, 84, 45, 94)]
#y_goal_thresholds = [(0, 100, 13, 44, 58, 84)]
#b_goal_thresholds = [(0, 100, 13, 37, -67, 3)]

ball_tracking_roi = [0, 8, 320, 176]
goal_tracking_roi = [0, 0, 320, 100]

while True:
    img = sensor.snapshot() #映像の取得

    #ボールを見つける
    ball_rectarray = []
    ball_x = 0
    ball_y = 0

    for blob in img.find_blobs(ball_thresholds, roi = ball_tracking_roi, pixel_threshold = 10, area_threshold = 10, merge = True, margin = 10):
        if(blob[2] < 75):
            ball_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        ball_maxrect = max(ball_rectarray, key = lambda x: x[1])    #配列の中から一番画面の下にあるものを選定
        ball_x = ball_maxrect[0] + (ball_maxrect[2] / 2)  #中心のx座標の算出
        ball_y = ball_maxrect[1] + (ball_maxrect[3] / 2)  #中心のy座標の算出
        img.draw_circle(int(ball_x), int(ball_y), int((ball_maxrect[2] / 2 + ball_maxrect[3] / 2) / 2))
        img.draw_string(ball_maxrect[0], ball_maxrect[1] - 12, "ball")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    is_goal_front = 0

    #黄色ゴールを見つける
    y_goal_rectarray = []
    y_goal_x = 0
    y_goal_y = 0
    b_goal_width = 0
    y_goal_hight = 0

    for blob in img.find_blobs(y_goal_thresholds, roi = goal_tracking_roi, pixel_threshold = 100, area_threshold = 100, merge = False):
        y_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        y_goal_maxrect = max(y_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        y_goal_x = y_goal_maxrect[0] + (y_goal_maxrect[2] / 2)  #中心のx座標の算出
        y_goal_y = y_goal_maxrect[1] + (y_goal_maxrect[3] / 2)  #中心のy座標の算出
        y_goal_width = y_goal_maxrect[2]
        y_goal_hight = y_goal_maxrect[3]
        img.draw_rectangle(y_goal_maxrect)     #オブジェクトを囲う四角形の描画
        img.draw_string(y_goal_maxrect[0], y_goal_maxrect[1] - 12, "yellow goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #青色ゴールを見つける
    b_goal_rectarray = []
    b_goal_x = 0
    b_goal_y = 0
    b_goal_width = 0
    b_goal_hight = 0

    for blob in img.find_blobs(b_goal_thresholds, roi = goal_tracking_roi, pixel_threshold = 100, area_threshold = 100, merge = False):
        b_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        b_goal_maxrect = min(b_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        b_goal_x = b_goal_maxrect[0] + (b_goal_maxrect[2] / 2)  #中心のx座標の算出
        b_goal_y = b_goal_maxrect[1] + (b_goal_maxrect[3] / 2)  #中心のy座標の算出
        b_goal_width = b_goal_maxrect[2]
        b_goal_hight = b_goal_maxrect[3]
        img.draw_rectangle(b_goal_maxrect)     #オブジェクトを囲う四角形の描画
        img.draw_string(b_goal_maxrect[0], b_goal_maxrect[1] - 12, "blue goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass


    g_cort_rectarray = []
    g_cort_x = 0
    g_cort_y = 0

    for blob in img.find_blobs(g_cort_thresholds, roi = goal_tracking_roi, pixel_threshold = 100, area_threshold = 100, merge = True, margin = 20):
        g_cort_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        g_cort_high = max(g_cort_rectarray, key=lambda x: x[1])  # Y座標が一番大きい要素を選定
        g_cort_y = g_cort_high[1]
        img.draw_rectangle(g_cort_high)     #オブジェクトを囲う四角形の描画
        img.draw_string(g_cort_high[0], g_cort_high[1] - 12, "cort")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #取得した値を変換
    ball_dir = int(ball_x / ANGLE_CONVERSION)
    ball_y_dir = int((ball_y / ANGLE_Y_CONVERSION))
    def distance_vector(camera_x, camera_y, ball_x, ball_y, camera_height):
        # カメラの座標から行列を作成
        camera_matrix = [
            [1, 0, 0],
            [0, 1, 0],
            [0, camera_height, 0]
        ]
        # カメラ座標とボール座標をベクトルとして表現
        camera_vector = [
            [camera_x - ball_x],
            [camera_y - ball_y],
            [1]  # Z座標は常に1と仮定
        ]

        # 行列とベクトルの掛け算
        result_vector = matrix_multiply(camera_matrix, camera_vector)

        # 結果のベクトルからXZ平面上の距離を取得
        distance = math.sqrt(result_vector[0][0] ** 2 + result_vector[2][0] ** 2)

        return distance

    # 行列とベクトルの掛け算を行う関数
    def matrix_multiply(matrix, vector):
        result = [[sum(a * b for a, b in zip(row, col)) for col in zip(*vector)] for row in matrix]
        return result

    # XZ平面上の距離ベクトルを計算
    ball_dis_1 = int(((abs(100-(distance_vector(camera_x, camera_y, ball_x, ball_y, camera_height)/100))*10/7)))
    def logistic_function(x):
        return 100 / (1 + math.exp(-0.1 * (x - 50)))
    ball_dis = int((logistic_function(ball_dis_1)-75)*4)
    if(ball_dis > 100):
        ball_dis = 100
    if ball_dis <= 0:
        ball_dis = 0
    if ball_x == 0 or ball_y == 0:# ボールがない場合は距離を0として返す
        ball_dis = 0


    y_goal_dir = int(y_goal_x / ANGLE_CONVERSION)
    y_goal_hight = int(y_goal_hight);

    b_goal_dir = int(b_goal_x / ANGLE_CONVERSION)
    b_goal_hight = int(b_goal_hight);

    if(y_goal_hight > b_goal_hight):
        is_y_goal = 1
        goal_dir = y_goal_dir
        goal_size = y_goal_hight
        if(y_goal_x + (y_goal_width / 2) < 170 or y_goal_x - (y_goal_width / 2) > 150):
            is_goal_front = 0
        else:
            is_goal_front = 1
    else:
        is_y_goal = 0
        goal_dir = b_goal_dir
        goal_size = b_goal_hight
        if(b_goal_x + (b_goal_width / 2) < 170 or b_goal_x - (b_goal_width / 2) > 150):
            is_goal_front = 0
        else:
            is_goal_front = 1

    g_cort_dis = int(g_cort_y/2)



    bool_data = (is_goal_front << 1) | is_y_goal

    #uart
    send_data = bytearray([0xFF, ball_dir, ball_dis, goal_dir, goal_size, bool_data, 0xAA])
    uart.write(send_data)
    print(g_cort_dis)
