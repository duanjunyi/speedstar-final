#! /usr/bin/python3.7

import cv2
import hilens as hl
import utils
from lane import laneDetect
from npsocket import NumpySocket
import threading
import numpy as np


class Hilens:
    def __init__(self, checkvalue, width, height):
        # 系统初始化，your_check_value需要替换成创建技能时填写的检验值保持一致
        hl.init(checkvalue)

        # 初始化摄像头
        self.camera = hl.VideoCapture(width=width, height=height)
        self.display = hl.Display(hl.HDMI)

    def read_img(self):
        input_yuv = self.camera.read()  # 读取一帧图片(YUV NV21格式)
        img_rgb = cv2.cvtColor(input_yuv, cv2.COLOR_YUV2RGB_NV21)  # 转为RGB格式
        return img_rgb

    def __del__(self):
        hl.terminate()


class ObjDetectThread(threading.Thread):  # 目标检测线程类
    def __init__(self, threadname, modelname):
        threading.Thread.__init__(self)
        self.name = threadname
        # 初始化模型，your_model_name需要替换为转换生成的om模型的名称
        model_path = hl.get_model_dir() + modelname
        self.model = hl.Model(model_path)

    def run(self):
        global input_img, bboxes
        while True:
            img_preprocess, img_w, img_h = utils.preprocess(input_img)  # 缩放为模型输入尺寸
            output = self.model.infer([img_preprocess.flatten()])  # 模型推理
            bboxes = utils.get_result(output, img_w, img_h)  # 获取检测结果
            # output_img = utils.draw_boxes(output_img, bboxes)  # 在图像上画框


class LaneDetectThread(threading.Thread):  # 车道线检测线程类
    def __init__(self, threadname, laneDet):
        threading.Thread.__init__(self)
        self.name = threadname
        self.laneDet = laneDet

    def run(self):
        global input_img, curve_rad, distance_from_center
        while True:
            curve_rad, distance_from_center = laneDet.spin(input_img)





if __name__ == "__main__":
    # 初始化通讯传递变量
    bboxes = []
    curve_rad= 0
    distance_from_center = 0

    # 创建socket
    socket_sender = NumpySocket()
    socket_sender.initialize_sender('192.168.2.1', 9999)  # 地址和端口

    # 初始化Hilens摄像头和模型
    # 摄像头
    frameWidth = 1280  # 宽
    frameHeight = 720  # 长
    hl_camera = Hilens('checkvalue', frameWidth, frameHeight)  # 根据实际情况指定
    input_img = hl_camera.read_img()  # 初始化图像
    output_img = input_img

    # 初始化车道线检测常量
    # 相机内参
    camMat = np.array([[6.678151103217834e+02, 0, 6.430528691213178e+02],
                       [0, 7.148758960098705e+02, 3.581815819255082e+02], [0, 0, 1]])  # 相机校正矩阵
    camDistortion = np.array([[-0.056882894892153, 0.002184364631645, -0.002836821379133, 0, 0]])  # 相机失真矩阵
    # 透视变换
    src_points = np.array([[0., 527.], [416., 419.], [781., 420.], [1065., 542.]], dtype="float32")  # 源点
    dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")  # 目标点
    # src_points = np.array([[498., 596.], [789., 596.], [250., 720.], [1050., 720.]], dtype="float32")  # 源点
    # dst_points = np.array([[300., 100.], [980., 100.], [300., 720.], [980., 720.]], dtype="float32")  # 目标点
    MWarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算
    # 视觉处理
    kerSz = (3, 3)  # 膨胀与腐蚀核大小
    grayThr = 125  # 二值化阈值
    roiXRatio = 0.4  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置
    nwindows = 20  # 窗的数目
    window_width = 200  # 窗的宽度
    minpix = 200  # 最小连续像素，小于该长度的被舍弃以去除噪声影响
    # 距离映射
    x_cmPerPixel = 90 / 665.00  # x方向上一个像素对应的真实距离 单位：cm
    y_cmPerPixel = 81 / 680.00  # y方向上一个像素对应的真实距离 单位：cm
    roadWidth = 80  # 道路宽度 单位：cm
    y_offset = 50.0  # 由于相机位置较低，识别到的车道线距离车身较远，不是当前位置，定义到的车道线与车身距离 单位：cm<no usage>
    cam_offset = 18.0  # 相机中心与车身中轴线的距离 单位：cm
    isShow = False  # 是否显示
    laneDet = laneDetect(MWarp, camMat, camDistortion, kerSz, grayThr, frameHeight, frameWidth, roiXRatio, window_width,
                 nwindows, minpix, x_cmPerPixel, y_cmPerPixel, roadWidth, isShow)  # 初始化车道线检测


    # 初始化线程
    objThread = ObjDetectThread('objDetect', 'your_model_name.om')  # 目标检测线程
    laneThread = LaneDetectThread('laneDetect', laneDet)  # 车道线检测线程

    #启动线程
    objThread.start()
    laneThread.start()
    while True:
        input_img = hl_camera.read_img()  # 用来处理的图
        # output_img = input_img  # 用来显示的图
        # 发送代码，待补充



