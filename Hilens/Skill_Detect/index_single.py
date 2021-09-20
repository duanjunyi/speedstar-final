#! /usr/bin/python3.7

import cv2
import hilens
from utils import preprocess, draw_boxes, get_result
from lane import laneDetect
from npsocket import NumpySocket
import numpy as np


class Hilens:
    def __init__(self, checkvalue):
        # 系统初始化，your_check_value需要替换成创建技能时填写的检验值保持一致
        hilens.init(checkvalue)

        # 初始化摄像头
        self.camera = hilens.VideoCapture()
        self.display = hilens.Display(hilens.HDMI)

    def read_img(self):
        input_yuv = self.camera.read()  # 读取一帧图片(YUV NV21格式)
        input_img = cv2.cvtColor(input_yuv, cv2.COLOR_YUV2RGB_NV21)  # 转为RGB格式
        return input_img

    def show(self, img, bboxes):
        img = draw_boxes(img, bboxes)  # 画框
        output_yuv = hilens.cvt_color(img, hilens.RGB2YUV_NV21)
        self.display.show(output_yuv)  # 显示到屏幕上

    def __del__(self):
        hilens.terminate()


class ObjDetect():
    def __init__(self, modelname):
        # 初始化模型，your_model_name需要替换为转换生成的om模型的名称
        model_path = hilens.get_model_dir() + modelname
        self.model = hilens.Model(model_path)

    def run(self, img):
        img_preprocess, img_w, img_h = preprocess(img)  # 缩放为模型输入尺寸
        output = self.model.infer([img_preprocess.flatten()])  # 模型推理
        bboxes = get_result(output, img_w, img_h)  # 获取检测结果
        return bboxes


def run():
    socket_sender = NumpySocket()
    socket_sender.initialize_sender('192.168.2.1', 9999)
    # 初始化Hilens摄像头和模型
    # 摄像头
    hl_camera = Hilens('detect')  # 根据实际情况指定

    # 初始化车道线检测常量
    frameWidth = 1280  # 宽
    frameHeight = 720  # 高
    # 透视变换
    # 快速绕圈
    # src_points = np.array([[498., 596.], [789., 596.], [250., 720.], [1050., 720.]], dtype="float32")  # 源点
    # dst_points = np.array([[300., 100.], [980., 100.], [300., 720.], [980., 720.]], dtype="float32")  # 目标点
    src_points = np.array([[399, 640], [872, 635], [261, 710], [998, 713]], dtype="float32")
    dst_points = np.array([[376, 24], [893, 24], [366, 699], [895, 702]], dtype="float32")
    Mwarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算
    camMat = np.array([[6.678151103217834e+02, 0, 6.430528691213178e+02],
                       [0, 7.148758960098705e+02, 3.581815819255082e+02], [0, 0, 1]])  # 相机校正矩阵
    camDistortion = np.array([[-0.056882894892153, 0.002184364631645, -0.002836821379133, 0, 0]])  # 相机失真矩阵

    # 视觉处理
    kerSz = (3, 3)  # 膨胀与腐蚀核大小
    grayThr = 125  # 二值化阈值
    roiXRatio = 0.4  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置

    # 划窗检测
    winWidth = 200  # 窗的宽度
    winNum = 20  # 窗的数目
    winThr = 8  # 单条车道线需有8个框检出车道线
    pixThr = 200  # 最小连续像素，小于该长度的被舍弃以去除噪声影响

    # 距离映射
    roadWidCm = 80  # 道路宽度 单位：cm
    roadWidPix = 660  # 透视变换后车道线像素数
    isShow = True  # 是否返回可视化图片
    laneDet = laneDetect(Mwarp, camMat, camDistortion, kerSz, grayThr, frameHeight, frameWidth, roiXRatio,
                winWidth, winNum, winThr, pixThr, roadWidCm, roadWidPix, isShow)  # 初始化车道线检测

    # 初始化目标检测类
    objDet = ObjDetect('yolov3_new.om')

    while True:
        input_img = hl_camera.read_img()  # 读取图像
        bboxes = objDet.run(input_img)  # 目标检测
        distance_from_center, img_show_real = laneDet.spin(input_img)  # 车道线检测
        hl_camera.show(img_show_real, bboxes)  # 显示目标检测框和车道线
        send_msg = socket_sender.wrap(bboxes, distance_from_center)
        socket_sender.send_array(send_msg)


if __name__ == "__main__":
    run()

