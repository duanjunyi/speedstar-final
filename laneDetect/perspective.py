"""
计算图片透视变换矩阵：
    R: 重新选择源点和目标点
    D: 重新选择目标点
    ESC: 退出
"""

import cv2
import numpy as np
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parent
FONT = cv2.FONT_HERSHEY_SIMPLEX

# 图片路径
# img_path = BASE_DIR / 'chessboard1.png'
img_path = BASE_DIR / 'video/快速绕圈.mp4'

def imread(img_path):
    """ 读入图片或视频第一帧 """
    assert img_path.exists(), 'File does not exist'
    if img_path.suffix in ['.jpg', '.png']:
        return cv2.imread(str(img_path))
    elif img_path.suffix == '.mp4':
        cap = cv2.VideoCapture(str(img_path))  # 读入视频
        ret, img  = cap.read()
        if ret:
            cap.release()
            return img

# 读入图片
img = imread(img_path)
img_src = img.copy()
img_dst = np.zeros_like(img)

pts_cnt = [0, 0]# 标记源点数, 目标点数
src_points = np.array([[498., 596.], [789., 596.], [250., 720.], [1050., 720.]], dtype="float32")   # 源点
dst_points = np.array([[300., 100.], [980., 100.], [300., 720.], [980., 720.]], dtype="float32")    # 目标点


def draw_src_pts(event, x, y, flags, param):
    """ 左键单击画点 """
    if event == cv2.EVENT_LBUTTONDOWN and pts_cnt[0] < 4:
        cv2.circle(img_src, (x,y), 5, (0, 0, 255), -1)
        src_points[pts_cnt[0]] = [x, y]
        pts_cnt[0] += 1
        cv2.putText(img_src, str(pts_cnt[0]), (x+10, y), FONT, 1, (0, 0, 255))

def draw_dst_pts(event, x, y, flags, param):
    """ 左键单击画点 """
    global img_dst
    if event == cv2.EVENT_LBUTTONDOWN and pts_cnt[0]==4 and pts_cnt[1] < 4:
        cv2.circle(img_dst, (x,y), 5, (0, 0, 255), -1)
        dst_points[pts_cnt[1]] = [x, y]
        pts_cnt[1] += 1
        cv2.putText(img_dst, str(pts_cnt[1]), (x+10, y), FONT, 1, (0, 0, 255))

    if event == cv2.EVENT_LBUTTONDOWN and pts_cnt[1] == 4:     # 显示投影变换，并输出
        MWarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算
        img_dst = cv2.warpPerspective(img, MWarp, (img.shape[1], img.shape[0]), cv2.INTER_LINEAR)  # 透视变换
        for i in range(4): # 画点
            px, py = dst_points[i].astype(int)
            cv2.circle(img_dst, (px, py), 5, (0, 0, 255), -1)
            cv2.putText(img_dst, str(i+1), (px+10, py ), FONT, 1, (0, 0, 255))
        # 输出
        fmt = 'np.array([[{:d}, {:d}], [{:d}, {:d}], [{:d}, {:d}], [{:d}, {:d}]], dtype="float32")'
        src_str = 'src_points = ' + fmt.format(*[x for x in src_points.astype(int).flat])  # 源点
        dst_str = 'dst_points = ' + fmt.format(*[x for x in dst_points.astype(int).flat])  # 目标点
        print(src_str)
        print(dst_str)
        print('MWarp = ')
        print(MWarp)

if __name__ == '__main__':
    cv2.namedWindow('src')
    cv2.namedWindow('dst')
    cv2.setMouseCallback('src', draw_src_pts)
    cv2.setMouseCallback('dst', draw_dst_pts)

    while True:
        cv2.imshow('src',img_src)
        cv2.imshow('dst',img_dst)
        c = cv2.waitKey(10)
        if c == 27:                         # ESC退出
            break
        elif c in [ord('r'), ord('R')]:     # r 重绘
            img_src = img.copy()
            img_dst = np.zeros_like(img)
            pts_cnt = [0, 0]
        elif c in [ord('d'), ord('D')]:     # d 重绘目标点
            img_dst = np.zeros_like(img)
            pts_cnt[1] = 0

    cv2.destroyAllWindows()
