#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
拍摄视频，保存在out_videos/中
"""
import cv2
import datetime
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parent
SAVE_DIR = BASE_DIR / 'out_videos'
SAVE_DIR.mkdir(exist_ok=True)


def out_name():
    return str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M')) + '.mp4'


if __name__ == '__main__':
    cap = cv2.VideoCapture('/dev/video10')
    frame_cnt = 0
    while frame_cnt < 1000:  # 保存1000帧
        ret, frame = cap.read()
        if ret:
            if frame_cnt==0: # 为视频第一帧
                frame_cnt += 1
                out_file = str(SAVE_DIR / out_name())
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                height, width = frame.shape[:2]
                out_writer = cv2.VideoWriter(out_file, fourcc, 24, (width, height), True)
                print('Start Recording')

            else:
                out_writer.write(frame)
                frame_cnt += 1


    out_writer.release()
    print(f'Save video at: {out_file}')



