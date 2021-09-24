#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np

class FuzzyCtr():
    def __init__(self, input1, input2, rules):
        """
        初始化
        :param input1: 控制变量1的n个取值如：[-5, -2, -1, 0, 1, 2, 5]
        :param input2: 控制变量2的n个取值
        :param rules: 规则矩阵,横向代表x(axis=1),纵向代表y(axis=0)
        """
        # 后面补充一个大数
        self.x_range = np.concatenate([np.array(input1), [input1[-1]+1]]).astype(float)
        self.y_range = np.concatenate([np.array(input2), [input2[-1]+1]]).astype(float)
        self.rules = np.array(rules).astype(float)

    def control(self, x, y):
        """ 计算控制变量 """
        # 限制幅值
        x = self.clump(x, self.x_range[0], self.x_range[-2])
        y = self.clump(y, self.y_range[0], self.y_range[-2])
        # 查找输入在rules矩阵里的坐标
        xi = self.search_idx(x, self.x_range)
        yi = self.search_idx(y, self.y_range)
        # 计算隶属度
        mu_x = np.array([self.x_range[xi+1]-x, x-self.x_range[xi]]) / [self.x_range[xi+1]-self.x_range[xi]]
        mu_y = np.array([self.y_range[yi+1]-y, y-self.y_range[yi]]) / [self.y_range[yi+1]-self.y_range[yi]]
        # 计算隶属度矩阵
        mu_matrix = np.matmul(mu_y[:, None], mu_x[None, :])
        # 计算输出
        rules_act = self.rules[yi:yi+2, xi:xi+2]
        output = np.sum(mu_matrix*rules_act)
        return output


    def search_idx(self, x, x_range):
        """ 若 x 在 x_range 中的第i个左开右闭区间[x_range[i], x_range[i+1])，返回i """
        for i in range(len(x_range)-1):
            if x >= x_range[i] and x < x_range[i+1]:
                return i


    def clump(self, x, low, high):
        """ 限制幅值 """
        if x < low:
            return low
        if x > high:
            return high
        return x


# 定义车道线循迹控制器
bias_range = [-50, -30, -15, 0, 15, 30, 50]
slope_range = [-5, -4, -3, -2, -1, 0, 1]
        # -50, -30, -15,   0,  15,  30, 50
rules = [[-40, -32, -25, -17, -13,  -10, -6],   # -5
         [-35, -25, -20, -14,  -9,  -6, -3],   # -4
         [-30, -22, -16, -9,   -6,  -3,  0],   # -3
         [-25, -17, -11, -6,  -3,   0,  5],  # -2
         [-24, -19, -13,  -3,   0,  5, 10],   # -1
         [-15, -10,  -5,   0,   5,  10, 15],   #  0
         [-10,  -5,   0,  5,  10,  15, 20]]     #  1


FollowLineCtr = FuzzyCtr(bias_range, slope_range, rules)


if __name__ == '__main__':

    for x in range(-30, 30, 5):
        for y in np.array(range(-5, 5))*0.4:

            print(x, y, FollowLineCtr.control(x, y))