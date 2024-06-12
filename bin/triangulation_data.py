
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time        :2022/7/29 9:28
# @Author      :weiz
# @ProjectName :Sfm-python-master
# @File        :triangulation_data.py
# @Description :
# Copyright (C) 2021-2025 Jiangxi Institute Of Intelligent Industry Technology Innovation
import numpy as np
 
 
point1 = [738, 389]  # 0.5410932
point1_gripper = [0.5122332451863898, 0.5128182769039566, 0.9630757531852082, -0.7115522167741687, -1.7916510038378088, 1.7013068394360287]
point2 = [797, 374]
point2_gripper = [0.3820852469525631, 0.5235437570806256, 0.9711089613285708, -0.7071726699555351, -1.9459688091738208, 1.7263544531858268]
 
# point1 = [797, 374]  # 0.54940385
# point1_gripper = [0.3820852469525631, 0.5235437570806256, 0.9711089613285708, -0.7071726699555351, -1.9459688091738208, 1.7263544531858268]
# point2 = [715, 343]
# point2_gripper = [0.5056936674804589, 0.4650883838281726, 1.0634729391460807, -0.6999271853483486, -1.749012465435611, 1.6882826312357782]
 
# point1 = [715, 343]  # 0.64878213
# point1_gripper = [0.5056936674804589, 0.4650883838281726, 1.0634729391460807, -0.6999271853483486, -1.749012465435611, 1.6882826312357782]
# point2 = [733, 368]
# point2_gripper = [0.5485518509025477, 0.5157901201998042, 0.9837568437877331, -0.6609672121210775, -1.7084733902118903, 1.7261963960916609]
 
# point1 = [733, 368]  # 0.5727386
# point1_gripper = [0.5485518509025477, 0.5157901201998042, 0.9837568437877331, -0.6609672121210775, -1.7084733902118903, 1.7261963960916609]
# point2 = [738, 389]
# point2_gripper = [0.5122332451863898, 0.5128182769039566, 0.9630757531852082, -0.7115522167741687, -1.7916510038378088, 1.7013068394360287]
 
 
def pixel_normalization(point1, point2):
    points_norm = []
    points_norm.append(point1)
    points_norm.append(point2)
    points_norm = np.array(points_norm)
    points_norm = (points_norm - np.mean(points_norm)) / np.std(points_norm)
    return points_norm[0], points_norm[1]
 
 
if __name__ == "__main__":
    p1, p2 = pixel_normalization(point1, point2)
    print(p1)
    print(p2)