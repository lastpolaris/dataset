import argparse
import multiprocessing
import os

import cv2

import numpy as np
import math
import sys
import shutil
from tqdm import tqdm 
import json

def triangulation(points_Picked):
    A = []
    for X in points_Picked:
        M1 = X[1][0:3]
        A.append(X[0][0] * M1[2] - M1[0])
        A.append(X[0][1] * M1[2] - M1[1])
        
    # print(np.array(A))
    U, sigma, V = np.linalg.svd(A)
    p_1 = [V[3][0], V[3][1], V[3][2], V[3][3]]
    p_2 = [V[3][0] / V[3][3], V[3][1] / V[3][3], V[3][2] / V[3][3],V[3][3] / V[3][3]]
    return p_2

def projection_points(point, projection):
    points_2d = np.array(projection @ point)
    x = (points_2d[0] / points_2d[2]).round()
    y = (points_2d[1] / points_2d[2]).round()
    return int(x),int(y)


def main():
    points_Picked = []
    
    img1 = cv2.imread('./test/Image115_c0.png')
    with open('./camera0.json') as metajson:
        data1 = json.load(metajson)
    camera_intrinsic1 = data1['camera_intrinsic']
    global_camera1 =  data1['global_camera']
    projection1 = np.array(camera_intrinsic1) @ np.array(global_camera1)
    points1 = np.array(cv2.selectROI("select", img1, False,False)[:2])
    points_Picked.append([points1,projection1])


    img2 = cv2.imread('./test/Image115_c1.png')
    with open('./camera1.json') as metajson:
        data2 = json.load(metajson)
    camera_intrinsic2 = data2['camera_intrinsic']
    global_camera2 =  data2['global_camera']
    projection2 = np.array(camera_intrinsic2) @ np.array(global_camera2)
    points2 = np.array(cv2.selectROI("select", img2, False,False)[:2])
    points_Picked.append([points2,projection2])
    p3d = triangulation(points_Picked)

    print(p3d)
    
    img1 = cv2.circle(img1, projection_points(p3d,projection1), 5,
                                 (0, 255, 0), -1, 16)
    img2 = cv2.circle(img2, projection_points(p3d,projection2), 5,
                                 (0, 255, 0), -1, 16)
    img3 = cv2.imread('./test/Image115_c2.png')
    with open('./camera2.json') as metajson:
        data3 = json.load(metajson)
    camera_intrinsic3 = data3['camera_intrinsic']
    global_camera3 =  data3['global_camera']
    projection3 = np.array(camera_intrinsic3) @ np.array(global_camera3)
    img3 = cv2.circle(img3, projection_points(p3d,projection3), 5,
                                 (0, 255, 0), -1, 16)
    
    img4 = cv2.imread('./test/Image115_c3.png')
    with open('./camera3.json') as metajson:
        data4 = json.load(metajson)
    camera_intrinsic4= data4['camera_intrinsic']
    global_camera4 =  data4['global_camera']
    projection4 = np.array(camera_intrinsic4) @ np.array(global_camera4)
    img4 = cv2.circle(img4, projection_points(p3d,projection4), 5,
                                 (0, 255, 0), -1, 16)
    cv2.imshow('img1',img1)
    cv2.imshow('img2',img2)
    cv2.imshow('img3',img3)
    cv2.imshow('img4',img4)
    cv2.waitKey()
def main2(tri_img1,tri_img2,test_img1,test_img2):
    points_Picked = []
    
    img1 = cv2.imread(f'./test/{tri_img1}_225.png')
    with open(f'./{tri_img1}.json') as metajson:
        data1 = json.load(metajson)
    camera_intrinsic1 = data1['camera_intrinsic']
    global_camera1 =  data1['global_camera']
    projection1 = np.array(camera_intrinsic1) @ np.array(global_camera1)
    points1 = np.array(cv2.selectROI("select", img1, False,False)[:2])
    points_Picked.append([points1,projection1])


    img2 = cv2.imread(f'./test/{tri_img2}_225.png')
    with open(f'./{tri_img2}.json') as metajson:
        data2 = json.load(metajson)
    camera_intrinsic2 = data2['camera_intrinsic']
    global_camera2 =  data2['global_camera']
    projection2 = np.array(camera_intrinsic2) @ np.array(global_camera2)
    points2 = np.array(cv2.selectROI("select", img2, False,False)[:2])
    points_Picked.append([points2,projection2])
    p3d = triangulation(points_Picked)

    print(p3d)
    
    img1 = cv2.circle(img1, projection_points(p3d,projection1), 5,
                                 (0, 255, 0), -1, 16)
    img2 = cv2.circle(img2, projection_points(p3d,projection2), 5,
                                 (0, 255, 0), -1, 16)
    img3 = cv2.imread(f'./test/{test_img1}_225.png')
    with open(f'./{test_img1}.json') as metajson:
        data3 = json.load(metajson)
    camera_intrinsic3 = data3['camera_intrinsic']
    global_camera3 =  data3['global_camera']
    projection3 = np.array(camera_intrinsic3) @ np.array(global_camera3)
    img3 = cv2.circle(img3, projection_points(p3d,projection3), 5,
                                 (0, 255, 0), -1, 16)
    
    img4 = cv2.imread(f'./test/{test_img2}_225.png')
    with open(f'./{test_img2}.json') as metajson:
        data4 = json.load(metajson)
    camera_intrinsic4= data4['camera_intrinsic']
    global_camera4 =  data4['global_camera']
    projection4 = np.array(camera_intrinsic4) @ np.array(global_camera4)
    img4 = cv2.circle(img4, projection_points(p3d,projection4), 5,
                                 (0, 255, 0), -1, 16)
    
    image1_resized = cv2.resize(img1, (640, 360))
    image2_resized = cv2.resize(img2, (640, 360))
    image3_resized = cv2.resize(img3, (640, 360))
    image4_resized = cv2.resize(img4, (640, 360))

    # 创建一个空白画布
    canvas = np.zeros((720, 1280, 3), dtype=np.uint8)

    # 将四张图片拼接到画布上
    canvas[:360, :640] = image1_resized
    canvas[:360, 640:] = image2_resized
    canvas[360:, :640] = image3_resized
    canvas[360:, 640:] = image4_resized
    cv2.imshow('img1',canvas)
   
    cv2.waitKey()

def main4(dataroot,tri_img1,tri_img2,test_img1,test_img2):
    points_Picked = []
    
    img1 = cv2.imread(dataroot + f'/{tri_img1}_225.png')
    with open(f'./{tri_img1}.json') as metajson:
        data1 = json.load(metajson)
    camera_intrinsic1 = np.array(data1['camera_intrinsic'])
    global_camera1 =  data1['global_camera']
    # dist1 = np.array(data1["dist"])
    # newcameramtx =np.array( [])
    # img1 = cv2.undistort(img1,camera_intrinsic1[:3,:3],dist1,None, newcameramtx)
    projection1 = camera_intrinsic1 @ np.array(global_camera1)
    points1 = np.array(cv2.selectROI("select", img1, False,False)[:2])
    points_Picked.append([points1,projection1])


    img2 = cv2.imread(dataroot + f'/{tri_img2}_225.png')
    with open(f'./{tri_img2}.json') as metajson:
        data2 = json.load(metajson)
    camera_intrinsic2 = data2['camera_intrinsic']
    global_camera2 =  data2['global_camera']
    projection2 = np.array(camera_intrinsic2) @ np.array(global_camera2)
    points2 = np.array(cv2.selectROI("select", img2, False,False)[:2])
    points_Picked.append([points2,projection2])
    

    
    
    
    img3 = cv2.imread(dataroot + f'/{test_img1}_225.png')
    with open(f'./{test_img1}.json') as metajson:
        data3 = json.load(metajson)
    camera_intrinsic3 = data3['camera_intrinsic']
    global_camera3 =  data3['global_camera']
    points3 = np.array(cv2.selectROI("select", img3, False,False)[:2])
    projection3 = np.array(camera_intrinsic3) @ np.array(global_camera3)
    # points_Picked.append([points3,projection3])
    
    
    img4 = cv2.imread(dataroot + f'/{test_img2}_225.png')
    with open(f'./{test_img2}.json') as metajson:
        data4 = json.load(metajson)
    camera_intrinsic4= data4['camera_intrinsic']
    global_camera4 =  data4['global_camera']
    projection4 = np.array(camera_intrinsic4) @ np.array(global_camera4)
    # p3d = [0,1000,0,1]
    p3d = triangulation(points_Picked)
    print(p3d)
    img1 = cv2.circle(img1, projection_points(p3d,projection1), 5,
                                 (0, 255, 0), -1, 16)
    img2 = cv2.circle(img2, projection_points(p3d,projection2), 5,
                                 (0, 255, 0), -1, 16)
    img3 = cv2.circle(img3, projection_points(p3d,projection3), 5,
                                 (0, 255, 0), -1, 16)
    img4 = cv2.circle(img4, projection_points(p3d,projection4), 5,
                                 (0, 255, 0), -1, 16)
    image1_resized = cv2.resize(img1, (640, 360))
    image2_resized = cv2.resize(img2, (640, 360))
    image3_resized = cv2.resize(img3, (640, 360))
    image4_resized = cv2.resize(img4, (640, 360))

    # 创建一个空白画布
    canvas = np.zeros((720, 1280, 3), dtype=np.uint8)

    # 将四张图片拼接到画布上
    canvas[:360, :640] = image1_resized
    canvas[:360, 640:] = image2_resized
    canvas[360:, :640] = image3_resized
    
    canvas[360:, 640:] = image4_resized

    cv2.imshow('img1',canvas)
   
    cv2.waitKey()

if __name__ == '__main__':
    
    main4('./sceneRealworld2/test',64,69,62,68)