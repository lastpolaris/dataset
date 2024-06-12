#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time        :2022/7/27 14:59
# @Author      :weiz
# @ProjectName :robotVision3
# @File        :triangulation.py
# @Description :
import cv2
 
from triangulation_data import *
import math
 
 
def get_M(R, t):
    """
    获取旋转平移的矩阵
    :param R:
    :param t:
    :return:
    """
    M = [[R[0][0], R[0][1], R[0][2], t[0]],
         [R[1][0], R[1][1], R[1][2], t[1]],
         [R[2][0], R[2][1], R[2][2], t[2]]]
    return np.array(M)
 
 
 
def get_M_homo(R, t):
    """
    获取旋转平移的齐次矩阵
    :param R:
    :param t:
    :return:
    """
    M = [[R[0][0], R[0][1], R[0][2], t[0]],
         [R[1][0], R[1][1], R[1][2], t[1]],
         [R[2][0], R[2][1], R[2][2], t[2]],
         [0,       0,       0,       1]]
    return np.array(M)
 
 
def get_world2point2_R_t(R_camera2gripper, t_camera2gripper, point1_gripper, point2_gripper):
    """
    获取关键点1坐标系到关键点2坐标系的 R,t
    :param R_camera2gripper:手眼标定的R
    :param t_camera2gripper:手眼标定的t
    :param point1_gripper:视点1时刻机械臂末端的位姿[x,y,z,rx,ry,rz]
    :param point2_gripper:视点2时刻机械臂末端的位姿[x,y,z,rx,ry,rz]
    :return:
    """
    R_gripper2base_point1 = np.zeros((3, 3))
    t_gripper2base_point1 = np.array([point1_gripper[0], point1_gripper[1], point1_gripper[2]])
    cv2.Rodrigues(np.array([point1_gripper[3], point1_gripper[4], point1_gripper[5]]), R_gripper2base_point1)
 
    R_gripper2base_point2 = np.zeros((3, 3))
    t_gripper2base_point2 = np.array([point2_gripper[0], point2_gripper[1], point2_gripper[2]])
    cv2.Rodrigues(np.array([point2_gripper[3], point2_gripper[4], point2_gripper[5]]), R_gripper2base_point2)
 
    R_gripper2camera = np.linalg.inv(np.array(R_camera2gripper))
    t_gripper2camera = -np.dot(R_gripper2camera, t_camera2gripper)
 
    R_base2gripper_point2 = np.linalg.inv(np.array(R_gripper2base_point2))
    t_base2gripper_point2 = -np.dot(R_base2gripper_point2, t_gripper2base_point2)
 
    M = get_M_homo(R_camera2gripper, t_camera2gripper)
    M = np.matmul(get_M_homo(R_gripper2base_point1, t_gripper2base_point1), M)
    M = np.matmul(get_M_homo(R_base2gripper_point2, t_base2gripper_point2), M)
    M = np.matmul(get_M_homo(R_gripper2camera, t_gripper2camera), M)
 
    R_world2point2 = M[0:3, 0:3]
    t_world2point2 = np.array([M[0][3], M[1][3], M[2][3]])
    # print(M)
    # print(R_world2point2)
    # print(t_world2point2)
 
    return R_world2point2, t_world2point2, M
 
 
def triangulation(point1, point2, ipm, R_world2point2, t_world2point2):
    """
    三角测量：利用机械臂位姿信息直接进行三维坐标估计
    :param point1:该坐标系为世界坐标系
    :param point2:
    :param ipm:相机内参
    :param R_world2point2:
    :param t_world2point2:
    :return:
    """
    ipm = np.array(ipm)
    M1 = np.matmul(ipm, get_M(np.eye(3, 3), [0, 0, 0]))
    M2 = np.matmul(ipm, get_M(R_world2point2, t_world2point2))
    A = []
    A.append(point1[0] * M1[2] - M1[0])
    A.append(point1[1] * M1[2] - M1[1])
    A.append(point2[0] * M2[2] - M2[0])
    A.append(point2[1] * M2[2] - M2[1])
    # print(np.array(A))
    U, sigma, V = np.linalg.svd(A)
    p_1 = [V[3][0], V[3][1], V[3][2], V[3][3]]
    p_2 = [V[3][0] / V[3][3], V[3][1] / V[3][3], V[3][2] / V[3][3]]
    # print(p_1)
    # print(p_2)
 
    total_error = 0
    point1_pro, _ = cv2.projectPoints(np.array([p_2]), np.eye(3), np.array([[0.], [0.], [0.]]), ipm, None)
    point2_pro, _ = cv2.projectPoints(np.array([p_2]), R_world2point2, t_world2point2, ipm, None)
    print(point2_pro)
    total_error = total_error + cv2.norm(np.array([[point1]], dtype=np.double), point1_pro, cv2.NORM_L2) / 1.
    total_error = total_error + cv2.norm(np.array([[point1]], dtype=np.float64), point2_pro, cv2.NORM_L2) / 1.
    print(total_error)
 
    return p_2


def my_triangulation(point1, point2, ipm, world2point1, world2point2):
    """
    三角测量：利用机械臂位姿信息直接进行三维坐标估计
    :param point1:该坐标系为世界坐标系
    :param point2:
    :param ipm:相机内参
    :param R_world2point2:
    :param t_world2point2:
    :return:
    """
    ipm = np.array(ipm)
    world2point1 = np.array(world2point1)
    world2point2 = np.array(world2point2)
    M1 = np.matmul(ipm, world2point1)
    M2 = np.matmul(ipm, world2point2)
    A = []
    A.append(point1[0] * M1[2] - M1[0])
    A.append(point1[1] * M1[2] - M1[1])
    A.append(point2[0] * M2[2] - M2[0])
    A.append(point2[1] * M2[2] - M2[1])
    # print(np.array(A))
    U, sigma, V = np.linalg.svd(A)
    p_1 = [V[3][0], V[3][1], V[3][2], V[3][3]]
    p_2 = [V[3][0] / V[3][3], V[3][1] / V[3][3], V[3][2] / V[3][3]]
    # print(p_1)
    # print(p_2)

    total_error = 0
    #point1_pro, _ = cv2.projectPoints(np.array([p_2]), world2point1[0:3,0:3], world2point1[0:3,3], ipm, None)
    #point2_pro, _ = cv2.projectPoints(np.array([p_2]), world2point2[0:3,0:3], world2point2[0:3,3], ipm, None)
    # print(point2_pro)
    #total_error = total_error + cv2.norm(np.array([[point1]], dtype=np.double), point1_pro, cv2.NORM_L2) / 1.
    #total_error = total_error + cv2.norm(np.array([[point1]], dtype=np.float64), point2_pro, cv2.NORM_L2) / 1.
    # print(total_error)

    return p_2
def my_triangulation_multiview(points, ipm, world2points):
    """
    三角测量：利用机械臂位姿信息直接进行三维坐标估计
    :param point1:该坐标系为世界坐标系
    :param point2:
    :param ipm:相机内参
    :param R_world2point2:
    :param t_world2point2:
    :return:
    """
    ipm = np.array(ipm)
    world2points = np.array(world2points)


    A = []
    for i in range(len(points)):
        M1 = np.matmul(ipm, world2points[i])
        A.append(points[i][0] * M1[2] - M1[0])
        A.append(points[i][1] * M1[2] - M1[1])

    # print(np.array(A))
    U, sigma, V = np.linalg.svd(A)
    p_1 = [V[3][0], V[3][1], V[3][2], V[3][3]]
    p_2 = [V[3][0] / V[3][3], V[3][1] / V[3][3], V[3][2] / V[3][3]]

    return p_2
def triangulation_opencv(point1, point2, ipm, R_world2point2, t_world2point2):
    point1 = np.array([point1], dtype=np.float32)
    point2 = np.array([point2], dtype=np.float32)
 
    M1 = np.zeros((3, 4))
    M2 = np.zeros((3, 4))
    M1[0:3, 0:3] = np.eye(3)
    M2[0:3, 0:3] = np.float32(R_world2point2)
    M2[:, 3] = np.float32(t_world2point2)
    M1 = np.matmul(ipm, M1)
    M2 = np.matmul(ipm, M2)
    p_homo = cv2.triangulatePoints(M1, M2, point1.T, point2.T)
    p = []
    # 齐次坐标转三维坐标：前三个维度除以第四个维度
    for i in range(len(p_homo[0])):
        col = p_homo[:, i]
        col /= col[3]
        p.append([col[0], col[1], col[2]])
    return p[0]

def my_triangulation_opencv(point1, point2, ipm, world2point1,world2point2):
    point1 = np.array([point1], dtype=np.float32)
    point2 = np.array([point2], dtype=np.float32)
 
    M1 = np.zeros((3, 4))
    M2 = np.zeros((3, 4))
    
    M1 = np.float32(world2point1)

    M2 = np.float32(world2point2)
    
    M1 = np.matmul(ipm, M1)
    M2 = np.matmul(ipm, M2)
    p_homo = cv2.triangulatePoints(M1, M2, point1.T, point2.T)
    p = []
    # 齐次坐标转三维坐标：前三个维度除以第四个维度
    for i in range(len(p_homo[0])):
        col = p_homo[:, i]
        col /= col[3]
        p.append([col[0], col[1], col[2]])
    return p[0]
 
 
 
 
 
R_camera2gripper =[[-0.999266726198567, -0.016016251269208765, -0.0347777171142492],
                   [-0.03487664683144269, 0.005939423009193905, 0.99937397542667],
                   [-0.015799665129108013, 0.9998540908300567, -0.00649366092498505]]  # 旋转
t_camera2gripper = [0.057164748537088694, 0.10581519613132581, 0.1255394553568957]  # 位移
ipm = np.array([[535.05, 0, 653.005], [0, 535.01, 386.191], [0, 0, 1]])   # 内参

def my_print(name,value):
    print(f'-------------------------test{name}-------------------------------------------')
    print(value)
def test():
    global_camera1 = np.array([[-0.8829475928589265, 0.4694715627858908, -2.3435891625042823e-17, -15165.930028331266],
                               [-0.13726020113055826, -0.2581488929050013, -0.9563047559630351, -902.9260299452461],
                               [0.44895788828154604, 0.8443669823171053, -0.2923717047227367, -2823.5548445544905],
                               [0, 0, 0, 1]])
    global_camera2 = np.array([[-0.40673664307580026, -0.9135454576426002, -4.300618117770482e-17, 4723.9497425381705],
                      [0.4841053367575533, -0.21553758263541178, -0.8480480961564255, 5816.552509023407],
                      [-0.7747304861061582, 0.34493223579748816, -0.5299192642332047, -12184.339610325496],
                     [0, 0, 0, 1]])
    world_point = np.array([-12078, 9998, -1980, 1])

    my_ipm = np.array([[808.0808080808081, 0, 640.0, 0], [0, 808.0808080808081, 360.0,0], [0, 0, 1, 0]])

    point = my_ipm @ global_camera1 @ world_point
    point2 = my_ipm @ global_camera2 @ world_point
    point = [point[0]/point[2],point[1]/point[2]]
    point2 = [point2[0]/point2[2],point2[1]/point2[2]]
    print(point)
    print(point2)
    p = my_triangulation(point, point2, my_ipm[:,0:3], global_camera1[0:3], global_camera2[0:3])
    my_print('oringin',p)
    point = [point[0]+10, point[1] +10]
    point2 = [point2[0] +10, point2[1]+10]
    p_noisy = my_triangulation(point, point2, my_ipm[:, 0:3], global_camera1[0:3], global_camera2[0:3])
    my_print('noisy',p_noisy)



if __name__ == "__main__":
    my_ipm = [[808.0808080808081, 0, 640.0], [0, 808.0808080808081, 360.0], [0, 0, 1]]
    global_camera1 = [[-0.8829475928589265, 0.4694715627858908, -2.3435891625042823e-17, -15165.930028331266], 
                     [-0.13726020113055826, -0.2581488929050013, -0.9563047559630351, -902.9260299452461], 
                     [0.44895788828154604, 0.8443669823171053, -0.2923717047227367, -2823.5548445544905]]
    global_camera2 = [[-0.40673664307580026, -0.9135454576426002, -4.300618117770482e-17, 4723.9497425381705],
                      [0.4841053367575533, -0.21553758263541178, -0.8480480961564255, 5816.552509023407],
                      [-0.7747304861061582, 0.34493223579748816, -0.5299192642332047, -12184.339610325496]]
    global_camera3 = [[0.951056516295153, -0.30901699437494723, 0.0, 13886.03976902981],
                        [0.1256885349454394, 0.3868295348132556, -0.9135454576426002, -4384.704263115094],
                        [-0.28230107154560213, -0.8688333604228334, -0.40673664307579993, 6061.973280866766]]
    global_camera4 = [[0.8910065241883675, 0.45399049973954686, 0.0, 5551.2765904379785],
                     [-0.2200989617741746, 0.43196853462877133, -0.8746197071393956, -8846.798307108278],
                     [0.3970690379262704, -0.7792918652449209, -0.48480962024633695, 12833.045972402702]]
    global_camera_all = [global_camera1,global_camera2,global_camera3,global_camera4]
    camera_point1 = [775, 346, 143, 184]
    camera_point2 = [867, 82, 40, 85]
    camera_point3 = [265, 204, 52, 89]
    camera_point4 = [137, 216, 94, 108]
    camera_point_all = [camera_point1,camera_point2,camera_point3,camera_point4]
    my_point1 = [camera_point1[0] + camera_point1[2]//2,camera_point1[1] + camera_point1[3]//2]  # 840,430
    my_point2 = [camera_point2[0] + camera_point2[2]//2,camera_point2[1] + camera_point2[3]//2]  # 887,124
    my_point3 = [camera_point3[0] + camera_point3[2]//2,camera_point3[1] + camera_point3[3]//2]  # 291,248
    my_point4 = [camera_point4[0] + camera_point4[2]//2,camera_point4[1] + camera_point4[3]//2]  # 184.270
    my_point_all = [my_point1, my_point2, my_point3, my_point4]


    world_point = [-12078,9998,-1980]
    R_world2point2, t_world2point2, M = get_world2point2_R_t(R_camera2gripper, t_camera2gripper, point1_gripper, point2_gripper)  
    p = triangulation(point1, point2, ipm, R_world2point2, t_world2point2)
    print(p)
    
    p = triangulation_opencv(point1, point2, ipm, R_world2point2, t_world2point2)
    print(p)



    p2 = my_triangulation(my_point1, my_point2, ipm, global_camera1, global_camera2)
    my_print('1-2', p2)

    p2 = my_triangulation(my_point2, my_point4, ipm, global_camera2, global_camera4)
    my_print('2-4', p2)

    p2 = my_triangulation(my_point1, my_point4, ipm, global_camera1, global_camera4)
    my_print('1-4', p2)
    p2 = my_triangulation(my_point3, my_point4, ipm, global_camera3, global_camera4)
    my_print('3-4', p2)
    fault_test = False
    if fault_test:
        p2 = my_triangulation(my_point1, my_point4, ipm, global_camera3, global_camera4)
        my_print('fault', p2)
        p2 = my_triangulation(my_point2, my_point4, ipm, global_camera1, global_camera4)
        my_print('fault2', p2)
        p2 = my_triangulation(my_point1, my_point3, ipm, global_camera1, global_camera4)
        my_print('fault3', p2)
        p2 = my_triangulation(my_point2, my_point4, ipm, global_camera3, global_camera4)
        my_print('fault4', p2)
        p2 = my_triangulation(my_point1, my_point2, ipm, global_camera1, global_camera4)
        my_print('fault5', p2)
    print('-------------------------multiview------------------------------')
    p2 = my_triangulation_multiview(my_point_all, ipm, global_camera_all)
    print(p2)

    print('test-----------------------------------------------------------')
    test()



