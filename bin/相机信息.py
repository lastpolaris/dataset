import cv2
from pyquaternion import Quaternion 
import numpy as np
from scipy.spatial.transform import Rotation
import json
import os
from math import sin,cos
def eulerToMatrix(heading, attitude, bank):
    # Convert euler angles back to matrix
    sa, ca = sin(attitude), cos(attitude)
    sb, cb = sin(bank), cos(bank)
    sh, ch = sin(heading), cos(heading)

    return [
        [
            ch*ca,
            (-ch*sa*cb) + (sh*sb),
            (ch*sa*sb) + (sh*cb)
        ],
        [
            sa,
            ca*cb,
            -ca*sb
        ],
        [
            -sh*ca,
            (sh*sa*cb) + (ch*sb),
            (-sh*sa*sb) + (ch*cb)
        ]
    ]

def rotationToHomogeneous(r): #rotation matrix to homogeneous transformation
    q = np.transpose(np.array([r[0], r[1], r[2], np.zeros(3)], np.float64))
    return np.transpose(np.array([ q[0], q[1], q[2], [ 0.0, 0.0, 0.0, 1.0 ] ], np.float64))


def translationToHomogeneous(t): #translation vector to homogeneous transformation
    tt = np.transpose(np.array([ np.zeros(4), np.zeros(4), np.zeros(4), [ t[0], t[1], t[2], 0.0 ] ], np.float64))
    return tt + np.identity(4)

def worldToCamera(translation,rotation,inverse = False):
    # rotationMatrix = eulerToMatrix(np.deg2rad(rotation[0]), np.deg2rad(rotation[1]), np.deg2rad(rotation[2]))
    rotationMatrix = eulerToMatrix(rotation[0], rotation[1], rotation[2])
    worldToCamera = np.dot(translationToHomogeneous(translation), rotationToHomogeneous(rotationMatrix))
    
    output=worldToCamera
    if inverse:
        output = np.linalg.inv(output)
    return output

def get_matrix(rotaion,translation,inverse = False):
    output = np.eye(4)
    output[:3,:3] = Rotation.from_euler('xyz', [np.deg2rad(rotaion[0]), np.deg2rad(rotaion[1]), np.deg2rad(rotaion[2])]).as_matrix()
    # output[:3,:3] = eulerToMatrix(np.deg2rad(rotaion[0]), np.deg2rad(rotaion[1]), np.deg2rad(rotaion[2]))
    #output[:3,:3] = Rotation.from_euler('xyz', [rotaion[0],rotaion[1], rotaion[2]]).as_matrix()
    
    output[:3, 3] = translation
    if inverse:
        output = np.linalg.inv(output)
    # print("变换矩阵:\n", output)
    
    return output


def get_camera_intrinsic_ue(f,W_pixel,H_pixel,W_physics,H_physics):
    
    camera_intrinsic = [[f*W_pixel/W_physics,0,W_pixel/2,0],
                        [0,f*H_pixel/H_physics,H_pixel/2,0],
                        [0,0,1,0]]
    return camera_intrinsic
def get_camera_intrinsic(f,W_pixel,H_pixel,dx,dy,cx,cy):
    
    camera_intrinsic = [[f/dx,0,cx,0],
                        [0,f/dy,cy,0],
                        [0,0,1,0]]
    return camera_intrinsic


def get_meta_ue(camera_param,ego_global_rotation,ego_global_translation):
    camera_intrinsic = get_camera_intrinsic_ue(*camera_param)  # f,W_pixel,H_pixel,W_physics,H_physics

    
    camera_ego_rotation = [-90,0,90]  # 
    camera_ego_translation = [0,0,0]
    ego_camera = get_matrix(camera_ego_rotation,camera_ego_translation,True)
    ego_global_rotation[0] *= -1
    ego_global_rotation[1] *= -1
    global_ego = get_matrix(ego_global_rotation,ego_global_translation,True)
    global_camera = np.diag([1,1,-1,1]) @ ego_camera @ global_ego  # 左右手坐标系转换  
    return camera_intrinsic, global_camera

def get_meta(camera_param,ego_global_rotation,ego_global_translation):
    # ego_global_rotation[0] *= -1
    # ego_global_rotation[1] *= -1
    # ego_camera = worldToCamera(ego_global_rotation,ego_global_translation,False)
    ego_camera = get_matrix(ego_global_rotation,ego_global_translation,False)
    camera_intrinsic = get_camera_intrinsic(*camera_param) 
    return camera_intrinsic,ego_camera

def main():
    # -----------------------------------------------虚幻引擎使用的-------------------------------------------------- # 
    # camera_param = [15,1280,720,23.76,13.365]
    # ego_global_rotation = [0,-29.0,-63.0]
    # ego_global_translation = [-11989.0,11302.0,-1516.0]  
    camera_param = [15,1280,720,23.76,13.365] # 焦距、图片宽、高、胶片宽、高（后两项一般不用动）
    ego_global_rotation = [0,-20,-7]  # 旋转向量
    ego_global_translation = [-16752,6810,-1454] # 位移向量
    camera_intrinsic, global_camera = get_meta_ue(camera_param,ego_global_rotation,ego_global_translation)
    meta = {
    "camera_intrinsic": camera_intrinsic,
    "global_camera": global_camera.tolist()
    }
    json_str = json.dumps(meta)
    print(meta)
    with open('./camera1.json', 'w') as file:  
        file.write(json_str)    


def write_json(camera_param,ego_global_rotation,ego_global_translation,filename):
    camera_intrinsic, global_camera = get_meta(camera_param,ego_global_rotation,ego_global_translation)
    meta = {
    "camera_intrinsic": camera_intrinsic,
    "global_camera": global_camera.tolist()
    }
    json_str = json.dumps(meta)
    print(meta)
    with open(filename, 'w') as file:
        file.write(json_str)    

    
# def main2():
#     # 0
#     camera_param = [20.161920,360,288,2.3000000000e-02,2.3000000000e-02,366.514507/2,305.832552/2]
#     ego_global_rotation = [1.9007833770e+00,4.9730769727e-01,1.8415452559e-01]
#     ego_global_translation = [-4.8441913843e+03,5.5109448682e+02,4.9667438357e+03]
#     write_json(camera_param,ego_global_rotation,ego_global_translation,'./camera0.json')
#     # 1
#     camera_param = [19.529144,360,288,2.3000000000e-02,2.3000000000e-02,360.228130/2,255.166919/2]
#     ego_global_rotation = [1.9347282363e+00,-7.0418616982e-01,-2.3783238362e-01]
#     ego_global_translation = [-65.433635,1594.811988,2113.640844]
#     write_json(camera_param,ego_global_rotation,ego_global_translation,'./camera1.json')
#     # 2
#     camera_param = [19.903218,360,288,2.3000000000e-02,2.3000000000e-02,355.506436/2,241.205640/2]
#     ego_global_rotation = [-1.8289537286e+00,3.7748154985e-01,3.0218614321e+00]
#     ego_global_translation = [1.9782813424e+03,-9.4027627332e+02,1.2397750058e+04]
#     write_json(camera_param,ego_global_rotation,ego_global_translation,'./camera2.json')
#     # 3
#     camera_param = [20.047015,360,288,2.3000000000e-02,2.3000000000e-02,349.154019/2,245.786168/2]
#     ego_global_rotation = [-1.8418460467e+00,-4.6728290805e-01,-3.0205552749e+00]
#     ego_global_translation = [4.6737509054e+03,-2.5743341287e+01,8.4155952460e+03]
#     write_json(camera_param,ego_global_rotation,ego_global_translation,'./camera3.json')
    
    

# def main3():
#     Rotation =  [[ 0.93339471,-0.18683844  ,0.30637512],
#                  [ 0.08686367 , 0.94601211,  0.31227518],
#                  [-0.34817958 ,-0.26486313  ,0.89923217]]
#     translation =  [-80.44747937,-34.12556036,38.63550577]
#     intrinsic =  [[1.98822337e+03,0.00000000e+00,5.72591884e+02],
#                  [0.00000000e+00,1.98510004e+03,3.57660970e+02],
#                  [0.00000000e+00,0.00000000e+00,1.00000000e+00]]


if __name__ == '__main__':
    main()