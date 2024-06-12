import cv2
from pyquaternion import Quaternion 
import numpy as np
from scipy.spatial.transform import Rotation
import json
import os
from math import sin,cos

def myprint(str1):
    print('--------------------------------------------------------------')
    print(str1)

def get_matrix(rotation,translation,inverse = False):
    """
    计算外参，拿到的是旋转角度的情况下用这个
    """
    output = np.eye(4)
    # output[:3,:3] = Rotation.from_euler('xyz', [np.deg2rad(rotaion[0]), np.deg2rad(rotaion[1]), np.deg2rad(rotaion[2])]).as_matrix()
    # output[:3,:3] = eulerToMatrix(np.deg2rad(rotaion[0]), np.deg2rad(rotaion[1]), np.deg2rad(rotaion[2]))
    output[:3,:3] = Rotation.from_euler('xyz', [rotation[0],rotation[1], rotation[2]]).as_matrix()
    # output[:3,:3] = cv2.Rodrigues(np.array(rotation).reshape(3,1))[0].tolist() cv2标定得到的rotationVector用这个获取旋转矩阵Matrix
    output[:3, 3] = translation
    if inverse:
        output = np.linalg.inv(output)
    # print("变换矩阵:\n", output)
    
    return output

def get_matrix_ours(rotationMatrix,translation,inverse = False):  # 
    """
    计算外参，拿到的是旋转矩阵的情况下用这个
    """
    output = np.eye(4)
    output[:3,:3] = rotationMatrix
    output[:3, 3] = translation
    if inverse:
        output = np.linalg.inv(output)
    return output

def write_json2(camera_intrinsic,global_camera,dist,filename):
    """
    写入json文件
    """
    meta = {
    "camera_intrinsic": camera_intrinsic.tolist(),
    "global_camera": global_camera.tolist(),
    "dist": dist
    }
    json_str = json.dumps(meta)
    print(meta)
    with open(filename, 'w') as file:
        file.write(json_str) 
def main():
    rotation62_69 =  [  [0.734606019081237, -0.0118014787353319, 0.678391274877027],
  [-0.0477296962353446, 0.996472856852483, 0.0690197193092891],
  [-0.676813026490256, -0.0830817107184898, 0.731451677499744]
]
    translation62_69 =  [4599.38719768906,	312.559048652946,	4300.01742408740]
   
    rotation64_69 =   [
  [0.935182558615242, -0.0337493697263050, -0.352554339222944],
  [0.0354316045592265, 0.999370687567481, -0.00168232852306637],
  [0.352389249921458, -0.0109182816403541, 0.935789830926696]
]
    translation64_69 =   [-2975.25359066697	,190.880543821017	,100.918512377616]
    
    rotation68_69 =   [
                        [0.830239130721091, -0.0660255245075057, -0.553483166801843],
                        [0.0804955341325689, 0.996753267811412, 0.00184176328112263],
                        [0.551564551801691, -0.0460820270906133, 0.832858326472768]
                        ]
    translation68_69 =   [-4351.92157217084	,392.970016655707	,1869.95148291682]
    
    # 根据旋转矩阵和平移矩阵计算外参
    matrix62_69 = get_matrix_ours(np.array(rotation62_69).T.tolist(),translation62_69,False)  # 如果是matlab标定得到的旋转矩阵，据需要转置
    matrix64_69 = get_matrix_ours(np.array(rotation64_69).T.tolist(),translation64_69,False)
    matrix68_69 = get_matrix_ours(np.array(rotation68_69).T.tolist(),translation68_69,False) 
    
    matrix69 = np.eye(4)  # 世界原点，所以他的外参是单位矩阵
    matrix64 = np.array(matrix64_69) @ matrix69 # 其实不用乘也可以，代表一个转换的过程
    matrix68 = np.array(matrix68_69) @ matrix69
    matrix62 = np.array(matrix62_69) @ matrix69 

   
    myprint(matrix68)
    myprint(matrix64)
    myprint(matrix69)
    myprint(matrix62)


    

    intrinsic62 =  [
                    [2780.74045164475, 0, 0],
                    [0, 2431.30419214899, 0],
                    [603.304849624114, 335.933668520886, 1]
                    ]
    intrinsic68 =  [
                    [1980.19015803597, 0, 0],
                    [0, 1991.46136460787, 0],
                    [624.350603120951, 344.194797871133, 1]
                    ]
    intrinsic64 =  [
                    [1999.80047809944, 0, 0],
                    [0, 2003.91594485890, 0],
                    [640.923265367901, 357.785650009929, 1]
                    ]
    intrinsic69 =  [
                    [2316.85909529136, 0, 0],
                    [0, 2329.20550843308, 0],
                    [634.756511556630, 362.475001115765, 1]
                    ]
    dist_62 =  [-4.72669403e-01 , 8.84532170e-03 ,-2.91307138e-03 , 7.16745935e-03, -1.47354456e+01]
    dist_68 =  [-4.96383754e-01 , 8.56413380e-02 , 1.77492689e-03 , 1.07565922e-02 , 7.26445530e+00]
    dist_69 =  [-0.73989785  ,3.92157188, 0.00851629 ,-0.00732661 , 1.68473561]
    dist_64 =  [-4.06804607e-01 ,-5.29043825e+00, -1.97943528e-03 , 5.48920283e-03, 6.43736292e+01]
    w,h = 720, 1280

    write_json2(np.hstack((np.array(intrinsic62).T, np.zeros((3, 1)))),matrix62,dist_62,'./62.json') # 如果是matlab标定得到的内参矩阵，据需要转置
    write_json2(np.hstack((np.array(intrinsic64).T, np.zeros((3, 1)))),matrix64,dist_64,'./64.json')
    write_json2(np.hstack((np.array(intrinsic68).T, np.zeros((3, 1)))),matrix68,dist_68,'./68.json')
    write_json2(np.hstack((np.array(intrinsic69).T, np.zeros((3, 1)))),matrix69,dist_69,'./69.json')


if __name__ == '__main__':
    main()