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

    
def get_matrix_ours(rotation,translation,inverse = False):
    output = np.eye(4)
    output[:3,:3] = rotation
    output[:3, 3] = translation
    if inverse:
        output = np.linalg.inv(output)
    return output

def write_json2(camera_intrinsic,global_camera,dist,filename):

    meta = {
    "camera_intrinsic": camera_intrinsic.tolist(),
    "global_camera": global_camera.tolist(),
    "dist": dist
    }
    json_str = json.dumps(meta)
    print(meta)
    with open(filename, 'w') as file:
        file.write(json_str) 
def main3():
    rotation62_69 =  [[ 0.93339471,-0.18683844  ,0.30637512],
                 [ 0.08686367 , 0.94601211,  0.31227518],
                 [-0.34817958 ,-0.26486313  ,0.89923217]]
    translation62_69 =  [-80.44747937,-34.12556036,38.63550577]
   
    rotation69_64 =  [[ 0.97109253,0.0154267,0.23820436],
                     [-0.00259568 ,0.99853293 ,-0.05408558],
                     [-0.23868926 , 0.0519038 ,  0.96970791]]
    translation69_64 =  [-74.34780841,7.33163197,25.95823596]
    rotation64_68 =  [[ 0.60356699 , 0.0509151 ,  0.79568495],
                        [-0.02242047 , 0.99864816 ,-0.04689547],
                        [-0.796997  ,  0.01046493,  0.6038926 ]]
    translation64_68 =  [-121.75902971, 8.81619197,63.44144186]
    matrix62_69 = get_matrix_ours(rotation62_69,translation62_69,True)
    matrix69_64 = get_matrix_ours(rotation69_64,translation69_64,True)
    matrix64_68 = get_matrix_ours(rotation64_68,translation64_68,True) 
    matrix68 = np.eye(4)
    matrix64 = np.array(matrix64_68) @ matrix68
    matrix69 = np.array(matrix69_64) @ matrix64
    matrix62 = np.array(matrix62_69) @ matrix69

    myprint(matrix68)
    myprint(matrix64)
    myprint(matrix69)
    myprint(matrix62)


    

    intrinsic62 =  [[1.98822337e+03,0.00000000e+00,5.72591884e+02],
                    [0.00000000e+00,1.98510004e+03,3.57660970e+02],
                    [0.00000000e+00,0.00000000e+00,1.00000000e+00]]
    intrinsic68 =  [[2.00815056e+03,0.00000000e+00,5.90742396e+02],
                     [0.00000000e+00,2.01016936e+03,3.07228280e+02],
                     [0.00000000e+00,0.00000000e+00,1.00000000e+00]]
    intrinsic64 =  [[1.95203055e+03,0.00000000e+00,6.23086152e+02],
                    [0.00000000e+00 ,1.95401760e+03 ,3.70373446e+02],
                    [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]]
    intrinsic69 =  [[1.95698911e+03 ,0.00000000e+00 ,6.53187474e+02],
                    [0.00000000e+00 ,1.95892244e+03 ,3.44528371e+02],
                    [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]]
    dist_62 =  [-4.72669403e-01 , 8.84532170e-03 ,-2.91307138e-03 , 7.16745935e-03, -1.47354456e+01]
    dist_68 =  [-4.96383754e-01 , 8.56413380e-02 , 1.77492689e-03 , 1.07565922e-02 , 7.26445530e+00]
    dist_69 =  [-0.73989785  ,3.92157188, 0.00851629 ,-0.00732661 , 1.68473561]
    dist_64 =  [-4.06804607e-01 ,-5.29043825e+00, -1.97943528e-03 , 5.48920283e-03, 6.43736292e+01]
    w,h = 720, 1280
    # intrinsic62, roi=cv2.getOptimalNewCameraMatrix(np.array(intrinsic62),np.array(dist_62),(w,h),1,(w,h))
    # intrinsic64, roi=cv2.getOptimalNewCameraMatrix(np.array(intrinsic64),np.array(dist_64),(w,h),1,(w,h))
    # intrinsic68, roi=cv2.getOptimalNewCameraMatrix(np.array(intrinsic68),np.array(dist_68),(w,h),1,(w,h))
    # intrinsic69, roi=cv2.getOptimalNewCameraMatrix(np.array(intrinsic69),np.array(dist_69),(w,h),1,(w,h))
    write_json2(np.hstack((intrinsic62, np.zeros((3, 1)))),matrix62,dist_62,'./62.json')
    write_json2(np.hstack((intrinsic64, np.zeros((3, 1)))),matrix64,dist_64,'./64.json')
    write_json2(np.hstack((intrinsic68, np.zeros((3, 1)))),matrix68,dist_68,'./68.json')
    write_json2(np.hstack((intrinsic69, np.zeros((3, 1)))),matrix69,dist_69,'./69.json')


if __name__ == '__main__':
    main3()