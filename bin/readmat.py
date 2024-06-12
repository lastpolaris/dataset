import scipy.io as scio
import os
import os.path as osp

BASEDIR = osp.dirname(osp.abspath(__file__))
def myprint(str1):
    print('------------------------------------------')
    print(str1)
path_mat = osp.join(BASEDIR, 'cam_param_parking.mat')  # 这里演示读取人体姿态估计数据集lsp_dataset里的mat格式标注文件
data = scio.loadmat(path_mat)
myprint(data)
myprint(type(data['cam_param']))  # <class 'numpy.ndarray'>
myprint(data['cam_param'][0])  # (3, 14, 2000)

new_data = data['cam_param'][...]  

myprint(new_data[0][0][1]) 
