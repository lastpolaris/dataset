import os
import cv2
vedio_path = "./sceneRealWorld3/68/img"
rezult_path = "./sceneRealWorld3/68/groundtruth_rect.txt"
img_prefix = ['png','jpg','jpeg']


def read_images(path,img_prefix):
    files = os.listdir(path)
    img_files = []
    for file in files:

        index = file.find('.')
        prefix = file[index+1:]
        if prefix in img_prefix:
            img_files.append(file)
    return img_files

img_files = read_images(vedio_path,img_prefix)
with open(rezult_path,'r',) as f:
    ground_truth = f.readlines()
    print(ground_truth)
    for i in range(len(ground_truth)):
        ground_truth[i] = ground_truth[i].replace('\n','').split(', ')
        ground_truth[i] = [int(ground_truth[i][0]),int(ground_truth[i][1]),int(ground_truth[i][2]),int(ground_truth[i][3])]


        img = cv2.imread(vedio_path + '/' + img_files[i])
        print("-------------------------------------------")
        print(img)
        img = cv2.rectangle(img,ground_truth[i][:2],(ground_truth[i][0] + ground_truth[i][2],ground_truth[i][1] + ground_truth[i][3]),(255,0,0),2)

        cv2.putText(img, img_files[i], (0, 30), cv2.FONT_HERSHEY_SIMPLEX , 1,  (0, 0, 255) , 2)
        cv2.imshow('before',img)
        cv2.waitKey(10)

    for i in range(len(ground_truth),len(img_files)):
        img = cv2.imread(vedio_path +'\\'+ img_files[i])
        print("-------------------------------------------")
        print(img)
        cv2.putText(img, img_files[i], (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        roi = cv2.selectROI("select", img, False,False)  # 进行区域的选择
        # if roi == (0,0,0,0):
        #     break
        ground_truth.append(list(roi))
with open(rezult_path,'w',) as f:
    for item in ground_truth:
        f.writelines(str(item).replace('[','').replace(']','')+'\n')