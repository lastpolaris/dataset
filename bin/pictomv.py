from moviepy.editor import ImageSequenceClip
from moviepy.editor import ImageSequenceClip

image_path = "X:/tracking/SiamTrackers/SiamCAR/SiamCAR/bin/output/sceneTank/camera4/"
import cv2
import os
import os.path as osp


def img2video(img_dir, img_size, video_dir, fps):
    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')  # opencv3.0
    videoWriter = cv2.VideoWriter(video_dir, fourcc, fps, img_size)

    for idx in sorted(os.listdir(img_dir)):
        img = osp.join(img_dir, idx)
        frame = cv2.imread(img)
        # print(frame.shape)  # h, w, c (480, 640, 3)
        videoWriter.write(frame)

    videoWriter.release()
    print('Finish changing!')


if __name__ == '__main__':
    img_dir = image_path
    par_dir = osp.dirname(img_dir)
    video_path = osp.join(par_dir, 'X:/output.avi')

    fps = 10
    img_size = (640, 480)  # w, h
    img2video(img_dir=img_dir, img_size=img_size, video_dir=video_path, fps=fps)


