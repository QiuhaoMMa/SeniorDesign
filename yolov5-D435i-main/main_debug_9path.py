# tianjia stepmotor de jiao liu
import pyrealsense2 as rs
import numpy as np
import cv2
import random
import torch
import time
import shutil
import serial

import platform
import pathlib
plt = platform.system()
if plt != 'Windows':
  pathlib.WindowsPath = pathlib.PosixPath


# Define function to load YOLOv5 model
def load_model(model_path):
    return torch.hub.load('ultralytics/yolov5', 'custom', model_path)

# Initialize model variable
model = None
model.conf = 0.5

def save_matrix_to_txt(matrix, filename):
    with open(filename, "a") as file:  # 使用追加模式打开文件
        matrix_as_string = ', '.join(map(str, matrix))
        file.write(matrix_as_string + "\n")


# Function to copy file to another directory
def copy_file(source_file, destination_directory):
    shutil.copy(source_file, destination_directory)

def get_mid_pos(frame,box,depth_data,randnum):
    distance_list = []
    mid_pos = [(box[0] + box[2])//2, (box[1] + box[3])//2] #确定索引深度的中心像素位置
    min_val = min(abs(box[2] - box[0]), abs(box[3] - box[1])) #确定深度搜索范围
    #print(box,)
    for i in range(randnum):
        bias = random.randint(-min_val//4, min_val//4)
        dist = depth_data[int(mid_pos[1] + bias), int(mid_pos[0] + bias)]
        cv2.circle(frame, (int(mid_pos[0] + bias), int(mid_pos[1] + bias)), 4, (255,0,0), -1)
        #print(int(mid_pos[1] + bias), int(mid_pos[0] + bias))
        if dist:
            distance_list.append(dist)
    distance_list = np.array(distance_list)
    distance_list = np.sort(distance_list)[randnum//2-randnum//4:randnum//2+randnum//4] #冒泡排序+中值滤波
    #print(distance_list, np.mean(distance_list))
    return np.mean(distance_list)
def dectshow(org_img, boxs,depth_data, depth_intrin):
    img = org_img.copy()

    for box in boxs:
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
        dist = get_mid_pos(org_img, box, depth_data, 24)
        print(dist)

        cv2.putText(img, box[-1] + str(dist / 1000)[:6] + 'm',(int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        a = int(box[0])
        b = int(box[1])

        x1 = int(box[0])
        y1 = int(box[1])
        x2 = int(box[2])
        y2 = int(box[3])

        cv2.circle(img, (x1, y1), 5, (0, 0, 255), -1)  # 红色圆点
        cv2.circle(img, (x2, y1), 5, (0, 255, 255), -1)  # 黄色圆点
        cv2.circle(img, (x2, y2), 5, (255, 0, 0), -1)  # 蓝色圆点

        depth1 = aligned_depth_frame.get_distance(x1, y1)  # 获取第一个点的深度值
        depth2 = aligned_depth_frame.get_distance(x2, y1)  # 获取第二个点的深度值
        depth3 = aligned_depth_frame.get_distance(x2, y2)  # 获取第三个点的深度值

        point1 = rs.rs2_deproject_pixel_to_point(depth_intrin, [x1, y1], depth1)
        point2 = rs.rs2_deproject_pixel_to_point(depth_intrin, [x2, y1], depth2)
        point3 = rs.rs2_deproject_pixel_to_point(depth_intrin, [x2, y2], depth3)

        # 计算两点之间的距离
        width = np.linalg.norm(np.array(point2) - np.array(point1))
        height = np.linalg.norm(np.array(point3) - np.array(point2))
        Area = width * height * 10000
        # print("宽 is:", width, "m")
        # print("长 is:", height, "m")

        cv2.putText(img, f'Area: {str(Area)[:8]}cm^2', (int(box[0]), int(box[1]) - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 0), 2)
        cv2.putText(img, f'W: {str(width)[:4]}m H: {str(height)[:4]}m', (int(box[0]), int(box[1]) - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)
        


    for i in range(len(boxs) ):
        X = int((boxs[i][2] + boxs[i][0]) / 2)
        Y = int((boxs[i][3] + boxs[i][1]) / 2)
        dis = aligned_depth_frame.get_distance(X, Y)
        camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (X, Y), dis)
        camera_xyz = np.round(np.array(camera_xyz), 3)
        cv2.circle(img, (X, Y), 4, (255, 255, 255), 5)
        
        # cv2.putText(img, str(camera_xyz), (X,Y), 0, 1,[0, 0, 255], thickness=2, lineType=cv2.LINE_AA)
        # print('%f,%f,%f',camera_xyz[0],camera_xyz[1],camera_xyz[2])



        # theta = np.pi/2
        theta = np.pi / 2 if n >= stepmotortime/2 else -np.pi / 2
        
        h = 0.04
        d = 0.18
        matrix1 = np.array([[1, 0, 0, d], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        matrix2 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, h], [0, 0, 0, 1]])
        matrix3 = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2), 0, 0], [np.sin(-np.pi/2), np.cos(-np.pi/2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        matrix4 = np.array([[1, 0, 0, 0], [0, np.cos(-np.pi/2), -np.sin(-np.pi/2), 0], [0, np.sin(-np.pi/2), np.cos(-np.pi/2), 0], [0, 0, 0, 1]])

        matrix5 = np.array([[np.cos(theta), 0, np.sin(theta), 0], [0, 1, 0, 0], [-np.sin(theta), 0, np.cos(theta), 0], [0, 0, 0, 1]])



        # Perform matrix multiplication using np.dot()
        result_mat = matrix1 @ matrix2 @ matrix3 @ matrix4 @ matrix5


        cam = np.array([[camera_xyz[0]], [camera_xyz[1]], [camera_xyz[2]],[1]])

        cam_final = np.dot(result_mat,cam)


        result = [cam_final[0,0], cam_final[1,0], cam_final[2,0]]

        # coordinates_detected = False



        result = [cam_final[0, 0], cam_final[1, 0], cam_final[2, 0], Area]



        if result[1] == 0:
            continue  # Skip this coordinate if it has long decimal values or y-value is 0

        # Round the coordinates to 3 decimal places
        result = tuple(round(coord, 3) for coord in result)

        # coordinates_detected = True



        save_matrix_to_txt(result, "output.txt")




        # Specify the source file (output.txt) and destination directory
        source_file = "output.txt"
        destination_directory = "/home/qiuhao/Design/output.txt"

        # Call the function to copy the file
        copy_file(source_file, destination_directory)

        # with open("output.txt","w") as file:
        #      file.write(result)
        print(camera_xyz)
    #cv2.circle(img, (640, 360), 8, [0, 0, 255], thickness=-1)
    cv2.imshow('dec_img', img)




if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    align_to = rs.stream.color  # 与color流对齐
    align = rs.align(align_to)
    n = 0
    stepmotortime = 32

    

    # ser = serial.Serial('/dev/ttyACM0', 9600)  # Replace '/dev/ttyUSBX' with the correct serial port
    # ser.write(b'M')

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            #新添内容
            aligned_frames = align.process(frames)  # 获取对齐帧
            aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）

            if not depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays

            depth_image = np.asanyarray(depth_frame.get_data())

            color_image = np.asanyarray(color_frame.get_data())

            results = model(color_image)
            boxs= results.pandas().xyxy[0].values

            #boxs = np.load('temp.npy',allow_pickle=True)
            dectshow(color_image, boxs, depth_image, depth_intrin)
            camera_xyz = dectshow(color_image, boxs, depth_image, depth_intrin)
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))
            # Show images
            #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            #cv2.imshow('RealSense', images)
            key = cv2.waitKey(1)
            start_time = time.time()
            # Press esc or 'q' to close the image window

            time.sleep(1)
            n = n + 1

            if key & 0xFF == ord('q') or key == 27 or n == stepmotortime:
                cv2.destroyAllWindows()
                break
            


    finally:
        # Stop streaming
        pipeline.stop()
        # ser.close()
