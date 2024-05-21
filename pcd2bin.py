# ===============================================================================================
# transpose the pcd file of customer format into the KITTI format with visualization for bin file
# ===============================================================================================

import numpy as np
import os
import argparse
from pypcd import pypcd
import csv
from tqdm import tqdm
import open3d as o3d
import time

def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")
    parser.add_argument(
        "--pcd_file_path",
        help="Path for .pcd files for transpose",
        type=str,
        default="./pcd_files"
    )
    parser.add_argument(
        "--bin_file_path",
        help="Path for .bin files for saving",
        type=str,
        default="./bin_files"
    )
    args = parser.parse_args()

    ## Find all pcd files
    pcd_files = []
    file_names = []
    for (path, dir, files) in os.walk(args.pcd_file_path):
        for filename in files:
            # print(filename)
            ext = os.path.splitext(filename)[-1]
            if ext == '.pcd':
                pcd_files.append(path + "/" + filename)
                file_names.append(filename[:-4])

    ## Sort pcd files by the length and indexes of file name
    pcd_files.sort()
    file_names.sort()
    time.sleep(0.1)
    print("Finish to load point clouds!")

    ## Make bin_path directory if it not exists
    try:
        if not (os.path.isdir(args.bin_file_path)):
            os.makedirs(os.path.join(args.bin_file_path))
    except OSError as e:
        if e.errno != errno.EEXIST:
            time.sleep(0.1)
            print ("Failed to create directory!!!!!")
            raise

    # Converting process in here !!!
    time.sleep(0.1)
    print("Start converting the pcd files into bin files of KITTI format!")
    seq = 0
    for pcd_file in tqdm(pcd_files):
        # Get pcd data from the pre-specified file
        pcd = pypcd.PointCloud.from_path(pcd_file)

        # Initialize the bin file saving path and corresponding filename
        bin_file_name = "{}.bin".format(file_names[seq])
        bin_file_path = os.path.join(args.bin_file_path, bin_file_name)
        
        # Get the x, y and z demension data from the pcd file
        np_x = (np.array(pcd.pc_data['x'], dtype=np.float32)).astype(np.float32)
        np_y = (np.array(pcd.pc_data['y'], dtype=np.float32)).astype(np.float32)
        np_z = (np.array(pcd.pc_data['z'], dtype=np.float32)).astype(np.float32)
        # Load the intensity file if and only if it exists
        try:
            np_i = (np.array(pcd.pc_data['intensity'], dtype=np.float32)).astype(np.float32) / 256
        except:
            shape = (len(np_x))
            # Forged intensity because the normal pcd file may not contain the intensity value
            np_i = np.ones(shape)

        ## To noted that the ring and time demensions are even more rare in the pcd files
        ## The demensions are only preserved for the possible usage of other developers
        # np_r = (np.array(pc.pc_data['ring'], dtype=np.float32)).astype(np.float32)
        # np_t = (np.array(pc.pc_data['time'], dtype=np.float32)).astype(np.float32)

        ## Stack all data
        x_max = max(np_x)
        x_min = min(np_x)

        y_max = max(np_y)
        y_min = min(np_y)

        z_max = max(np_z)
        z_min = min(np_z)

        # print the maximum and minimum value of all three demensions
        # print('x最大值/最小值：', x_max, x_min)
        # print('y最大值/最小值：', y_max, y_min)
        # print('z最大值/最小值：', z_max, z_min)

        # 将自己的pcd数据转化为kitti格式，x_tar，y_tar和z_tar分别对应x,y,z三个坐标轴的上下限
        x_tar = [-40, 40]
        y_tar = [-3, 20]
        z_tar = [0, 70.4]

        np_x = (((np_x - x_min) * (x_tar[1] - x_tar[0]) / (x_max - x_min)) + x_tar[0]).astype(np.float32)
        np_y = (((np_y - y_min) * (y_tar[1] - y_tar[0]) / (y_max - y_min)) + y_tar[0]).astype(np.float32)
        np_z = (((np_z - z_min) * (z_tar[1] - z_tar[0]) / (z_max - z_min)) + z_tar[0]).astype(np.float32)

        points_32 = np.transpose(np.vstack((np_x, np_y, np_z, np_i))).reshape(-1)

        points_32 = points_32.astype(np.float32)

        ## Save bin file                                    
        points_32.tofile(bin_file_path)

        seq = seq + 1

    ## Coverting process finished
    time.sleep(0.1)
    print("Converting of {} pcd files into KITTI format bin files has been finished!".format(str(seq+1)))
    # Pick an example bin file to show the visualzation result
    bin_file_for_show = './bin_files/MedievalHouse2.bin'

    time.sleep(0.1)
    print("The bin file for demonstration is {}".format(bin_file_for_show))
    data = np.fromfile(bin_file_for_show, dtype=np.float32)
    data2 = data.reshape(-1,4)

    pcd = o3d.geometry.PointCloud()
    data3 = data2[:, :3]
    pcd.points = o3d.utility.Vector3dVector(data3)

    o3d.visualization.draw_geometries([pcd])
    
if __name__ == "__main__":
    main()