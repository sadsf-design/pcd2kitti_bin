#
# Module:       pcd2bin.py
# Description:  .pcd to .bin converter
#
# Author:       Yuseung Na (ys.na0220@gmail.com)
# Version:      1.0
#
# Revision History
#       January 19, 2021: Yuseung Na, Created
#

import numpy as np
import os
import argparse
from pypcd import pypcd
import csv
from tqdm import tqdm
import open3d as o3d


def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")
    parser.add_argument(
        "--pcd_path",
        help=".pcd file path.",
        type=str,
        default="./pcd_files"
    )
    parser.add_argument(
        "--bin_path",
        help=".bin file path.",
        type=str,
        default="./bin_files"
    )
    parser.add_argument(
        "--file_name",
        help="File name.",
        type=str,
        default="converted"
    )
    args = parser.parse_args()

    ## Find all pcd files
    pcd_files = []
    file_names = []
    for (path, dir, files) in os.walk(args.pcd_path):
        for filename in files:
            # print(filename)
            ext = os.path.splitext(filename)[-1]
            if ext == '.pcd':
                pcd_files.append(path + "/" + filename)
                file_names.append(filename[:-4])

    ## Sort pcd files by file name
    pcd_files.sort()
    file_names.sort()
    print("Finish to load point clouds!")

    ## Make bin_path directory
    try:
        if not (os.path.isdir(args.bin_path)):
            os.makedirs(os.path.join(args.bin_path))
    except OSError as e:
        if e.errno != errno.EEXIST:
            print("Failed to create directory!!!!!")
            raise

    ## Generate csv meta file
    csv_file_path = os.path.join(args.bin_path, "meta.csv")
    csv_file = open(csv_file_path, "w")
    meta_file = csv.writer(
        csv_file, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
    )
    ## Write csv meta file header
    meta_file.writerow(
        [
            "pcd file name",
            "bin file name",
        ]
    )
    print("Finish to generate csv meta file")

    ## Converting Process
    print("Converting Start!")
    seq = 0
    for pcd_file in tqdm(pcd_files):
        ## Get pcd file
        pc = pypcd.PointCloud.from_path(pcd_file)
        print(pcd_file)
        print(file_names[seq])

        ## Generate bin file name
        bin_file_name = "{}_{}.bin".format(args.file_name, file_names[seq])
        bin_file_path = os.path.join(args.bin_path, bin_file_name)

        ## Get data from pcd (x, y, z, intensity, ring, time)
        np_x = (np.array(pc.pc_data['x'], dtype=np.float32)).astype(np.float32)
        np_y = (np.array(pc.pc_data['y'], dtype=np.float32)).astype(np.float32)
        np_z = (np.array(pc.pc_data['z'], dtype=np.float32)).astype(np.float32)
        # np_i = (np.array(pc.pc_data['intensity'], dtype=np.float32)).astype(np.float32)/256

        # print(len(np_x))
        # print(len(np_y))
        # print(len(np_z))
        try:
            np_i = (np.array(pc.pc_data['intensity'], dtype=np.float32)).astype(np.float32) / 256
        except:
            shape = (len(np_x))
            np_i = np.ones(shape)
        # np_r = (np.array(pc.pc_data['ring'], dtype=np.float32)).astype(np.float32)
        # np_t = (np.array(pc.pc_data['time'], dtype=np.float32)).astype(np.float32)

        ## Stack all data
        x_max = max(np_x)
        x_min = min(np_x)

        y_max = max(np_y)
        y_min = min(np_y)

        z_max = max(np_z)
        z_min = min(np_z)
        print('x最大值/最小值：', x_max, x_min)
        print('y最大值/最小值：', y_max, y_min)
        print('z最大值/最小值：', z_max, z_min)

        x_tar = [-40, 40]
        y_tar = [-3, 20]
        z_tar = [0, 70.4]

        # np_x = (((np_x - x_min) * (x_tar[1] - x_tar[0]) / (x_max - x_min)) + x_tar[0]).astype(np.float32)
        # np_y = (((np_y - y_min) * (y_tar[1] - y_tar[0]) / (y_max - y_min)) + y_tar[0]).astype(np.float32)
        # np_z = (((np_z - z_min) * (z_tar[1] - z_tar[0]) / (z_max - z_min)) + z_tar[0]).astype(np.float32)

        points_32 = np.transpose(np.vstack((np_x, np_y, np_z, np_i))).reshape(-1)

        points_32 = points_32.astype(np.float32)

        ## Save bin file
        points_32.tofile(bin_file_path)
        # with open(bin_file_path, 'w') as f:  # Save as bin format
        #     points_32.tofile(f)

        ## Write csv meta file
        meta_file.writerow(
            [os.path.split(pcd_file)[-1], bin_file_name]
        )

        seq = seq + 1

    # data = np.fromfile('./bin_files/converted_car.bin', dtype=np.float32)
    # data = np.fromfile('./bin_files/converted_00001.bin')
    # print(data.shape)
    # data2 = data.reshape(-1, 4)
    # print(data2.shape)

    # data2[data2[:, 2] > -3]

    # pcd = o3d.geometry.PointCloud()
    # data3 = data2[:, :3]
    # pcd.points = o3d.utility.Vector3dVector(data3)

    # pcd = o3d.io.read_point_cloud("./pcd_files/SampleScene.pcd")
    # o3d.io.write_point_cloud("output.bin", pcd)

    # pcd.points = o3d.utility.Vector3dVector(data)

    # o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    main()
