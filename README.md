## PCD2KITTI_BIN ##

To transform the customer made .pcd files into the .bin files of KITTI format.

*This project is built and tested on Ubuntu 16.04 and Ubuntu 20.04. However, due to its dependencies are not limited it to Linux system. Therefore, Installation of it on the windows system is also feasible.*

## Installation ##
### 0. Setup the virtual environment with anaconda ###

```
conda create --name pcd2kitti_bin --file requirements.txt
```

[The anaconda can be installed through its official website. ]([Anaconda | The Operating System for AI](https://www.anaconda.com/)) | **Detailed  instructions available!**

### 1. Launch python file ###

```
conda activate pcd2kitti_bin
cd ./pcd2kitti_bin
python pcd2bin.py --pcd_file_path="./pcd_files" --bin_file_path="./bin_files"
```

#### 2. Parameters ####

*To be noted that the transformed .bin files are named as the very same of its original .pcd files.*

|Name|Description|Default value|
|:---|:---|:---|
|--pcd_file_path|Path for .pcd files for transpose|"./pcd_files"|
|--bin_file_path|Path for .bin files for saving|"./bin_files"|

#### 3. Running results

The project is capable of visualizing an example result .bin file.

The result for Moon_Building_Barracks.bin is:

![01](pcd2kitti_bin/figs/01.png)

The result for MedievalHouse2.bin is:

![02](pcd2kitti_bin/figs/02.png)

## Citation

> This repository is based on the following project.
>
> [PCD2BIN of Yuseung-Na. ]([[Yuseung-Na/pcd2bin: .pcd to .bin converter (python) (github.com)](https://github.com/Yuseung-Na/pcd2bin))) 
