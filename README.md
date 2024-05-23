
<!--
 <p align="center">
  <a href="" target='_blank'>
    <img src="https://visitor-badge.laobi.icu/badge?page_id=PardisTaghavi.real_time_tracking_AB3DMOT&left_color=gray&right_color=red">
  </a>
</p>
-->

### "AB3DMOT" modification for real-time tracking in ROS
PointPillar(Not used mmdetection) + SORT

### Result

![result2](https://github.com/skymun471/Ros-3D-Object-Tracking/assets/41955439/5844cc97-0b53-476d-871c-1dc16303d852)

### Requirements
Ubuntu 20.04/ ROS Noetic

### Installation

- clone this repo in catkin_ws/src directory:
- your env setting
1. ``` git clone https://github.com/skymun471/Ros-3D-Object-Tracking.git ```
2. ``` cd real_time_tracking_AB3DMOT/src/PointPillars/AB3DMOT && pip3 install -r requirements.txt ```
3. ``` cd real_time_tracking_AB3DMOT/src/PointPillars/AB3DMOT/Xinshuo_PyToolbox && pip3 install -r requirements.txt```
4. ``` cd real_time_tracking_AB3DMOT/src/PointPillars/ops && python3 setup.py develop```
4. ``` export PYTHONPATH=${PYTHONPATH}:/home/user/workspace/code/AB3DMOT```
5. ``` export PYTHONPATH=${PYTHONPATH}:/home/user/workspace/code/AB3DMOT/Xinshuo_PyToolbox```
6. ``` source ~/.profile && cd path/to/Ros-3D-Object-Tracking && source env/bin/activate```
### How to use
- PointPillars Datasets Download
Download

- [point cloud](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_velodyne.zip)(29GB), [images](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_image_2.zip)(12 GB), [calibration files](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_calib.zip)(16 MB)[labels](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_label_2.zip)(5 MB)。
- Format the datasets as follows
```
    kitti
        |- training
            |- calib (#7481 .txt)
            |- image_2 (#7481 .png)
            |- label_2 (#7481 .txt)
            |- velodyne (#7481 .bin)
        |- testing
            |- calib (#7518 .txt)
            |- image_2 (#7518 .png)
            |- velodyne (#7518 .bin)
```
- Pre-process KITTI datasets First
```
    cd real_time_tracking_AB3DMOT/src/PointPillars/
    python3 pre_process_kitti.py --data_root your_path_to_kitti
```
- Now, we have datasets as follows:
```
    kitti
        |- training
            |- calib (#7481 .txt)
            |- image_2 (#7481 .png)
            |- label_2 (#7481 .txt)
            |- velodyne (#7481 .bin)
            |- velodyne_reduced (#7481 .bin)
        |- testing
            |- calib (#7518 .txt)
            |- image_2 (#7518 .png)
            |- velodyne (#7518 .bin)
            |- velodyne_reduced (#7518 .bin)
        |- kitti_gt_database (# 19700 .bin)
        |- kitti_infos_train.pkl
        |- kitti_infos_val.pkl
        |- kitti_infos_trainval.pkl
        |- kitti_infos_test.pkl
        |- kitti_dbinfos_train.pkl
```
-------------------------------------------------------------------
- [PointPillars Detector Training]
```
    cd real_time_tracking_AB3DMOT/src/PointPillars && python3 train.py --data_root your_path_to_kitti
```
-------------------------------------------------------------------
- [PointPillars Detector Evaluation]
```
    python3 evaluate.py --ckpt pretrained/epoch_160.pth --data_root your_path_to_kitti
``` 
-------------------------------------------------------------------
- [PointPillars Detector Test]
```
# 1. infer and visualize point cloud detection
python3 test.py --ckpt pretrained/epoch_160.pth --pc_path your_pc_path 

# 2. infer and visualize point cloud detection and gound truth.
python3 test.py --ckpt pretrained/epoch_160.pth --pc_path your_pc_path --calib_path your_calib_path  --gt_path your_gt_path

# 3. infer and visualize point cloud & image detection
python3 test.py --ckpt pretrained/epoch_160.pth --pc_path your_pc_path --calib_path your_calib_path --img_path your_img_path
```
-------------------------------------------------------------------
- [PointPillars Detecting + SORT Tracking result Evaluation]
1. Kitti Tracking Dataset Preparation

Please download the official [KITTI multi object tracking](http://www.cvlibs.net/datasets/kitti/eval_tracking.php) dataset 

[left color images], [velodyne point cloud], [GPS/IMU data], [training labels], [camera calibration] data are needed.
```
    AB3DMOT
    ├── data
    │   ├── KITTI
    │   │   │── tracking
    │   │   |   │── training
    │   │   │   │   ├──calib & velodyne & label_02 & image_02 & oxts
    │   │   │   │── testing
    │   │   │   │   ├──calib & velodyne & image_02 & oxts
    ├── AB3DMOT_libs
    ├── configs
```


2. 3D Multi-Object Tracking  (This make the pointrcnn_val_H1 in results directory)
```
python3 main.py --dataset KITTI --det_name pointrcnn 
```
!!! I'm going to modify 'main.py' to use the PointPillars detection model. !!!

!!! main.py generates the result files of the detection model, and evaluation.py uses them. !!!


3. 3D MOT Evaluation(pointrcnn_val_H1 in results directory)

set with a threshold of 0.25 3D IoU during evaluation
```
python3 scripts/KITTI/evaluate.py pointrcnn_val_H1 1 3D 0.25 
```

-------------------------------------------------------------------
- ROS PointPillar + SORT

1. ``` catkin_make```
and Don't forget to source the package ```source devel/setup.bash ```

2. ``` chmod +x modelROS.py``` (do it one time to change accessibility of the file)
3. ``` cd real_time_tracking_AB3DMOT/src/PointPillars && python3 modelROS.py ```
4. ``` rosbag play [rosbag.bag] ```
5. ``` rviz ```

-------------------------------------------------------------------

