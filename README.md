# **Storm2025雷达站**

#### 成果展示：

- ##### 分区赛

敌方地面兵种全部标记

![](/home/wenyu/A_radar_Storm2025/images-2025/1.jpeg)

------

- ##### 复活赛

英雄在补给点也可标记

![](/home/wenyu/A_radar_Storm2025/images-2025/2.png)

#### 结构：

wenyu@wenyu:~/A_radar_Storm2025$ tree
├── arrays_test_blue.npy
├── arrays_test_blue.npy_image_points.npy
├── arrays_test_red.npy
├── calibration_fixed.py
├── config.yaml
├── dahua
│   ├── dahua_camera.py
│   ├── ImageConvert.py
│   ├── MVSDK.py
│   └── __pycache__
│       ├── dahua_camera.cpython-310.pyc
│       ├── ImageConvert.cpython-310.pyc
│       └── MVSDK.cpython-310.pyc
├── detect_function.py
├── export.py
├── hik
│   ├── hik_camera.py
│   └── __pycache__
│       └── hik_camera.cpython-310.pyc
├── images
│   ├── 253Dtest.png
│   ├── img.png
│   ├── map_blue.jpg
│   ├── map.jpg
│   ├── map_mask.jpg
│   ├── map_red.jpg
│   ├── map_red_s_mask.jpg
│   └── test_image.jpg
├── images-2025
│   ├── map_blue.jpg
│   ├── map_mask.jpg
│   ├── map.png
│   └── map_red.jpg
├── information_ui.py
├── LICENSE
├── livox
│   ├── livox_handler2.py
│   ├── livox_handler.py
│   ├── livox_lidar_config.json
│   └── __pycache__
│       └── livox_handler.cpython-310.pyc
├── main.py
├── models
│   ├── armor.engine
│   ├── armor.onnx
│   ├── armor.pt
│   ├── car.engine
│   ├── car.onnx
│   ├── car.pt
│   ├── common.py
│   ├── experimental.py
│   ├── __pycache__
│   │   ├── common.cpython-310.pyc
│   │   ├── experimental.cpython-310.pyc
│   │   └── yolo.cpython-310.pyc
│   ├── train_log.txt
│   └── yolo.py
├── MvImport
│   ├── CameraParams_const.py
│   ├── CameraParams_header.py
│   ├── MvCameraControl_class.py
│   ├── MvErrorDefine_const.py
│   └── PixelType_header.py
├── MvImport_Linux
│   ├── CameraParams_const.py
│   ├── CameraParams_header.py
│   ├── MvCameraControl_class.py
│   ├── MvErrorDefine_const.py
│   ├── PixelType_const.py
│   ├── PixelType_header.py
│   └── __pycache__
│       ├── CameraParams_const.cpython-310.pyc
│       ├── CameraParams_header.cpython-310.pyc
│       ├── MvCameraControl_class.cpython-310.pyc
│       ├── MvErrorDefine_const.cpython-310.pyc
│       ├── PixelType_const.cpython-310.pyc
│       └── PixelType_header.cpython-310.pyc
├── onnx2engine.py
├── __pycache__
│   ├── detect_function.cpython-310.pyc
│   ├── export.cpython-310.pyc
│   ├── hik_camera.cpython-310.pyc
│   ├── information_ui.cpython-310.pyc
│   ├── pointcloud_recorder.cpython-310.pyc
│   └── video_recorder.cpython-310.pyc
├── recordings
│   └── camera
├── RM_serial_py
│   ├── example_receive.py
│   ├── example_send.py
│   ├── __pycache__
│   │   └── ser_api.cpython-310.pyc
│   └── ser_api.py
├── rosbag.sh
├── save_img
│   └── game
│       ├── 1
│       └── 2
├── start_livox.sh
├── utils
│   ├── activations.py
│   ├── augmentations.py
│   ├── autoanchor.py
│   ├── autobatch.py
│   ├── aws
│   │   ├── __init__.py
│   │   ├── mime.sh
│   │   ├── resume.py
│   │   └── userdata.sh
│   ├── callbacks.py
│   ├── dataloaders.py
│   ├── docker
│   │   ├── Dockerfile
│   │   ├── Dockerfile-arm64
│   │   └── Dockerfile-cpu
│   ├── downloads.py
│   ├── flask_rest_api
│   │   ├── example_request.py
│   │   ├── README.md
│   │   └── restapi.py
│   ├── general.py
│   ├── __init__.py
│   ├── loggers
│   │   ├── clearml
│   │   │   ├── clearml_utils.py
│   │   │   ├── hpo.py
│   │   │   ├── __init__.py
│   │   │   └── README.md
│   │   ├── comet
│   │   │   ├── comet_utils.py
│   │   │   ├── hpo.py
│   │   │   ├── __init__.py
│   │   │   ├── optimizer_config.json
│   │   │   └── README.md
│   │   ├── __init__.py
│   │   └── wandb
│   │       ├── __init__.py
│   │       ├── log_dataset.py
│   │       ├── README.md
│   │       ├── sweep.py
│   │       ├── sweep.yaml
│   │       └── wandb_utils.py
│   ├── loss.py
│   ├── metrics.py
│   ├── plots.py
│   ├── __pycache__
│   │   ├── augmentations.cpython-310.pyc
│   │   ├── autoanchor.cpython-310.pyc
│   │   ├── dataloaders.cpython-310.pyc
│   │   ├── downloads.cpython-310.pyc
│   │   ├── general.cpython-310.pyc
│   │   ├── __init__.cpython-310.pyc
│   │   ├── metrics.cpython-310.pyc
│   │   ├── plots.cpython-310.pyc
│   │   └── torch_utils.cpython-310.pyc
│   ├── segment
│   │   ├── augmentations.py
│   │   ├── dataloaders.py
│   │   ├── general.py
│   │   ├── __init__.py
│   │   ├── loss.py
│   │   ├── metrics.py
│   │   ├── plots.py
│   │   └── __pycache__
│   │       ├── general.cpython-310.pyc
│   │       └── __init__.cpython-310.pyc
│   ├── torch_utils.py
│   └── triton.py
├── video_recorder.py
└── yaml
    ├── armor.yaml
    └── car.yaml



###### Q&A：

###### 2101383889@qq.com

###### https://github.com/Wenyu2405