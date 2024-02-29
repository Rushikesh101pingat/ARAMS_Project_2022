# **ARAMS PROJECT 2022**
## Student Names : 
### Rushikesh Pingat (Mat.Nr. 3300656)
### Bipeen Dhakal (Mat.Nr. 3282119 )

# Steps : 
1. Download all the dependencies eg. YoloX, pip, AprilTags, etc.
2. Clone the repository into your desired directory(workspace)
3. build the workspace: `colcon build`
4. source it:  ` source install/setup.bash`
5. Refer to section below for YOLOX installation(if the download of our source code does not work).
6. Then run following Launch files and Nodes in different terminals: 
7. 
        $ ros2 launch tb3_gazebo arams.launch.py 

        $ ros2 launch my_robot_slam clean_initiate.launch.py

        $ ros2 launch my_robot_slam clean_begin.launch.py

        $ ros2 launch yolox_ros_py yolox_m_openvino.launch.py

        $ ros2 launch apriltag_ros tag_16h5_all.launch.py

        $ ros2 run my_robot_slam auto_explorer






If the Gazebo world is Dark, Setting Scene -> shadows to **False** is a workaround.


## YOLOX Installation:

Installation:



STEP 1 : 

        pip3 install yolox

STEP 2 : 

        source /opt/ros/foxy/setup.bash

        sudo apt install ros-foxy-v4l2-camera

        pip install openvino

        pip install torch

        pip3 install yolox

        colcon build --symlink-install # weights (YOLOX-Nano) files will be installed automatically.

Step additional (if above steps does not work):

        git clone https://github.com/Ar-Ray-code/yolox_ros.git --recursive

        clone this in working directory


Important note: add the following "weights" folder into YOLOX-ROS folder in order to run this repo out of the box.

[Drive link]([https://link-url-here.org](https://drive.google.com/drive/folders/1h4USl24771hBSz2cAFnH-EDg5MkHZDTr?usp=drive_link))

If YOLOX still does not run, make sure you have installed following packages(if not downloaded automatically):

            -numpy
            -torch>=1.7
            -opencv_python
            -loguru
            -scikit-image
            -tqdm
            -torchvision
            -Pillow
            -thop
            -ninja
            -tabulate
            -tensorboard




