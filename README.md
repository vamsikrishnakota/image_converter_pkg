# Image Conversion Pkg




## Prerequisites

Before setting up the package, ensure you have the following:

- **ROS 2 Humble Hawksbill** installed. ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **Installation of usb_cam package** 
    ```bash
    sudo apt-get install ros-humble-usb-cam
- OpenCV installed (usually included with `cv_bridge`).  
- `colcon` build tool installed.  
- `image_transport` and `cv_bridge` ROS 2 dependencies.

## Step-by-Step Procedure for creating the workspace
# 1. Create the workspace 
- Create the workspace 
    ```bash
    mkdir -p ~/ros2_ws/src
# 2. Clone the Repository
- Change the path into source directory of the workspace
    ```bash
    cd ~/ros2_ws/src
- Clone the repository from github
    ```bash
    git clone https://github.com/MarthaSuryaTeja/image_conversion_pkg.git
# 3. Build the Package 
- Change the path to ros2_ws directory
    ```bash
    cd ..
- Build the Package
    ```bash
    colcon build

## Run the Node
- Open a terminal and source the workspace
    ```bash
    source ~/ros2_ws/install/setup.bash
- Launch the image_conversion node with the following command
    ```bash
    ros2 launch image_conversion_pkg image_conversion_launch.py
This will start the image_Conversion node and collects the data from the camera

## Check the results
- Open another terminal and paste the below command
    ```bash
    ros2 run rqt_image_view rqt_image_view
This will open a window which show the feed of /camera
### Mode - 1 (Gray Scale Image)
- To check the gray scale converted image. Use below command
    ```bash
    ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"
- This is the service which is assigned to change the image feed into the gray scale format
![Screenshot from 2024-12-04 22-50-36](https://github.com/user-attachments/assets/b52f8f39-65aa-4339-86ba-a3d49112e744)


### Mode - 2 (Coloured Image)
- To change the converted image into Colour, then paste the below command in new terminal
    ```bash
    ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
- This is the service which is assigned to change the image feed into the gray scale format
- After entering this command, open the rqt_image_view window and change the drop down to /camera/converted_image

![Screenshot from 2024-12-04 22-51-05](https://github.com/user-attachments/assets/8db27425-d3a8-429a-84d5-570d60c44715)


