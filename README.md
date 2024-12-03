# ROS2 video conversion 

## Instructions

Clone the repository in `~/colcon_ws/src` and build it using 
```bash
git clone https://github.com/TonyOx369/ROS2-video-conversion.git
```
```bash
source /opt/ros/humble/setup.bash
```
```bash
colcon build
```  
Open three different terminals and run 

```bash
source ~/colcon_ws/install/setup.bash
```
```bash
source /opt/ros/humble/setup.bash
```
in each of them.


1. Launch the `usb_cam` node:
   ```bash
   ros2 launch image_conversion image_conversion.launch.py
   ```
2. Run the `image_conversion` node to display the video feed:
  ```bash
  ros2 run image_conversion image_conversion
  ```
3. Call the service `switch_mode` to switch the mode of the node:
>>a. Convert the original RGB input image to Grayscale
  ```bash
  ros2 service call /switch_mode std_srvs/srv/SetBool "{data: true}"
  ```
  >>b. Convert the Grayscale image back to RGB
  ```bash
  ros2 service call /switch_mode std_srvs/srv/SetBool "{data: false}"
  ```
## Results
https://drive.google.com/file/d/1FK0jH_MTd4GoOYdXZUucFLUbNprO8aek/view?usp=sharing
