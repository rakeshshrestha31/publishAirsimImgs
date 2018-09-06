# Disclaimer
Most of the code in this repo was developed by Behzad Boroujerdian and Hasan Genc from the Department Of Electrical and Computer Engineering at The University of Texas at Austin. I put some finishing touches and wrote the guidelines below in the hope that this will be useful for others. This is supposed to be a preliminary solution while Microsoft don't provide an alternative one.

This repo was forked from github user: rakeshshrestha31  

# Changes
I have made changes in the fork of Microsoft's AirSim to build with g++ which is necessary to work with ROS on the client side.  
The modified fork can be found [in this repository](https://github.com/aravindk2604/AirSim.git).  

This `publishAirsimImgs` repository contains a simple example to demonstrate the link betweeen AirSim v1.2, Unreal 4.18 and ROS Kinetic tested on Ubuntu 16.04.  
I have written a simple example modifiying the code already developed by the previous contributors mentioned in the `Disclaimer` section and incorporated new APIs to stream images on a ROS topic called **front_center/image_raw** visualized on RVIZ. This data is obtained from the `front_center` camera of a car model used in AirSim whose movements are controlled by the `teleop` code written for TurtleBot. The video below shows the demonstration of this example.  
  
[![AirSim_to_RVIZ_image_streaming](extras/airsim_rviz_picture.png)](https://youtu.be/Ubqx9WifekQ)
  

This is still work in progress and I will update in detail on how to build and run the example.

