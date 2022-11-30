# ROS_reefscan_cots_detector
ROS node which simulates COTS detection (at random)

## Description:  
    reefscan_cots_detector_simulated.py
        A node called reefscan_cots_detector_simulated which simulates COTS 
        detection at random.


## Develop dependencies  

NONE
      
## Both prod and dev dependencies
    ccip_msgs (see https://github.com/AIMS/ccip_msgs)
    reefscan (see https://github.com/AIMS/ROS_reefscan)

## Prod only dependencies  

NONE

## Deploy  
   `mkdir -p ~/catkin_ws/src`
   # COPY everything in this repository to ~/catkin_ws/src/reefscan_cots_detector/
   `cd ~/catkin_ws`
   `chmod a+x src/reefscan_cots_detector/src/reefscan_cots_detector_simulated.py`  
   `catkin_make`  
   `source ~/catkin_ws/devel/setup.bash`  

## Start on DEV server

   `rosrun reefscan_cots_detector reefscan_cots_detector_simulated.py`