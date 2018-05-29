# astra_body_tracker for Orbbec Astra camera, using the Astra SDK

## Description:
   - Publishes ROS topic for body tracking information from the Astra SDK.  
   - Tested on Orbbec Pro, but not Mini yet.
   - Publishes BodyInfo.msg tracking info on "astra_body_tracker/data"

#  Installation Instructions / Prerequisites
   - Clone body_tracker_msgs into your Catkin workspace and catkin_make to confirm complies OK
   - Clone this project into your Catkin workspace
   - Install Orbbec Astra SDK from https://orbbec3d.com/develop/
   - Add the following to your .bashrc (or the code won't compile!)
     (this might not all be required, but works)

      # Orbbec Camera:
      export OPENNI2_INCLUDE=/home/system/OpenNI-Linux-x64-2.3/Include
      export OPENNI2_REDIST=/home/system/OpenNI-Linux-x64-2.3/Redist

      export ASTRA_SDK=/home/system/AstraSDK
      export ASTRA_ROOT=/home/system/AstraSDK

      export ASTRA_SDK_INCLUDE=/home/system/AstraSDK/install/include
      export ASTRA_SDK_LIB=/home/system/AstraSDK/install/lib



