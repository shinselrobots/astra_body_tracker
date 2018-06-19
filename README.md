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

# Testing

    - First, make sure the Astra SDK is working correctly:
      - cd <your directory>/AstraSDK/bin
      - ./SimpleBodyViewer-SFML
      You should see body tracking with Skeleton displayed.  If not, see Astra website.

    - When the tracker is running, you should see something like this logged: 
      "Floor mask: width: 640 height: 480 bottom center value:"

    - When tracking a person, you should see something like this logged:
      Body Id: 41 Status: Tracking
      Body 41 CenterOfMass (-143.216354, 15.024889, 827.818970)
          Joint Tracking Enabled: True     Hand Pose Recognition Enabled: True
      Astra: 2D Tracking for ID 41 :   px: 0.2441 py: 0.1192
          x: -0.07321 y: 0.1683 z: 0
                Head:Body 41 Joint 0 status 2 @ world (-27.8, 315.9, 974.3) depth (303.7, 55.1)

    - When tracking, the following are published:
      - rostopic list
        /body_tracker/marker
        /body_tracker/position
        /body_tracker/skeleton
      - Confirm each with rostopic echo

    - If the camera is mounted on a robot, and included in the URDF, the /body_tracker/marker
      topic will show the position of the user (spine joints) as balls in RVIZ.
      - make sure to add the topic /body_tracker/marker to see the tracking balls.

# Known Issues
    - You can safely ignore this warning: 
      "Warning: USB events thread - failed to set priority. This might cause loss of data..."      
    - When robot is moving (or if Astra camera is mounted on a Pan/Tilt that is tracking a target)
      the Astra SDK will often think some object is a person.  So, it is important in your code to track
      individuals by ID, and ignore other ID's that are picked up (they may be bogus).
      I find it useful to use the Gesture to indicate which person in a scene is the correct person to track.
      
# Licensing
    - Code written by me is provided under BSD license
    - This package uses sample code from the Astra SDK, provided by Orbbec.  See Orbbec Astra SDK for licensing.




