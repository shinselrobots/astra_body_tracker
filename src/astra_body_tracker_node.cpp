/* Body Tracker Node 
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "ros/console.h"

//For Orbbec Astra SDK
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

#include <astra_body_tracker/BodyInfo.h>  // Publish custom message

using namespace astra_body_tracker;

class astra_body_tracker_node 
{
public:
  astra_body_tracker_node(std::string name) :
    _name(name)
  {
    ROS_INFO("%s: Initializing", _name.c_str());
    bool initialized = false;

    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1",myparm1_,"mydefault");

    // Subscribers
    //robot_behavior_state_ = nh_.subscribe("/behavior/cmd", 1, &behavior_logic_node::behaviorStateCB, this);

    // PUBLISHERS
    body_tracking_data_pub_ = nh_.advertise<astra_body_tracker::BodyInfo>("astra_body_tracker/data", 1);

    ROS_INFO("astra_body_tracker: Advertised Publisher: astra_body_tracker/data");

  }

  ~astra_body_tracker_node()
  {
    ROS_INFO("astra_body_tracker_node shutting down");
  }



//////////////////////////////////////////////////////////
// Modified Orbec Astra sample code

void output_floor(astra_bodyframe_t bodyFrame)
{
    astra_floor_info_t floorInfo;

    astra_status_t rc = astra_bodyframe_floor_info(bodyFrame, &floorInfo);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_floor_info()\n", rc);
        return;
    }

    const astra_bool_t floorDetected = floorInfo.floorDetected;
    const astra_plane_t* floorPlane = &floorInfo.floorPlane;
    const astra_floormask_t* floorMask = &floorInfo.floorMask;

    if (floorDetected != ASTRA_FALSE)
    {
        printf("Floor plane: [%f, %f, %f, %f]\n",
               floorPlane->a,
               floorPlane->b,
               floorPlane->c,
               floorPlane->d);

        const int32_t bottomCenterIndex = floorMask->width / 2 + floorMask->width * (floorMask->height - 1);
        printf("Floor mask: width: %d height: %d bottom center value: %d\n",
            floorMask->width,
            floorMask->height,
            floorMask->data[bottomCenterIndex]);
    }
}

void output_body_mask(astra_bodyframe_t bodyFrame)
{
    astra_bodymask_t bodyMask;

    const astra_status_t rc = astra_bodyframe_bodymask(bodyFrame, &bodyMask);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_bodymask()\n", rc);
        return;
    }

    /*
    const int32_t centerIndex = bodyMask.width / 2 + bodyMask.width * bodyMask.height / 2;
    printf("Body mask: width: %d height: %d center value: %d\n",
        bodyMask.width,
        bodyMask.height,
        bodyMask.data[centerIndex]);
    */
}

void output_bodyframe_info(astra_bodyframe_t bodyFrame)
{
    astra_bodyframe_info_t info;

    const astra_status_t rc = astra_bodyframe_info(bodyFrame, &info);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_info()\n", rc);
        return;
    }

    // width and height of floor mask, body mask, and the size of depth image
    // that joint depth position is relative to.
    const int32_t width = info.width;
    const int32_t height = info.height;

    /*
    printf("BodyFrame info: Width: %d Height: %d\n",
        width,
        height);
    */
}



void output_joint(std::string joint_name, const int32_t bodyId, const astra_joint_t* joint)
{

    printf("%14s:", joint_name.c_str());

    // jointType is one of ASTRA_JOINT_* which exists for each joint type
    const astra_joint_type_t jointType = joint->type;

    // jointStatus is one of:
    // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
    // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
    // ASTRA_JOINT_STATUS_TRACKED = 2,
    const astra_joint_status_t jointStatus = joint->status;

    const astra_vector3f_t* worldPos = &joint->worldPosition;

    // depthPosition is in pixels from 0 to width and 0 to height
    // where width and height are member of astra_bodyframe_info_t
    // which is obtained from astra_bodyframe_info().
    const astra_vector2f_t* depthPos = &joint->depthPosition;

    printf("Body %u Joint %d status %d @ world (%.1f, %.1f, %.1f) depth (%.1f, %.1f)\n",
           bodyId,
           jointType,
           jointStatus,
           worldPos->x,
           worldPos->y,
           worldPos->z,
           depthPos->x,
           depthPos->y);

    // orientation is a 3x3 rotation matrix where the column vectors also
    // represent the orthogonal basis vectors for the x, y, and z axes.
    /* Not sure I need orientation for anything yet
    const astra_matrix3x3_t* orientation = &joint->orientation;
    const astra_vector3f_t* xAxis = &orientation->xAxis; // same as orientation->m00, m10, m20
    const astra_vector3f_t* yAxis = &orientation->yAxis; // same as orientation->m01, m11, m21
    const astra_vector3f_t* zAxis = &orientation->zAxis; // same as orientation->m02, m12, m22

    printf("Head orientation x: [%f %f %f]\n", xAxis->x, xAxis->y, xAxis->z);
    printf("Head orientation y: [%f %f %f]\n", yAxis->x, yAxis->y, yAxis->z);
    printf("Head orientation z: [%f %f %f]\n", zAxis->x, zAxis->y, zAxis->z);
    */
}

void output_hand_poses(const astra_body_t* body)
{
    const astra_handpose_info_t* handPoses = &body->handPoses;

    // astra_handpose_t is one of:
    // ASTRA_HANDPOSE_UNKNOWN = 0
    // ASTRA_HANDPOSE_GRIP = 1
    const astra_handpose_t leftHandPose = handPoses->leftHand;
    const astra_handpose_t rightHandPose = handPoses->rightHand;

    printf("Body %d Left hand pose: %d Right hand pose: %d\n",
        body->id,
        leftHandPose,
        rightHandPose);
}

void output_bodies(astra_bodyframe_t bodyFrame)
{
    int i;
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_body_list()\n", rc);
        return;
    }

    for(i = 0; i < bodyList.count; ++i)
    {
        astra_body_t* body = &bodyList.bodies[i];

        // Pixels in the body mask with the same value as bodyId are
        // from the same body.
        astra_body_id_t bodyId = body->id;

        // Tracking status
        // NOT_TRACKING = 0
        // TRACKING_LOST = 1
        // TRACKING_STARTED = 2
        // TRACKING = 3

        astra_body_status_t bodyStatus = body->status;
        if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED)
        {
            printf("Body Id: %d Status: Tracking started\n", bodyId);
        }
        else if (bodyStatus == ASTRA_BODY_STATUS_TRACKING)
        {
            printf("Body Id: %d Status: Tracking\n", bodyId);
        }
        else if (bodyStatus == ASTRA_BODY_STATUS_LOST)
        {
            printf("Body %u Status: Tracking lost.\n", bodyId);
        }
        else // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
        {
            printf("Body Id: %d Status: Not Tracking\n", bodyId);
        }

        const astra_vector3f_t* centerOfMass = &body->centerOfMass;
        const astra_body_tracking_feature_flags_t features = body->features;
        astra_joint_t* joint;  // AstraSDK/include/astra/capi/streams/body_types.h

        const bool jointTrackingEnabled = 
          (features & ASTRA_BODY_TRACKING_JOINTS) == ASTRA_BODY_TRACKING_JOINTS;
        const bool handPoseRecognitionEnabled = 
          (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

        printf("Body %d CenterOfMass (%f, %f, %f)\n",
            bodyId, centerOfMass->x, centerOfMass->y, centerOfMass->z);
        printf("    Joint Tracking Enabled: %s     Hand Pose Recognition Enabled: %s\n",
            jointTrackingEnabled       ? "True" : "False",
            handPoseRecognitionEnabled ? "True" : "False");

        ///////////////////////////////////////////////////////////////
        // Publish body tracking information, and display joint info for debug

        // Create structure for ROS Publisher data
        astra_body_tracker::BodyInfo_ <astra_body_tracker::BodyInfo> body_info;

        // Fill in ROS body_info structure from AstraSDK data
        body_info.body_id = bodyId;
        body_info.tracking_status = bodyStatus;

        body_info.centerOfMass.x = centerOfMass->x;
        body_info.centerOfMass.y = centerOfMass->y;
        body_info.centerOfMass.z = centerOfMass->z;

        joint = &body->joints[ASTRA_JOINT_HEAD];
        output_joint("Head", bodyId, joint );
        body_info.joint_position_head.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_head.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_head.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_NECK];
        output_joint("Neck", bodyId, joint );
        body_info.joint_position_neck.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_neck.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_neck.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_SHOULDER_SPINE];
        output_joint("Spine Top", bodyId, joint );
        body_info.joint_position_spine_top.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_spine_top.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_spine_top.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_MID_SPINE];
        output_joint("Spine Mid", bodyId, joint );
        body_info.joint_position_spine_mid.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_spine_mid.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_spine_mid.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_BASE_SPINE];
        output_joint("Spine Base", bodyId, joint );
        body_info.joint_position_spine_bottom.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_spine_bottom.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_spine_bottom.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_LEFT_SHOULDER];
        output_joint("Left Shoulder", bodyId, joint );
        body_info.joint_position_left_shoulder.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_left_shoulder.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_left_shoulder.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_LEFT_ELBOW];
        output_joint("Left Elbow", bodyId, joint );
        body_info.joint_position_left_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_left_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_left_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_LEFT_HAND];
        output_joint("Left Hand", bodyId, joint );
        body_info.joint_position_left_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_left_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_left_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_LEFT_SHOULDER];
        output_joint("Right Shoulder", bodyId, joint );
        body_info.joint_position_right_shoulder.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_right_shoulder.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_right_shoulder.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_LEFT_ELBOW];
        output_joint("Right Elbow", bodyId, joint );
        body_info.joint_position_right_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_right_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_right_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        joint = &body->joints[ASTRA_JOINT_LEFT_HAND];
        output_joint("Right Hand", bodyId, joint );
        body_info.joint_position_right_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->x;
        body_info.joint_position_right_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->y;
        body_info.joint_position_right_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->z;

        // Is hand open (0) or grasping (1)?
        output_hand_poses(body);
        const astra_handpose_info_t* handPoses = &body->handPoses;
        body_info.left_hand_closed = handPoses->leftHand;
        body_info.right_hand_closed = handPoses->rightHand;

        // Publish Body Data
        body_tracking_data_pub_.publish(body_info);

        printf("\n----------------------------\n\n");

    }
}

void output_bodyframe(astra_bodyframe_t bodyFrame)
{
    output_floor(bodyFrame);

    output_body_mask(bodyFrame);

    output_bodyframe_info(bodyFrame);

    output_bodies(bodyFrame);
}

void runLoop()
{
    set_key_handler();
    astra_initialize();
    const char* licenseString = "<INSERT LICENSE KEY HERE>";
    orbbec_body_tracking_set_license(licenseString);

    astra_streamsetconnection_t sensor;
    astra_streamset_open("device/default", &sensor);

    astra_reader_t reader;
    astra_reader_create(sensor, &reader);

    astra_bodystream_t bodyStream;
    astra_reader_get_bodystream(reader, &bodyStream);

    astra_stream_start(bodyStream);

    do
    {
        astra_update();

        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(reader, 0, &frame);

        if (rc == ASTRA_STATUS_SUCCESS)
        {
            astra_bodyframe_t bodyFrame;
            astra_frame_get_bodyframe(frame, &bodyFrame);

            astra_frame_index_t frameIndex;
            astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);
            // printf("Frame index: %d\n", frameIndex);

            output_bodyframe(bodyFrame);

            //printf("----------------------------\n");

            astra_reader_close_frame(&frame);
        }

        ros::spinOnce();  // ROS

    } while (shouldContinue);

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);

    astra_terminate();

}




//////////////////////////////////////////////////////////
private:
  std::string _name;
  ros::NodeHandle nh_;
  //ros::Subscriber robot_behavior_state_;
  std::string myparm1_;

  //ros::Publisher body_tracking_status_pub_;
  ros::Publisher body_tracking_data_pub_;

};


// The main entry point for this node.
int main( int argc, char *argv[] )
{
  ros::init( argc, argv, "astra_body_tracker" );
  astra_body_tracker_node node(ros::this_node::getName());
  node.runLoop();
  //ros::spin();

  return 0;
}



