// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <openni_tracker/User.h>
#include <openni_tracker/UserList.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <vector>
#include <string>
#include <std_msgs/UInt16MultiArray.h>
#include <stdint.h>

using std::string;
using namespace openni_tracker;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";
double smoothing;
int num_users;
XnSkeletonProfile skeleton_profile;

ros::Publisher users_pub;
UserList user_list;


void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().SetSmoothing(smoothing);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);

	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

geometry_msgs::PoseStamped get_joint_pose(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id) {

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
//    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    //Convert to pose stamped msg
    tf::Stamped<tf::Pose> tf_stamped(transform, ros::Time::now(), frame_id);
    geometry_msgs::PoseStamped msg;
    tf::poseStampedTFToMsg(tf_stamped, msg);

    return msg;
}

void publish_user_data(const std::string& frame_id) {
    XnUserID users[num_users];
    XnUInt16 users_count = num_users;
    g_UserGenerator.GetUsers(users, users_count);

    user_list.users.clear();

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;

        User msg;
        msg.user_id = user;
        msg.head = get_joint_pose(user, XN_SKEL_HEAD, frame_id);
        msg.neck = get_joint_pose(user, XN_SKEL_NECK, frame_id);
        msg.torso = get_joint_pose(user, XN_SKEL_TORSO, frame_id);

        if(skeleton_profile == XN_SKEL_PROFILE_ALL || skeleton_profile == XN_SKEL_PROFILE_UPPER || skeleton_profile == XN_SKEL_PROFILE_HEAD_HANDS)
        {
            msg.left_shoulder = get_joint_pose(user, XN_SKEL_LEFT_SHOULDER, frame_id);
            msg.left_elbow = get_joint_pose(user, XN_SKEL_LEFT_ELBOW, frame_id);
            msg.left_hand = get_joint_pose(user, XN_SKEL_LEFT_HAND, frame_id);

            msg.right_shoulder = get_joint_pose(user, XN_SKEL_RIGHT_SHOULDER, frame_id);
            msg.right_elbow = get_joint_pose(user, XN_SKEL_RIGHT_ELBOW, frame_id);
            msg.right_hand = get_joint_pose(user, XN_SKEL_RIGHT_HAND, frame_id);
		}
		else

        if(skeleton_profile == XN_SKEL_PROFILE_LOWER || skeleton_profile == XN_SKEL_PROFILE_ALL)
        {
            msg.left_hip = get_joint_pose(user, XN_SKEL_LEFT_HIP, frame_id);
            msg.left_knee = get_joint_pose(user, XN_SKEL_LEFT_KNEE, frame_id);
            msg.left_foot = get_joint_pose(user, XN_SKEL_LEFT_FOOT, frame_id);

            msg.right_hip = get_joint_pose(user, XN_SKEL_RIGHT_HIP, frame_id);
            msg.right_knee = get_joint_pose(user, XN_SKEL_RIGHT_KNEE, frame_id);
            msg.right_foot = get_joint_pose(user, XN_SKEL_RIGHT_FOOT, frame_id);
		}

		user_list.users.push_back(msg);
    }

    users_pub.publish(user_list);
}

XnSkeletonProfile skeleton_profile_from_string(string skeleton_profile_str)
{
	if(skeleton_profile_str == "XN_SKEL_PROFILE_ALL")
	{
		return XN_SKEL_PROFILE_ALL;
	}
	else if(skeleton_profile_str == "XN_SKEL_PROFILE_UPPER")
	{
		return XN_SKEL_PROFILE_UPPER;
	}
	else if(skeleton_profile_str == "XN_SKEL_PROFILE_LOWER")
	{
		return XN_SKEL_PROFILE_LOWER;
	}
	else if(skeleton_profile_str == "XN_SKEL_PROFILE_HEAD_HANDS")
	{
		return XN_SKEL_PROFILE_HEAD_HANDS;
	}
	else
	{
		ROS_ERROR("%s isn't a valid skeleton profile. Setting to default (XN_SKEL_PROFILE_ALL) instead.", skeleton_profile_str.c_str());
		return XN_SKEL_PROFILE_ALL;
	}
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int main(int argc, char **argv) {
    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); //Relative node handle

    //Get parameters
    string frame_id("openni_depth_frame");
    pnh.getParam("camera_frame_id", frame_id); 
    pnh.param("num_users", num_users, 15);
    pnh.param("smoothing", smoothing, 0.7); // changes the smoothing value applied to skeletons
    
    string skeleton_profile_str = "";
    string skeleton_profile_default = "XN_SKEL_PROFILE_ALL";
    pnh.param("skeleton_profile", skeleton_profile_str, skeleton_profile_default); // set what body region to track
    skeleton_profile = skeleton_profile_from_string(skeleton_profile_str);
    
    //Init active users array and publisher
    users_pub = pnh.advertise<UserList>("user_list", 10);

    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(skeleton_profile);
	
	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);
                
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		publish_user_data(frame_id);
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
