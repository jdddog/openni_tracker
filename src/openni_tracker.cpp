// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <vector>
#include <string>
#include <std_msgs/UInt16MultiArray.h>
#include <stdint.h>

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";
double smoothing;
int num_users;
XnSkeletonProfile skeleton_profile;

ros::Publisher users_pub;
std::vector<uint16_t> user_ids;
std_msgs::UInt16MultiArray msg;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
	//Remove user from list and publish
	int old_size = user_ids.size();
	user_ids.erase(std::remove(user_ids.begin(), user_ids.end(), nId), user_ids.end());	
	
	//if user did get removed, then publish
	if((old_size - user_ids.size()) == 1)
	{
		msg.data = user_ids;
		users_pub.publish(msg);
	}
	
	
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().SetSmoothing(smoothing);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		
		//Add user to list and publish
		user_ids.push_back(nId);
		msg.data = user_ids;
		users_pub.publish(msg);
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

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

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
    snprintf(child_frame_no, sizeof(child_frame_no), "person%d_%s", user, child_frame_id.c_str());

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

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[num_users];
    XnUInt16 users_count = num_users;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;

        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");
        
        if(skeleton_profile == XN_SKEL_PROFILE_ALL || skeleton_profile == XN_SKEL_PROFILE_UPPER || skeleton_profile == XN_SKEL_PROFILE_HEAD_HANDS)
        {
			publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "right_shoulder");
			publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "right_elbow");
			publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "right_hand");

			publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "left_shoulder");
			publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "left_elbow");
			publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "left_hand");
		}
		
        if(skeleton_profile == XN_SKEL_PROFILE_LOWER || skeleton_profile == XN_SKEL_PROFILE_ALL)
        {
			publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "right_hip");
			publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "right_knee");
			publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "right_foot");

			publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "left_hip");
			publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "left_knee");
			publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "left_foot");
		}
    }
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
    users_pub = pnh.advertise<std_msgs::UInt16MultiArray>("users", num_users, true);

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
		publishTransforms(frame_id);
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
