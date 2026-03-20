// #include <ros/ros.h>
// #include <iostream>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <gazebo_msgs/ModelStates.h>
// #include <rosgraph_msgs/Clock.h>
// #include <tf2_ros/transform_broadcaster.h>

// using namespace std;
// using namespace ros;

// class UGVPoseFromGZ{
// private:
//     // Publishers
//     tf2_ros::TransformBroadcaster tf_pub;
//     Publisher UGV_pose_publisher,
//               UGV_pose_amcl_publisher;

//     // Subscribers
//     Subscriber gz_subscriber;

//     geometry_msgs::PoseStamped pose_UGV_gz;
//     geometry_msgs::PoseWithCovarianceStamped pose_UGV_gz_amcl;
//     geometry_msgs::TransformStamped tf_map_to_UGV;

//     bool UGV_received;
//     int index_name_UGV;
//     std::string UGV_name;

// public:
//     // Declaring constructor and destructor
//     UGVPoseFromGZ(){};
//     ~UGVPoseFromGZ();

//     // Functions declaration
//     void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_model_msg);
//     void init_ros_comms(ros::NodeHandle *nh);
//     void publish_UGV_pose();

//     // Variables
//     double node_rate;
// };

// // Defining destructor.
// UGVPoseFromGZ::~UGVPoseFromGZ(){
//     ros::shutdown();
//     exit(0);
// }

// void UGVPoseFromGZ::init_ros_comms(ros::NodeHandle *nh)
// {
//     nh->param<std::string>(ros::this_node::getName() + "/ugv_name", UGV_name, "jackal");     // parameter name, string object reference, default value
//     nh->param<double>(ros::this_node::getName() + "/node_rate", node_rate, 50);           // parameter name, string object reference, default value

//     // Subs
//     gz_subscriber = nh->subscribe("gazebo/model_states", 1, &UGVPoseFromGZ::gazeboCallback, this);
    
//     // Pubs
//     UGV_pose_publisher = nh->advertise<geometry_msgs::PoseStamped>("ground_robot_localization/pose", 1);
//     UGV_pose_amcl_publisher = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);

//     // Init the variables
//     UGV_received = false;
//     index_name_UGV = -1;
// }

// // Gazebo model states callback
// void UGVPoseFromGZ::gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_model_msg){
//     tf_map_to_UGV.header.frame_id = "map";
//     tf_map_to_UGV.child_frame_id = "base_link";

//     int i = 0;
//     if(!UGV_received){ // For the first time find required model name (index)
//         while(!UGV_received){
//             if(!gazebo_model_msg->name[i].compare(UGV_name)){
//                 index_name_UGV = i;
//                 UGV_received = true;
//             }
//             i++;
//         }
//     }

//     pose_UGV_gz.pose = gazebo_model_msg->pose[index_name_UGV];
//     pose_UGV_gz_amcl.pose.pose = gazebo_model_msg->pose[index_name_UGV];
//     tf_map_to_UGV.transform.translation.x = gazebo_model_msg->pose[index_name_UGV].position.x;
//     tf_map_to_UGV.transform.translation.y = gazebo_model_msg->pose[index_name_UGV].position.y;
//     tf_map_to_UGV.transform.translation.z = gazebo_model_msg->pose[index_name_UGV].position.z;
//     tf_map_to_UGV.transform.rotation.x = gazebo_model_msg->pose[index_name_UGV].orientation.x;
//     tf_map_to_UGV.transform.rotation.y = gazebo_model_msg->pose[index_name_UGV].orientation.y;
//     tf_map_to_UGV.transform.rotation.z = gazebo_model_msg->pose[index_name_UGV].orientation.z;
//     tf_map_to_UGV.transform.rotation.w = gazebo_model_msg->pose[index_name_UGV].orientation.w;
// }
    
// // Publish UGV pose    
// void UGVPoseFromGZ::publish_UGV_pose(){
//     if(UGV_received){     // Publish UGV pose in world frame (from Gazebo)
//         pose_UGV_gz.header.stamp = ros::Time::now();
//         pose_UGV_gz_amcl.header.stamp = ros::Time::now();
//         tf_map_to_UGV.header.stamp = ros::Time::now();
//         UGV_pose_publisher.publish(pose_UGV_gz);
//         UGV_pose_amcl_publisher.publish(pose_UGV_gz_amcl);
//         tf_pub.sendTransform(tf_map_to_UGV);
//     }   
// }

// int main(int argc, char** argv){
//     ros::init(argc, argv, "ugv_pose_from_gz");
//     ros::NodeHandle nh;
//     cout << "ugv_pose_from_gz is running..." << endl;

//     UGVPoseFromGZ UPFG;
//     UPFG.init_ros_comms(&nh);
//     ros::Rate rate(UPFG.node_rate);
        
//     while(ros::ok()){
//         UPFG.publish_UGV_pose();
//         rate.sleep();
//         ros::spinOnce();
//     }   
// }


#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std;
using namespace ros;

class UGVPoseFromGZ
{
private:
    // Publishers
    tf2_ros::TransformBroadcaster tf_pub;
    Publisher UGV_pose_publisher;
    Publisher UGV_pose_amcl_publisher;

    // Subscribers
    Subscriber gz_subscriber;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    geometry_msgs::PoseStamped pose_UGV_gz;
    geometry_msgs::PoseWithCovarianceStamped pose_UGV_gz_amcl;
    geometry_msgs::TransformStamped tf_map_to_odom;

    bool UGV_received;
    bool index_found;
    bool init_average_started;
    bool init_average_done;

    int index_name_UGV;
    std::string UGV_name;
    std::string odom_frame;
    std::string base_frame;
    std::string map_frame;

    ros::Time init_average_start_time;

    double init_average_duration;
    double sum_x;
    double sum_y;
    double sum_z;
    double sum_yaw_sin;
    double sum_yaw_cos;
    int sample_count;

    geometry_msgs::Pose mean_initial_pose;
    geometry_msgs::Pose latest_gazebo_pose;
    geometry_msgs::Pose offset_pose_map_to_base;

private:
    bool isFiniteDouble(double value)
    {
        return std::isfinite(value);
    }

    bool isValidQuaternionMsg(const geometry_msgs::Quaternion &q_msg)
    {
        if (!isFiniteDouble(q_msg.x) || !isFiniteDouble(q_msg.y) ||
            !isFiniteDouble(q_msg.z) || !isFiniteDouble(q_msg.w))
        {
            return false;
        }

        double norm_sq = q_msg.x * q_msg.x +
                         q_msg.y * q_msg.y +
                         q_msg.z * q_msg.z +
                         q_msg.w * q_msg.w;

        return std::isfinite(norm_sq) && norm_sq > 1e-12;
    }

    bool isValidPoseMsg(const geometry_msgs::Pose &pose_msg)
    {
        return isFiniteDouble(pose_msg.position.x) &&
               isFiniteDouble(pose_msg.position.y) &&
               isFiniteDouble(pose_msg.position.z) &&
               isValidQuaternionMsg(pose_msg.orientation);
    }

    bool isValidTransformMsg(const geometry_msgs::TransformStamped &tf_msg)
    {
        return isFiniteDouble(tf_msg.transform.translation.x) &&
               isFiniteDouble(tf_msg.transform.translation.y) &&
               isFiniteDouble(tf_msg.transform.translation.z) &&
               isValidQuaternionMsg(tf_msg.transform.rotation);
    }

    bool findUGVIndex(const gazebo_msgs::ModelStates::ConstPtr &gazebo_model_msg)
    {
        for (size_t i = 0; i < gazebo_model_msg->name.size(); ++i)
        {
            if (gazebo_model_msg->name[i] == UGV_name)
            {
                index_name_UGV = static_cast<int>(i);
                index_found = true;
                UGV_received = true;
                return true;
            }
        }
        return false;
    }

    double getYawFromQuaternion(const geometry_msgs::Quaternion &q_msg)
    {
        if (!isValidQuaternionMsg(q_msg))
        {
            ROS_WARN_THROTTLE(1.0, "Invalid quaternion received while extracting yaw. Using 0.0.");
            return 0.0;
        }

        tf2::Quaternion q;
        tf2::fromMsg(q_msg, q);
        q.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    geometry_msgs::Quaternion createQuaternionFromYaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        q.normalize();
        return tf2::toMsg(q);
    }

    tf2::Transform poseMsgToTf(const geometry_msgs::Pose &pose_msg)
    {
        tf2::Transform tf_pose;
        tf2::Quaternion q;

        if (!isValidPoseMsg(pose_msg))
        {
            ROS_WARN_THROTTLE(1.0, "Invalid pose received in poseMsgToTf(). Returning identity transform.");
            tf_pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
            tf_pose.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
            return tf_pose;
        }

        tf2::fromMsg(pose_msg.orientation, q);
        q.normalize();

        tf_pose.setOrigin(tf2::Vector3(pose_msg.position.x,
                                       pose_msg.position.y,
                                       pose_msg.position.z));
        tf_pose.setRotation(q);
        return tf_pose;
    }

    geometry_msgs::Pose tfToPoseMsg(const tf2::Transform &tf_pose)
    {
        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = tf_pose.getOrigin().x();
        pose_msg.position.y = tf_pose.getOrigin().y();
        pose_msg.position.z = tf_pose.getOrigin().z();

        tf2::Quaternion q = tf_pose.getRotation();
        if (!std::isfinite(q.x()) || !std::isfinite(q.y()) ||
            !std::isfinite(q.z()) || !std::isfinite(q.w()) ||
            q.length2() < 1e-12)
        {
            ROS_WARN_THROTTLE(1.0, "Invalid tf quaternion in tfToPoseMsg(). Replacing with identity.");
            q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
        }
        else
        {
            q.normalize();
        }

        pose_msg.orientation = tf2::toMsg(q);
        return pose_msg;
    }

    void accumulateInitialPoseSample(const geometry_msgs::Pose &pose_msg)
    {
        if (!isValidPoseMsg(pose_msg))
        {
            ROS_WARN_THROTTLE(1.0, "Invalid initial Gazebo pose sample. Skipping.");
            return;
        }

        if (!init_average_started)
        {
            init_average_start_time = ros::Time::now();
            init_average_started = true;
            ROS_INFO("Started averaging initial Gazebo pose for %.2f seconds.", init_average_duration);
        }

        sum_x += pose_msg.position.x;
        sum_y += pose_msg.position.y;
        sum_z += pose_msg.position.z;

        double yaw = getYawFromQuaternion(pose_msg.orientation);
        sum_yaw_sin += std::sin(yaw);
        sum_yaw_cos += std::cos(yaw);
        sample_count++;

        ros::Duration elapsed = ros::Time::now() - init_average_start_time;
        if (!init_average_done && elapsed.toSec() >= init_average_duration && sample_count > 0)
        {
            mean_initial_pose.position.x = sum_x / static_cast<double>(sample_count);
            mean_initial_pose.position.y = sum_y / static_cast<double>(sample_count);
            mean_initial_pose.position.z = sum_z / static_cast<double>(sample_count);

            double mean_yaw = std::atan2(sum_yaw_sin / static_cast<double>(sample_count),
                                         sum_yaw_cos / static_cast<double>(sample_count));
            mean_initial_pose.orientation = createQuaternionFromYaw(mean_yaw);

            if (!isValidPoseMsg(mean_initial_pose))
            {
                ROS_ERROR("Computed mean initial pose is invalid. Resetting to identity.");
                mean_initial_pose.position.x = 0.0;
                mean_initial_pose.position.y = 0.0;
                mean_initial_pose.position.z = 0.0;
                mean_initial_pose.orientation.x = 0.0;
                mean_initial_pose.orientation.y = 0.0;
                mean_initial_pose.orientation.z = 0.0;
                mean_initial_pose.orientation.w = 1.0;
            }

            init_average_done = true;

            ROS_INFO("Initial pose averaging completed with %d samples.", sample_count);
            ROS_INFO("Mean initial pose: x=%.4f y=%.4f z=%.4f yaw=%.4f rad",
                     mean_initial_pose.position.x,
                     mean_initial_pose.position.y,
                     mean_initial_pose.position.z,
                     mean_yaw);
        }
    }

    geometry_msgs::Pose computeOffsetPose(const geometry_msgs::Pose &current_gazebo_pose)
    {
        if (!isValidPoseMsg(mean_initial_pose) || !isValidPoseMsg(current_gazebo_pose))
        {
            ROS_WARN_THROTTLE(1.0, "Invalid pose in computeOffsetPose(). Returning identity pose.");
            geometry_msgs::Pose identity_pose;
            identity_pose.position.x = 0.0;
            identity_pose.position.y = 0.0;
            identity_pose.position.z = 0.0;
            identity_pose.orientation.x = 0.0;
            identity_pose.orientation.y = 0.0;
            identity_pose.orientation.z = 0.0;
            identity_pose.orientation.w = 1.0;
            return identity_pose;
        }

        tf2::Transform T_init = poseMsgToTf(mean_initial_pose);
        tf2::Transform T_gz = poseMsgToTf(current_gazebo_pose);
        tf2::Transform T_offset = T_init.inverse() * T_gz;

        return tfToPoseMsg(T_offset);
    }

public:
    UGVPoseFromGZ()
        : tf_listener(tf_buffer)
    {
    }

    ~UGVPoseFromGZ();

    void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr &gazebo_model_msg);
    void init_ros_comms(ros::NodeHandle *nh);
    void publish_UGV_pose();

    double node_rate;
};

UGVPoseFromGZ::~UGVPoseFromGZ()
{
    ros::shutdown();
    exit(0);
}

void UGVPoseFromGZ::init_ros_comms(ros::NodeHandle *nh)
{
    nh->param<std::string>(ros::this_node::getName() + "/ugv_name", UGV_name, "jackal");
    nh->param<double>(ros::this_node::getName() + "/node_rate", node_rate, 50.0);
    nh->param<double>(ros::this_node::getName() + "/init_average_duration", init_average_duration, 2.0);
    nh->param<std::string>(ros::this_node::getName() + "/map_frame", map_frame, "map");
    nh->param<std::string>(ros::this_node::getName() + "/odom_frame", odom_frame, "odom");
    nh->param<std::string>(ros::this_node::getName() + "/base_frame", base_frame, "base_link");

    gz_subscriber = nh->subscribe("gazebo/model_states", 1, &UGVPoseFromGZ::gazeboCallback, this);

    UGV_pose_publisher = nh->advertise<geometry_msgs::PoseStamped>("ground_robot_localization/pose", 1);
    UGV_pose_amcl_publisher = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);

    UGV_received = false;
    index_found = false;
    init_average_started = false;
    init_average_done = false;
    index_name_UGV = -1;

    sum_x = 0.0;
    sum_y = 0.0;
    sum_z = 0.0;
    sum_yaw_sin = 0.0;
    sum_yaw_cos = 0.0;
    sample_count = 0;

    mean_initial_pose.position.x = 0.0;
    mean_initial_pose.position.y = 0.0;
    mean_initial_pose.position.z = 0.0;
    mean_initial_pose.orientation.x = 0.0;
    mean_initial_pose.orientation.y = 0.0;
    mean_initial_pose.orientation.z = 0.0;
    mean_initial_pose.orientation.w = 1.0;

    offset_pose_map_to_base = mean_initial_pose;
}

void UGVPoseFromGZ::gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr &gazebo_model_msg)
{
    if (!index_found)
    {
        if (!findUGVIndex(gazebo_model_msg))
        {
            ROS_WARN_THROTTLE(2.0, "Could not find model name '%s' in /gazebo/model_states.", UGV_name.c_str());
            return;
        }
    }

    if (index_name_UGV < 0 || index_name_UGV >= static_cast<int>(gazebo_model_msg->pose.size()))
    {
        ROS_WARN_THROTTLE(1.0, "UGV index is out of range for gazebo pose array.");
        return;
    }

    latest_gazebo_pose = gazebo_model_msg->pose[index_name_UGV];

    if (!isValidPoseMsg(latest_gazebo_pose))
    {
        ROS_WARN_THROTTLE(1.0, "Received invalid Gazebo pose. Skipping.");
        return;
    }

    UGV_received = true;

    if (!init_average_done)
    {
        accumulateInitialPoseSample(latest_gazebo_pose);
        return;
    }

    offset_pose_map_to_base = computeOffsetPose(latest_gazebo_pose);

    if (!isValidPoseMsg(offset_pose_map_to_base))
    {
        ROS_WARN_THROTTLE(1.0, "Computed offset pose is invalid. Skipping publish update.");
        return;
    }

    pose_UGV_gz.pose = offset_pose_map_to_base;
    pose_UGV_gz.header.frame_id = map_frame;

    pose_UGV_gz_amcl.pose.pose = offset_pose_map_to_base;
    pose_UGV_gz_amcl.header.frame_id = map_frame;
}

void UGVPoseFromGZ::publish_UGV_pose()
{
    if (!UGV_received || !init_average_done)
    {
        return;
    }

    if (!isValidPoseMsg(offset_pose_map_to_base))
    {
        ROS_WARN_THROTTLE(1.0, "Offset pose is invalid before publishing. Skipping.");
        return;
    }

    ros::Time stamp_now = ros::Time::now();

    pose_UGV_gz.header.stamp = stamp_now;
    pose_UGV_gz_amcl.header.stamp = stamp_now;

    UGV_pose_publisher.publish(pose_UGV_gz);
    UGV_pose_amcl_publisher.publish(pose_UGV_gz_amcl);

    try
    {
        geometry_msgs::TransformStamped tf_odom_to_base_msg =
            tf_buffer.lookupTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(0.05));

        if (!isValidTransformMsg(tf_odom_to_base_msg))
        {
            ROS_WARN_THROTTLE(1.0, "Received invalid %s -> %s transform. Skipping map -> odom publish.",
                              odom_frame.c_str(), base_frame.c_str());
            return;
        }

        tf2::Transform T_map_to_base = poseMsgToTf(offset_pose_map_to_base);

        tf2::Transform T_odom_to_base;
        tf2::fromMsg(tf_odom_to_base_msg.transform, T_odom_to_base);

        tf2::Quaternion q_odom_to_base = T_odom_to_base.getRotation();
        if (!std::isfinite(q_odom_to_base.x()) || !std::isfinite(q_odom_to_base.y()) ||
            !std::isfinite(q_odom_to_base.z()) || !std::isfinite(q_odom_to_base.w()) ||
            q_odom_to_base.length2() < 1e-12)
        {
            ROS_WARN_THROTTLE(1.0, "Invalid odom -> base quaternion. Skipping map -> odom publish.");
            return;
        }
        q_odom_to_base.normalize();
        T_odom_to_base.setRotation(q_odom_to_base);

        tf2::Transform T_map_to_odom = T_map_to_base * T_odom_to_base.inverse();

        tf2::Quaternion q_map_to_odom = T_map_to_odom.getRotation();
        if (!std::isfinite(q_map_to_odom.x()) || !std::isfinite(q_map_to_odom.y()) ||
            !std::isfinite(q_map_to_odom.z()) || !std::isfinite(q_map_to_odom.w()) ||
            q_map_to_odom.length2() < 1e-12)
        {
            ROS_WARN_THROTTLE(1.0, "Computed map -> odom quaternion is invalid. Skipping TF publish.");
            return;
        }
        q_map_to_odom.normalize();
        T_map_to_odom.setRotation(q_map_to_odom);

        tf_map_to_odom.header.stamp = stamp_now;
        tf_map_to_odom.header.frame_id = map_frame;
        tf_map_to_odom.child_frame_id = odom_frame;
        tf_map_to_odom.transform = tf2::toMsg(T_map_to_odom);

        if (!isValidTransformMsg(tf_map_to_odom))
        {
            ROS_WARN_THROTTLE(1.0, "Final map -> odom transform contains invalid values. Skipping TF publish.");
            return;
        }

        tf_pub.sendTransform(tf_map_to_odom);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(2.0, "Failed to lookup %s -> %s transform: %s",
                          odom_frame.c_str(), base_frame.c_str(), ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_pose_from_gz");
    ros::NodeHandle nh;
    cout << "ugv_pose_from_gz is running..." << endl;

    UGVPoseFromGZ UPFG;
    UPFG.init_ros_comms(&nh);
    ros::Rate rate(UPFG.node_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        UPFG.publish_UGV_pose();
        rate.sleep();
    }

    return 0;
}