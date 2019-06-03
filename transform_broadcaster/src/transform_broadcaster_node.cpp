#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odometryCallback(const nav_msgs::Odometry &data);
void tf_base_link_to_laser();
void tf_base_link_to_base_footprint();
void tf_odom_to_base_link();

bool isCameOdom = false;
// odom - base_link
geometry_msgs::TransformStamped odom_trans;
// base_link - laser
geometry_msgs::TransformStamped base_link_trans;
// base_link - base_footprint
geometry_msgs::TransformStamped base_footprint_trans;

ros::Time current_time;
nav_msgs::Odometry cur_odom;

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle n;
    ros::Subscriber odom_sub = n.subscribe("/odom", 8, odometryCallback);
    tf::TransformBroadcaster odom2base_link_broadcaster;
    tf::TransformBroadcaster base_link2laser_broadcaster;
    tf::TransformBroadcaster base_link2base_footprint_broadcaster;
    current_time = ros::Time::now();
    ros::Rate rate(50);
    while(ros::ok()){

        ROS_INFO("Fine");
        current_time = ros::Time::now();

        //send the transform
        if(isCameOdom){
            tf_odom_to_base_link();
            odom2base_link_broadcaster.sendTransform(odom_trans);
        }


        tf_base_link_to_base_footprint();
        base_link2base_footprint_broadcaster.sendTransform(base_footprint_trans);

        tf_base_link_to_laser();
        base_link2laser_broadcaster.sendTransform(base_link_trans);

        rate.sleep();
        ROS_INFO("very Fine");
        ros::spinOnce();
    }
}
void odometryCallback(const nav_msgs::Odometry &data){
    cur_odom = data;
    isCameOdom = true;
}
void tf_odom_to_base_link(){
    //transform over tf
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = cur_odom.pose.pose.position.x;
    odom_trans.transform.translation.y = cur_odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = cur_odom.pose.pose.orientation;
}

void tf_base_link_to_base_footprint(){
    //transform over tf
    base_footprint_trans.header.stamp = current_time;
    base_footprint_trans.header.frame_id = "base_link";
    base_footprint_trans.child_frame_id = "base_footprint";

    base_footprint_trans.transform.translation.x = 0.0;
    base_footprint_trans.transform.translation.y = 0.0;
    base_footprint_trans.transform.translation.z = 0.0;
    base_footprint_trans.transform.rotation.x = 0.0;
    base_footprint_trans.transform.rotation.y = 0.0;
    base_footprint_trans.transform.rotation.z = 0.0;
    base_footprint_trans.transform.rotation.w = 1.0;

}

void tf_base_link_to_laser(){
    //transform over tf
    base_link_trans.header.stamp = current_time;
    base_link_trans.header.frame_id = "base_footprint";
    base_link_trans.child_frame_id = "laser";

    base_link_trans.transform.translation.x = 0.25;
    base_link_trans.transform.translation.y = 0.0;
    base_link_trans.transform.translation.z = 0.0;
    geometry_msgs::Quaternion tf_quat = tf::createQuaternionMsgFromYaw(0);
    base_link_trans.transform.rotation = tf_quat;
}
