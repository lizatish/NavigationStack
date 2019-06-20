#include "rrt.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


void odometryCallback(const nav_msgs::Odometry data);
void globalMapCallback(const nav_msgs::OccupancyGrid& data);
void goalFromRvizCallback(const geometry_msgs::PoseStamped& data);
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data);


// Сообщение с путем
static nav_msgs::Path current_path;
static nav_msgs::Path previous_path;

static float ROBOT_HEIGHT = 0.7;
static float ROBOT_WIDTH = 0.5;
static const float CURVATURE = 0.2;

//Текущее положение платформы
static geometry_msgs::Pose currentPosition;
static nav_msgs::OccupancyGrid globalMap;

static bool isCameGlobalMap = false;
static bool isCameOdom = false;
static bool isGoalCame = false;

static geometry_msgs::Pose goal;
static geometry_msgs::PoseStamped goal_rviz;

int main(int argc, char **argv){
    ros::init(argc, argv, "kuka_path_searcher_node");

    // Создание публикатора пути
    ros::NodeHandle l;
    ros::Publisher path_for_rviz_pub = l.advertise<nav_msgs::Path>("/path_rrt", 8);
    ros::Publisher path_for_control_pub = l.advertise<nav_msgs::Path>("/target_path", 8);
    ros::Subscriber global_map_sub = l.subscribe("/map", 8, globalMapCallback);
    //    ros::Subscriber odom_sub = l.subscribe("/odom", 8, odometryCallback);
    ros::Publisher goal_pub = l.advertise<geometry_msgs::PoseStamped>("/rrt_goal", 8);

    ros::Subscriber goal_from_rviz_sub = l.subscribe("/move_base_simple/goal", 8, goalFromRvizCallback);
    ros::Subscriber slam_current_pose_sub = l.subscribe("/slam_out_pose", 8, slamOutPoseCallback);

    //    goal.position.x = 3;
    //    goal.position.y = 5;
    //    goal.orientation = tf::createQuaternionMsgFromYaw(1.5);

    ros::Rate rate(100);
    bool isAllowProcess = true;
    while(ros::ok() && isAllowProcess){
        //        cout << isCameOdom << " " << isCameGlobalMap << " " <<  isGoalCame << endl;
        if(isCameOdom && isCameGlobalMap && isGoalCame){

            previous_path = current_path;
            // Запуск планировщика
            RRT* rrt = new RRT();
            current_path = rrt->Planning(currentPosition, goal, globalMap, CURVATURE, ROBOT_HEIGHT, ROBOT_WIDTH);
            delete rrt;

            if(current_path.poses.size()){
                if(current_path.poses.size() < previous_path.poses.size() - 20 || !previous_path.poses.size()){
                    cout << "Path is " << current_path.poses.size() << endl;
                    path_for_control_pub.publish(current_path);
                }
                else{
                    cout << "Path is not found" << endl;
                }
            }
        }
        path_for_rviz_pub.publish(current_path);
        goal_pub.publish(goal_rviz);
        ros::spinOnce();
        rate.sleep();
    }
}
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data){
    currentPosition = data.pose;
    isCameOdom = true;

}

void odometryCallback(const nav_msgs::Odometry data){
    currentPosition = data.pose.pose;
    isCameOdom = true;
}
void goalFromRvizCallback(const geometry_msgs::PoseStamped& data){
    goal = data.pose;

    goal_rviz.pose = goal;
    goal_rviz.header.frame_id = "map";
    goal_rviz.header.stamp = ros::Time::now();
    isGoalCame = true;
}

void globalMapCallback(const nav_msgs::OccupancyGrid& data){
    globalMap.info = data.info;
    globalMap.info.origin.position.x = -(data.info.height*data.info.resolution)/2;
    globalMap.info.origin.position.y = -(data.info.width*data.info.resolution)/2;
    globalMap.data = data.data;

    isCameGlobalMap = true;
}
