#include "rrt.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


void odometryCallback(const nav_msgs::Odometry data);
void globalMapCallback(const nav_msgs::OccupancyGrid& data);
void goalFromRvizCallback(const geometry_msgs::PoseStamped& data);
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data);
bool checkObstaclesOnPath();
nav_msgs::Path formPath(geometry_msgs::Pose s, geometry_msgs::Pose f);


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
static bool isObstaclesOnPath = true;

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
        if(isCameOdom && isCameGlobalMap && isGoalCame){

            if(isObstaclesOnPath){
                ros::Time start_time = ros::Time::now();
                cout << "Start path find... ";
                current_path = formPath(currentPosition, goal);
                cout << ros::Time::now().toSec() - start_time.toSec() << endl;
            }
            checkObstaclesOnPath();

        }
        path_for_control_pub.publish(current_path);

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
    isGoalCame = true;

    goal_rviz.pose = goal;
    goal_rviz.header.frame_id = "map";
    goal_rviz.header.stamp = ros::Time::now();
}

void globalMapCallback(const nav_msgs::OccupancyGrid& data){
    globalMap = data;
    isCameGlobalMap = true;
}

bool checkObstaclesOnPath(){
    uint map_width = globalMap.info.width;
    uint map_height = globalMap.info.height;
    float mapResolution = globalMap.info.resolution;
    int start_size_x = -ROBOT_HEIGHT/(2*mapResolution);
    int start_size_y = -ROBOT_WIDTH/(2*mapResolution);
    int finish_size_x = -start_size_x;
    int finish_size_y = -start_size_y;

    if (!current_path.poses.size()){
        isObstaclesOnPath = true;
        return true;
    }

    for(int k = 0; k < current_path.poses.size(); k++){

        int x = int(current_path.poses.at(k).pose.position.x/mapResolution) + map_width/2;
        int y = int(current_path.poses.at(k).pose.position.y/mapResolution) + map_height/2;
        float robot_yaw = tf::getYaw(current_path.poses.at(k).pose.orientation);
        float sin_yaw = sin(robot_yaw);
        float cos_yaw = cos(robot_yaw);


        if(x + start_size_x >= 0 && x + finish_size_x < map_width
                && y + start_size_y >= 0 && y + finish_size_y < map_height)
            for(int i = start_size_x; i <= finish_size_x; i++){
                for(int j = start_size_y; j <= finish_size_y; j++){

                    // Составляющая поворота
                    int x_robot_size = x + i * cos_yaw + j * sin_yaw;
                    int y_robot_size = y - i * sin_yaw + j * cos_yaw;

                    if(int(globalMap.data[map_width * y_robot_size + x_robot_size]) > 75){
                        isObstaclesOnPath = true;
                        return true;
                    }
                }

            }
    }
    isObstaclesOnPath = false;
    return false;
}

nav_msgs::Path formPath(geometry_msgs::Pose s, geometry_msgs::Pose f){

    vector<nav_msgs::Path> path_various;
    nav_msgs::Path path;
    int counter = 0;
    while(counter <= 5){
        RRT* rrt = new RRT();
        path = rrt->Planning(s, f, globalMap, CURVATURE, ROBOT_HEIGHT, ROBOT_WIDTH);
        if(path.poses.size() != 0)
            path_various.push_back(path);
        counter++;
        delete rrt;
    }
    cout << path_various.size() << endl;

    nav_msgs::Path shortest_path;
    if(path_various.size() != 0){
        isObstaclesOnPath = false;
        shortest_path = path_various.at(0);
        for(int i = 1; i < path_various.size() - 1; i++){
            if(path_various.at(i).poses.size() < path_various.at(i+1).poses.size())
                shortest_path = path_various.at(i);
        }
    }
    return shortest_path;
}
