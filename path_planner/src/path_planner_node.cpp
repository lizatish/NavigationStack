#include "rrt.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
//#include <gmapping/grid/map.h>


void odometryCallback(const nav_msgs::Odometry data);
void globalMapCallback(const nav_msgs::OccupancyGrid& data);
void goalFromRvizCallback(const geometry_msgs::PoseStamped& data);
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data);


// Сообщение с путем
static nav_msgs::Path current_path;
static nav_msgs::Path previous_path;

static float ROBOT_HEIGHT = 0.7;
static float ROBOT_WIDTH = 0.5;
static const float CURVATURE = 0.3;

//Текущее положение платформы
static geometry_msgs::Pose currentPosition;
static nav_msgs::OccupancyGrid globalMap;

static bool isCameGlobalMap = false;
static bool isCameOdom = false;
geometry_msgs::Pose goal;
geometry_msgs::PoseStamped goal_rviz;
bool isGoalCame = false;

int main(int argc, char **argv){
    ros::init(argc, argv, "kuka_path_searcher_node");

    // Создание публикатора пути
    ros::NodeHandle l;
    ros::Publisher path_for_rviz_pub = l.advertise<nav_msgs::Path>("/path_rrt", 8);
    ros::Publisher path_for_control_pub = l.advertise<nav_msgs::Path>("/target_path", 8);
    ros::Subscriber global_map_sub = l.subscribe("/map", 8, globalMapCallback);
    //    ros::Subscriber odom_sub = l.subscribe("/odom", 8, odometryCallback);
    ros::Publisher global_map_pub = l.advertise<nav_msgs::OccupancyGrid>("/g_map", 8);
    ros::Publisher goal_pub = l.advertise<geometry_msgs::PoseStamped>("/rrt_goal", 8);

    ros::Subscriber goal_from_rviz_sub = l.subscribe("/move_base_simple/goal", 8, goalFromRvizCallback);
    ros::Subscriber slam_current_pose_sub = l.subscribe("/slam_out_pose", 8, slamOutPoseCallback);

    //    goal.position.x = 3;
    //    goal.position.y = 5;
    //    goal.orientation = tf::createQuaternionMsgFromYaw(1.5);

    //    goal_rviz.pose = goal;
    //    goal_rviz.header.frame_id = "map";
    //    goal_rviz.header.stamp = ros::Time::now();



    ros::Rate rate(100);
    bool isAllowProcess = true;
    while(ros::ok() && isAllowProcess){
//        cout << currentPosition.position.x << " " <<
//                currentPosition.position.y << " " <<
//                goal.position.x << " " <<
//                goal.position.y << endl;
        //cout << isCameOdom << " " << isCameGlobalMap << " " <<  isGoalCame << endl;
        if(isCameOdom && isCameGlobalMap && isGoalCame){
            //      isCameOdom = false;
            //      isCameGlobalMap = false;

            previous_path = current_path;
            // Запуск планировщика
            RRT* rrt = new RRT();
            current_path = rrt->Planning(currentPosition, goal, globalMap, CURVATURE, ROBOT_HEIGHT, ROBOT_WIDTH);
            delete rrt;

            if(current_path.poses.size()){
                if(current_path.poses.size() < previous_path.poses.size() - 10|| !previous_path.poses.size()){
                    cout << "Path is " << current_path.poses.size() << endl;
                    path_for_control_pub.publish(current_path);
                }
                else{
                    cout << "Path is not found" << endl;
                }
            }
        }
        path_for_rviz_pub.publish(current_path);
        global_map_pub.publish(globalMap);
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
    //    // Получение угла поворота
    //    tf::Pose pose;
    //    tf::poseMsgToTF(data.pose.pose, pose);
    //    double yawAngle = tf::getYaw(pose.getRotation());

    // Начальные координаты
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
    //    cout << globalMap.info.origin.position.x << " " << globalMap.info.origin.position.y << " " << mapResolution << endl;
    ///// ттут чтото не так
    globalMap.info.resolution = data.info.resolution;
    globalMap.info.height = data.info.height;
    globalMap.info.width = data.info.width;
    globalMap.info.origin.position.x = data.info.origin.position.x;
     globalMap.info.origin.position.y = data.info.origin.position.y;
//cout << globalMap.info.origin<< endl;
    globalMap.data = data.data;

    for(int i = 0; i < globalMap.info.width; i++)
        for(int j = 0; j < globalMap.info.height; j++){

            if((int)globalMap.data[globalMap.info.width * j + i] == -1) {
                globalMap.data[globalMap.info.width * j + i] = 50;
            }
            if((int)globalMap.data[globalMap.info.width * j + i] > 75) {
                globalMap.data[globalMap.info.width * j + i] = 100;
            }
            if((int)globalMap.data[globalMap.info.width * j + i] < 20) {
                globalMap.data[globalMap.info.width * j + i] = 0;
            }
        }
    isCameGlobalMap = true;
}
