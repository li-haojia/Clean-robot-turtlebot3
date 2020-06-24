
/*This code is used to plan the trajectory of the robot  
*/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace tf;

float x_current;
float y_current;

float normeNextGoal;

class quaternion_ros{
  public :
    float w;
    float x;
    float y;
    float z;

  quaternion_ros();

  void toQuaternion(float pitch, float roll, float yaw);

};

void quaternion_ros::toQuaternion(float pitch, float roll, float yaw)
{
  
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  w = cy * cr * cp + sy * sr * sp;
  x = cy * sr * cp - sy * cr * sp;
  y = cy * cr * sp + sy * sr * cp;
  z = sy * cr * cp - cy * sr * sp;
  cout << "Quaternion value:" << endl;
  cout << " w= " << w << " x= " << x << " y= " << y << " z= " << z << endl;
}

quaternion_ros::quaternion_ros(){
  w = 1;
  x = 0;
  y = 0;
  z = 0;
}




class Path_planned{
public:
  struct Goal{
    float x;
    float y;
    bool visited;
  };

  vector<Goal> Path;

  Path_planned();
  //Path_planned(float x, float y, bool visited);
  void addGoal(float X, float Y, bool visit);
};


Path_planned::Path_planned(){
}

//Path_planned(float x, float y, bool visited)

void Path_planned::addGoal(float X, float Y, bool visit){
  Path_planned::Goal newGoal;
  newGoal.x = X;
  newGoal.y = Y;
  newGoal.visited = visit;
  Path.push_back(newGoal);
}


Path_planned planned_path;

void pose_recu(const nav_msgs::Odometry& poses){ //void pose_recu(const nav_msgs::Odometry &poses){
    x_current = poses.pose.pose.position.x;
    y_current = poses.pose.pose.position.y;
}

int taille_last_path = 0;
bool new_path = false;

void path_recu(const nav_msgs::Path& path){ //void path_recu(const nav_msgs::Path &path){

  if((planned_path.Path.size() == 0) || (path.poses.size() != taille_last_path)){
    planned_path.Path.clear();
    new_path = true;
    for(int i = 0; i < path.poses.size(); i++){
      planned_path.addGoal(path.poses[i].pose.position.x, path.poses[i].pose.position.y, false);

      cout << path.poses[i].pose.position.x << " " <<  path.poses[i].pose.position.y << endl;
    }
    cout << "TAILLE DU PATH: " << path.poses.size() << endl;
    taille_last_path = path.poses.size();
  }
}


// int **count_antonin(char *)

int main(int argc, char *argv[]){
  srand(time(0));
	ros::init(argc, argv, "next_goal");
	ros::NodeHandle next_goal;
	ros::Subscriber sub1 = next_goal.subscribe("/odom",1000,pose_recu);
  ros::Subscriber sub2 = next_goal.subscribe("/path_planning_node/cleaning_plan_nodehandle/cleaning_path",1000,path_recu);

  ros::Publisher pub1 = next_goal.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
  ros::Publisher pub2 = next_goal.advertise<geometry_msgs::PoseStamped>("/path_planned/mowed_cell",1000);

  ros::Rate loop_rate(10);
  
  geometry_msgs::PoseStamped goal_msgs;
  geometry_msgs::PoseStamped mowed_cell; // Message avec les coordonnées de la cellule tondue
  int count = 0;
  double angle;
  bool goal_reached = false;

  if(!next_goal.getParam("/NextGoal/tolerance_goal",normeNextGoal)){
    cout << " rosparam load parameters.yaml" << endl;
    return 0;
  }

  cout << normeNextGoal << endl;

  while (ros::ok()){
    ros::spinOnce();
    if (new_path){
      count = 0;
      new_path = false;
    }
    cout << " count : " << count << endl;
    if(!planned_path.Path.empty()){
      
      
      if(sqrt(pow(x_current - planned_path.Path[count].x,2) + pow(y_current - planned_path.Path[count].y,2))<= normeNextGoal){
        
        // Construction du message de la cellule tondue
        mowed_cell.header.frame_id = "odom";
        mowed_cell.header.stamp = ros::Time::now();
        mowed_cell.pose.position.x = planned_path.Path[count].x;
        mowed_cell.pose.position.y = planned_path.Path[count].y;
        mowed_cell.pose.position.z = 0;
        mowed_cell.pose.orientation.w = 0;
        mowed_cell.pose.orientation.x = 0;
        mowed_cell.pose.orientation.y = 0;
        mowed_cell.pose.orientation.z = 0;
        pub2.publish(mowed_cell);

        count++;
        goal_reached = false;
      }
      if(goal_reached == false){
        goal_msgs.header.frame_id = "odom";
        goal_msgs.header.stamp = ros::Time::now();
        goal_msgs.pose.position.x = planned_path.Path[count].x;
        goal_msgs.pose.position.y = planned_path.Path[count].y;
        goal_msgs.pose.position.z = 0;
        if(count < planned_path.Path.size()){
          angle = atan2(planned_path.Path[count+1].y-planned_path.Path[count].y,planned_path.Path[count+1].x-planned_path.Path[count].x);
        }
        else{
          angle = atan2(planned_path.Path[0].y-planned_path.Path[count].y,planned_path.Path[0].x-planned_path.Path[count].x);
        }
        cout << angle << endl;
        quaternion_ros q;
        q.toQuaternion(0,0,float(angle));
        cout << " QUATERNION DANS LA BOUCLE : " << q.w << " " << q.x << " " << q.y << " " << q.z << endl;
        goal_msgs.pose.orientation.w = q.w;
        goal_msgs.pose.orientation.x = q.x;
        goal_msgs.pose.orientation.y = q.y;
        if (planned_path.Path[count].x < planned_path.Path[count+1].x){
          goal_msgs.pose.orientation.z = 0;
        }
        if (planned_path.Path[count].x > planned_path.Path[count+1].x){
          goal_msgs.pose.orientation.z = 2;
        }

        cout << " NEW GOAL " << endl;
        cout << " x = " << planned_path.Path[count].x << " y = " << planned_path.Path[count].y << endl;
        
        goal_reached = true;   
        pub1.publish(goal_msgs);     
      }
      cout << x_current << " " << y_current << endl;
      //当前
      cout << planned_path.Path[count].x << " " << planned_path.Path[count].y <<endl;
       //目标
      cout << " NORME : " << sqrt((x_current - planned_path.Path[count].x)*(x_current - planned_path.Path[count].x) + (y_current - planned_path.Path[count].y)*(y_current - planned_path.Path[count].y)) << endl;
      // 距离公式
    }


    
    loop_rate.sleep();
  }

  return 0;
}
