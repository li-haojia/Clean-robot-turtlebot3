#include "CleaningPathPlanner.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");//hjr

    tf::TransformListener tf(ros::Duration(10));

    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);
    //planner_costmap_ros_->pause();

    CleaningPathPlanning clr(&lcr);
    clr.GetPathInROS();
    //clr.GetBorderTrackingPathInROS();   //这句话为啥就给注释掉了呢？？？？？不解。
    ros::Rate r(10);
    while(ros::ok()){       //ros::ok可以通俗地理解为ros还好吗？是否被叫停。
      clr.PublishCoveragePath();
      ros::spinOnce();//这里要这个ros:spin()会不会是鸡肋？？？没有看到回调函数，只有一个tf树的监听者。
      r.sleep();
    }

    ros::shutdown();//关闭节点以及所有与之相关的发布，订阅，调用与服务。
    return 0;
}

