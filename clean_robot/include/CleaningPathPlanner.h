/***
 * @brief: cleaning robot path planning
 * @author: Wang
 * @date: 20170702
***/

#ifndef CLEANINGPATHPLANNING_H
#define CLEANINGPATHPLANNING_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace cv;
using namespace std;

constexpr double PI =3.14159;

struct cellIndex
{
    int row;
    int col;
    double theta; //{0, 45,90,135,180,225,270,315}     角度信息   hjr 注
};

/*************************************************
 *
 * 读取栅格地图并根据占据信息获取其对应的空闲（可行走）空间，
 * 按照遍历算法规划行走路线。
 *
 * **********************************************/
class CleaningPathPlanning
{
public:
    //CleaningPathPlanning() = delete;
    CleaningPathPlanning(costmap_2d::Costmap2DROS *costmap2d_ros);

    vector<geometry_msgs::PoseStamped> GetPathInROS(); 
    vector<geometry_msgs::PoseStamped> GetBorderTrackingPathInROS();

    void SetCoveredGrid(double wx,double wy);
    int GetSizeOfCell(){return this->SIZE_OF_CELL;}

    //for visualization
    void PublishCoveragePath();
    void PublishGrid();
private:
    //helper functions.
    bool initializeMats();
    bool initializeCoveredGrid();
    void getCellMatAndFreeSpace(Mat srcImg, Mat &cellMat,vector<cellIndex> &freeSpaceVec);
    void initializeNeuralMat(Mat cellMat, Mat neuralizedMat);
    void writeResult(Mat resultmat,vector<cellIndex> pathVec);
    void writeResult(cv::Mat resultmat,std::vector<cv::Point2i> pathVec);
    void mainPlanningLoop();
    double distance(Point2i pta,Point2i ptb);
    bool findElement(vector<cv::Point2i> pointsVec,cv::Point2i pt, int&index);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    bool cellContainsPoint(cv::Point2i pt,cellIndex cell);

    void GetBorderTrackingPathInCV(vector<cv::Point2i>&resultVec);
    vector<cellIndex> GetPathInCV();


    bool initialized_;
    Mat srcMap_;
    Mat cellMat_;
    Mat neuralizedMat_;
    vector<cellIndex> freeSpaceVec_;
    vector<cellIndex> pathVec_;
    vector<geometry_msgs::PoseStamped> pathVecInROS_;

    double resolution_;
    ros::Publisher plan_pub_;
    ros::Publisher grid_pub_;
    nav_msgs::OccupancyGrid covered_path_grid_;

    //tf::TransformListener &tf_;
    tf::Stamped<tf::Pose> initPose_;

    costmap_2d::Costmap2D* costmap2d_;
    costmap_2d::Costmap2DROS* costmap2d_ros_;

    int SIZE_OF_CELL; //must be odd number.
    int GRID_COVERED_VALUE;
};

#endif // CLEANINGPATHPLANNING_H
