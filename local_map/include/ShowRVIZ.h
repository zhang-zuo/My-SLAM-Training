//
// Created by zhangzuo on 22-9-19.
//

#ifndef SRC_SHOWRVIZ_H
#define SRC_SHOWRVIZ_H

#include "LocalMap.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include "Eigen/Geometry"

void RegisterPub(ros::NodeHandle &n);
void ShowRVIZ(vector<KeyFrame*> &mDataBaseKF, map<unsigned long, MapPoint*> &mDataBaseMP, KeyFrame* mCurrKF);
void Mat2EigenRt(cv::Mat &pose, Eigen::Quaterniond &R, Eigen::Vector3d &t, Eigen::Vector3d &O);

class MarkerVisualization {
public:
    std::string m_marker_ns;

    MarkerVisualization(float r, float g, float b, float a);

    void setScale(double s);
    void setLineWidth(double width);

    void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    void add_line(const Eigen::Vector3d &fp, const geometry_msgs::Point &mp);
    void reset();

    void publish_by(ros::Publisher& pub, const std_msgs::Header& header);

private:
    std::vector<visualization_msgs::Marker> m_markers;
    std_msgs::ColorRGBA m_image_boundary_color;
    std_msgs::ColorRGBA m_optical_center_connector_color;
    double m_scale;
    double m_line_width;

    static const Eigen::Vector3d imlt;
    static const Eigen::Vector3d imlb;
    static const Eigen::Vector3d imrt;
    static const Eigen::Vector3d imrb;
    static const Eigen::Vector3d oc  ;
    static const Eigen::Vector3d lt0 ;
    static const Eigen::Vector3d lt1 ;
    static const Eigen::Vector3d lt2 ;
};



#endif //SRC_SHOWRVIZ_H
