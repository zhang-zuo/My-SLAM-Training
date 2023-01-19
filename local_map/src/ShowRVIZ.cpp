//
// Created by zhangzuo on 22-9-19.
//
//整体显示的代码都是参考VINS中
#include "ShowRVIZ.h"

ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_point_cloud;
ros::Publisher pub_curr_mappoint_visual;
ros::Publisher pub_path;
nav_msgs::Path path;


MarkerVisualization cameraposevisual(0, 1, 0, 1);
MarkerVisualization mappointvisual(0, 0, 1, 1);


const Eigen::Vector3d MarkerVisualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d MarkerVisualization::imrt = Eigen::Vector3d(1.0, -0.5, 1.0);
const Eigen::Vector3d MarkerVisualization::imlb = Eigen::Vector3d(-1.0, 0.5, 1.0);
const Eigen::Vector3d MarkerVisualization::imrb = Eigen::Vector3d(1.0, 0.5, 1.0);
const Eigen::Vector3d MarkerVisualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d MarkerVisualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d MarkerVisualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d MarkerVisualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);


void RegisterPub(ros::NodeHandle &n)
{
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    cameraposevisual.setScale(0.3);
    cameraposevisual.setLineWidth(0.05);

    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);

    pub_curr_mappoint_visual = n.advertise<visualization_msgs::MarkerArray>("curr_mappoint_visual", 1000);
    mappointvisual.setScale(0.1);
    mappointvisual.setLineWidth(0.01);

    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
}


void ShowRVIZ(vector<KeyFrame*> &mDataBaseKF, map<unsigned long, MapPoint*> &mDataBaseMP, KeyFrame* mCurrKF)
{
    //显示当前关键帧的位姿
    Eigen::Quaterniond mCurr_R;
    Eigen::Vector3d mCurr_O;
    Eigen::Vector3d mCurr_t;

    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = (ros::Time)mCurrKF->mTimeStamp;

    cv::Mat KeyFramePose = mCurrKF->GetPose();
    Mat2EigenRt(KeyFramePose,mCurr_R,mCurr_t,mCurr_O);

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = mCurr_O.x();
    odometry.pose.pose.position.y = mCurr_O.y();
    odometry.pose.pose.position.z = mCurr_O.z();
    odometry.pose.pose.orientation.x = mCurr_R.x();
    odometry.pose.pose.orientation.y = mCurr_R.y();
    odometry.pose.pose.orientation.z = mCurr_R.z();
    odometry.pose.pose.orientation.w = mCurr_R.w();

    cameraposevisual.reset();
    cameraposevisual.add_pose(mCurr_O, mCurr_R);
    cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);


    //显示所有地图点的位置
    //这里的显示点云导致localmap出现线程崩溃
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = header;
    for(auto it = mDataBaseMP.begin(); it != mDataBaseMP.end(); it++)
    {
        MapPoint* mapPoint = it->second;
        if(it->first != mapPoint->mid)
            continue;

        cv::Mat mp3d = mapPoint->GetWorldPos();
        geometry_msgs::Point32 p;
        p.x = mp3d.at<float>(0);
        p.y = mp3d.at<float>(1);
        p.z = mp3d.at<float>(2);
        point_cloud.points.push_back(p);
//        ROS_WARN_STREAM("mp3d: " << mp3d.reshape(0,1));
//        ROS_WARN_STREAM("x: " << p.x << "\ty: " << p.y << "\tz: " << p.z);
    }
    pub_point_cloud.publish(point_cloud);


    //显示当前关键帧与当前地图点之间的连线
//    mappointvisual.reset();
//    auto mCurrMapPoints = mCurrKF->GetmvpMapPoint();
//    for(auto it = mCurrMapPoints.begin(); it != mCurrMapPoints.end(); it++)
//    {
//        MapPoint* currmappoint = it->second.first;
//        cv::Mat currmp3d = currmappoint->GetWorldPos();
//        geometry_msgs::Point p;
//        p.x = currmp3d.at<float>(0);
//        p.y = currmp3d.at<float>(1);
//        p.z = currmp3d.at<float>(2);
//        mappointvisual.add_line(mCurr_O, p);
//    }
//    mappointvisual.publish_by(pub_curr_mappoint_visual, header);


    //显示轨迹
    path.poses.clear();
    for(auto mDataKF : mDataBaseKF)
    {
        geometry_msgs::PoseStamped pose_stamped;
        Eigen::Quaterniond mKF_R;
        Eigen::Vector3d mKF_O;
        Eigen::Vector3d mKF_t;

        cv::Mat KFPose = mDataKF->GetPose();
        Mat2EigenRt(KFPose,mKF_R,mKF_t,mKF_O);

        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = mKF_O.x();
        pose_stamped.pose.position.y = mKF_O.y();
        pose_stamped.pose.position.z = mKF_O.z();
        path.poses.push_back(pose_stamped);
    }
    path.header = header;
    path.header.frame_id = "world";
    pub_path.publish(path);
}


void Mat2EigenRt(cv::Mat &pose, Eigen::Quaterniond &R, Eigen::Vector3d &t, Eigen::Vector3d &O)
{
    cv::Mat pose_R = pose.rowRange(0,3).colRange(0,3).t();
    cv::Mat pose_t = pose.rowRange(0,3).col(3);
    cv::Mat Ow = -pose_R*pose_t;

    Eigen::Matrix<double,3,3> Eigen_R;
    Eigen_R << pose_R.at<float>(0,0), pose_R.at<float>(0,1), pose_R.at<float>(0,2),
               pose_R.at<float>(1,0), pose_R.at<float>(1,1), pose_R.at<float>(1,2),
               pose_R.at<float>(2,0), pose_R.at<float>(2,1), pose_R.at<float>(2,2);
    R = Eigen::Quaterniond(Eigen_R);

    t << pose_t.at<float>(0), pose_t.at<float>(1), pose_t.at<float>(2);
    O << Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2);
//    ROS_WARN_STREAM("pose_t: " << pose_t);
//    ROS_WARN_STREAM("Eigen_R: " << Eigen_R);
//    ROS_WARN_STREAM("O: " << O);
}


void Eigen2Point(const Eigen::Vector3d& v, geometry_msgs::Point& p) {
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

MarkerVisualization::MarkerVisualization(float r, float g, float b, float a)
        : m_marker_ns("MarkerVisualization"), m_scale(0.2), m_line_width(0.01) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void MarkerVisualization::setScale(double s) {
    m_scale = s;
}
void MarkerVisualization::setLineWidth(double width) {
    m_line_width = width;
}

void MarkerVisualization::add_line(const Eigen::Vector3d &fp, const geometry_msgs::Point &mp)
{
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = m_line_width;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    geometry_msgs::Point pOw, pMw;
    Eigen2Point(fp, pOw);

    marker.points.push_back(pOw);
    marker.points.push_back(mp);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    m_markers.push_back(marker);
}

void MarkerVisualization::add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = m_line_width;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;


    geometry_msgs::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;

    Eigen2Point(q * (m_scale *imlt) + p, pt_lt);
    Eigen2Point(q * (m_scale *imlb) + p, pt_lb);
    Eigen2Point(q * (m_scale *imrt) + p, pt_rt);
    Eigen2Point(q * (m_scale *imrb) + p, pt_rb);
    Eigen2Point(q * (m_scale *lt0 ) + p, pt_lt0);
    Eigen2Point(q * (m_scale *lt1 ) + p, pt_lt1);
    Eigen2Point(q * (m_scale *lt2 ) + p, pt_lt2);
    Eigen2Point(q * (m_scale *oc  ) + p, pt_oc);

    // image boundaries
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_lb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_rb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_rt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_lt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // top-left indicator
    marker.points.push_back(pt_lt0);
    marker.points.push_back(pt_lt1);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lt1);
    marker.points.push_back(pt_lt2);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // optical center connector
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);


    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    m_markers.push_back(marker);
}

void MarkerVisualization::reset() {
    m_markers.clear();
}

void MarkerVisualization::publish_by(ros::Publisher &pub, const std_msgs::Header &header ) {
    visualization_msgs::MarkerArray markerArray_msg;

    for(auto& marker : m_markers) {
        marker.header = header;
        markerArray_msg.markers.push_back(marker);
    }

    pub.publish(markerArray_msg);
}