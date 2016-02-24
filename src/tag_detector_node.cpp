#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <deque>
#include <vector>

#include "traj_utils.h"

using namespace std;

class TagDetector
{
public:
    int tag_id;

    TagDetector(ros::NodeHandle & handle);

    void rcvImage(const sensor_msgs::ImageConstPtr & p_img);
    void rcvOdometry(const nav_msgs::Odometry &odom);
    void rcvTrigger(const geometry_msgs::PoseStamped &trg);
    //void pubTagPose();
    void visDetection();
private:
    double _marker_size;

    // aruco things
    aruco::CameraParameters _cam_param;
    aruco::MarkerDetector _marker_detector;
    vector<aruco::Marker> _markers;

    // publisher
    ros::Publisher _pose_pub;
    // subscriber
    ros::Subscriber _img_sub;
    ros::Subscriber _odom_sub;
    ros::Subscriber _trg_sub;

    // visualziation flag
    bool _is_vis = true;
    bool _is_triggered = false;

    // OpenCV things
    cv_bridge::CvImagePtr _p_bridge;

    // messages
    geometry_msgs::PoseStamped _pose;
    geometry_msgs::Pose _pose_cam_bd;

    nav_msgs::Odometry _odom;
    deque<nav_msgs::Odometry> _odom_queue;
    size_t _odom_queue_size = 300;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tag_detector_node");
    ros::NodeHandle handle("~");

    TagDetector detector(handle);
    ros::spin();

    return 0;
}

TagDetector::TagDetector(ros::NodeHandle & handle)
{
    {// load parameter
        string path;
        handle.getParam("camera_calibration_file", path);
        _cam_param.readFromXMLFile(path);

        vector<double> pos, ort;
        handle.getParam("camera_position", pos);
        handle.getParam("camera_orientation", ort);
        _pose_cam_bd.position.x = pos[_DIM_X];
        _pose_cam_bd.position.y = pos[_DIM_Y];
        _pose_cam_bd.position.z = pos[_DIM_Z];
        _pose_cam_bd.orientation.w = ort[0];
        _pose_cam_bd.orientation.x = ort[1];
        _pose_cam_bd.orientation.y = ort[2];
        _pose_cam_bd.orientation.z = ort[3];

        int _sz = _odom_queue_size;
        handle.param("odom_queue_size", _sz, _sz);
        _odom_queue_size = _sz;

        handle.getParam("tag_id", tag_id);
        handle.getParam("marker_size", _marker_size);
        handle.param("visualization_flag", _is_vis, _is_vis);
    }

    {// subscribed topic
        _img_sub = handle.subscribe("camera_image", 50, &TagDetector::rcvImage, this);
        _odom_sub = handle.subscribe("odometry", 50, &TagDetector::rcvOdometry, this);
        _trg_sub = handle.subscribe("trigger", 10, &TagDetector::rcvTrigger, this);
    }

    {// published topic
        _pose_pub = handle.advertise<geometry_msgs::PoseStamped>("tag_pose", 50);
    }

    {// visualization
        //cv::namedWindow("camera_image");
    }
}

void TagDetector::rcvTrigger(const geometry_msgs::PoseStamped &trg)
{
    _is_triggered = true;
}

void TagDetector::rcvImage(const sensor_msgs::ImageConstPtr & p_image)
{
    if (!_is_triggered) return ;
     // basic info and prepration
#if 0
    _pose.header = p_image->header;
#else
    _pose.header.stamp = ros::Time::now();
#endif
    _p_bridge = cv_bridge::toCvCopy(p_image, sensor_msgs::image_encodings::MONO8);

    { // dectect marker
        _marker_detector.detect(_p_bridge->image(cv::Rect(0, 0, 752, 480)), _markers, _cam_param, _marker_size);
        for (auto &mk: _markers)
        {
            if (mk.isValid() && mk.id == tag_id)
            {
                double pos[3], ort[4];
                mk.OgreGetPoseParameters(pos, ort);

                _pose.pose.position.x = pos[0];
                _pose.pose.position.y = pos[1];
                _pose.pose.position.z = pos[2];

                _pose.pose.orientation.w = ort[0];
                _pose.pose.orientation.x = ort[1];
                _pose.pose.orientation.y = ort[2];
                _pose.pose.orientation.z = ort[3];

                if (_odom_queue.empty()) continue;
                nav_msgs::Odometry odom = _odom;
                for (auto & o: _odom_queue)
                    if (o.header.stamp > p_image->header.stamp)
                    {
                        odom = o;
                        break;
                    }
                    /*
                ROS_WARN("[camera in body](%lf, %lf, %lf), [%lf, %lf, %lf, %lf]", 
                    _pose_cam_bd.position.x,
                    _pose_cam_bd.position.y,
                    _pose_cam_bd.position.z,
                    _pose_cam_bd.orientation.w,
                    _pose_cam_bd.orientation.x,
                    _pose_cam_bd.orientation.y,
                    _pose_cam_bd.orientation.z
                    );
                    */

                _pose.header.stamp = _odom.header.stamp;
                _pose.header.frame_id = "/map";
                _pose.pose = getWorldPoseMsgFromCam(
                        _pose.pose, _pose_cam_bd, odom.pose.pose);

                _pose_pub.publish(_pose);

            }
        }
    }
    visDetection();
}

void TagDetector::rcvOdometry(const nav_msgs::Odometry & odom)
{
    _odom = odom;
    _odom_queue.push_back(odom);
    while (_odom_queue.size() > _odom_queue_size) _odom_queue.pop_front();
}

void TagDetector::visDetection()
{
    if (!_is_vis) return ;
    cv::Mat img = _p_bridge->image(cv::Rect(0, 0, 752, 480));
    for (auto & mk: _markers)
    {
        //ROS_INFO_STREAM("[Marker]\n" << mk);
        mk.draw(img, cv::Scalar(0, 0, 255), 2);
    }

    if (_cam_param.isValid())
        for (auto & mk: _markers) 
        {
            if (mk.id != tag_id) continue;
            aruco::CvDrawingUtils::draw3dCube(img, mk, _cam_param);
            aruco::CvDrawingUtils::draw3dAxis(img, mk, _cam_param);
        }

    cv::imshow("camera_image", img);

    cv::waitKey(10);
}
