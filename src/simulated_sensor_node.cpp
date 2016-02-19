
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <random>

#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <math.h>
#include "octomap.h"

using namespace std;

class SimulatedSensor
{
private:
    ros::Rate _pub_rate = ros::Rate(10);

    ros::Subscriber _odom_sub;
    ros::Subscriber _blks_sub;
    ros::Subscriber _body_odom_sub;

    ros::Publisher _ob_pub;

    ros::Timer _ob_tmr;

    nav_msgs::Odometry _odom;
    nav_msgs::Odometry _body_odom;

    std::default_random_engine _r_engine;
    std::normal_distribution<double> _pos_uncertainty;

    VoxelTrajectory::OctoMap *_map;
    const double _PI = acos(-1);

    double _sensing_range = 3.0;
    double _view_feild = _PI * 1.0 / 3.0;

public:
    SimulatedSensor(ros::NodeHandle & handle)
    {
        {
            double rate;
            handle.param("observation_rate", rate, 10.0);
            _pub_rate = ros::Rate(rate);
        }

        {
            double mean, var;
            handle.param("uncertainty/position/mean", mean, 0.0);
            handle.param("uncertainty/position/variance", var, 0.0);
            _pos_uncertainty = std::normal_distribution<double>(mean, var);

            handle.param("sensing/range", _sensing_range, _sensing_range);
            handle.param("sensing/view_field", _view_feild, _view_feild);

            double resolution;
            vector<double> bdy;
            handle.getParam("map/resolution", resolution);
            handle.getParam("map/boundary", bdy);
            _map = new VoxelTrajectory::OctoMap(bdy.data(), resolution);
        }
        
        _odom_sub = handle.subscribe("target_odometry", 50, &SimulatedSensor::rcvOdometry, this);
        _blks_sub = handle.subscribe("obstacle_blocks", 10, &SimulatedSensor::rcvObstaclBlocks, this);
        _body_odom_sub = handle.subscribe("body_odometry", 50, &SimulatedSensor::rcvBodyOdometry, this);

        _ob_pub = handle.advertise<geometry_msgs::PoseStamped>("observed_pose", 50);

        _ob_tmr = handle.createTimer(_pub_rate, &SimulatedSensor::pubObservationCallback, this); 
    }

    void rcvOdometry(const nav_msgs::Odometry & odom)
    {
        _odom = odom;
    }

    void rcvBodyOdometry(const nav_msgs::Odometry & odom)
    {
        _body_odom = odom;
    }

    void rcvObstaclBlocks(const sensor_msgs::PointCloud & cloud)
    {
        vector<double> blk;
        blk.reserve(cloud.points.size() * 3);
        for (size_t idx = 0; idx < cloud.points.size(); idx +=2)
        {
            blk.push_back(cloud.points[idx|0].x);
            blk.push_back(cloud.points[idx|1].x);
            blk.push_back(cloud.points[idx|0].y);
            blk.push_back(cloud.points[idx|1].y);
            blk.push_back(cloud.points[idx|0].z);
            blk.push_back(cloud.points[idx|1].z);
        }
        _map->insertBlocks(blk);
    }

    void pubObservation()
    {
        Eigen::Vector3d p_b(
                _body_odom.pose.pose.position.x,
                _body_odom.pose.pose.position.y,
                _body_odom.pose.pose.position.z);

        Eigen::Vector3d p_t(
                _odom.pose.pose.position.x,
                _odom.pose.pose.position.y,
                _odom.pose.pose.position.z);
        
        double body_yaw = tf::getYaw(_body_odom.pose.pose.orientation);
        double yaw = atan2(p_t(1) - p_b(1), p_t(0) - p_b(0));
        double d_yaw = abs(body_yaw - yaw);
        d_yaw = fmod(d_yaw + _PI, 2 * _PI) - _PI; 
#if 0
        ROS_WARN_STREAM("[simulated_tag_observer] target " << p_t.transpose() << 
                ", tracker " << p_b.transpose() << ", atan2(-1, -1) " << atan2(-1.0, -1.0));
        ROS_WARN_STREAM("[simulated_tag_observer] d_yaw = " << d_yaw << ", dist = "
                << (p_b - p_t).norm() << ", yaw_b = " << body_yaw << ", yaw_r = " << yaw);
#endif
        if (abs(d_yaw) > _view_feild || (p_b - p_t).norm() > _sensing_range) return ;

        for (double t = 0.01; t <= 1.0 + 1e-7; t+=0.01)
        {
            Eigen::Vector3d p = p_b * t + p_t * (1.0 - t);
            if (_map->testObstacle(p.data())) return ;
        }

        geometry_msgs::PoseStamped pose;
        pose.header = _odom.header;
        pose.pose = _odom.pose.pose;
        pose.pose.position.x += _pos_uncertainty(_r_engine);
        pose.pose.position.y += _pos_uncertainty(_r_engine);
        pose.pose.position.z += _pos_uncertainty(_r_engine);
        _ob_pub.publish(pose);
    }

    void pubObservationCallback(const ros::TimerEvent & evt)
    {
        pubObservation();
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "simulated_sensor");

    ros::NodeHandle handle("~");
    SimulatedSensor sensor(handle);
    ros::spin();
    return 0;   
}
