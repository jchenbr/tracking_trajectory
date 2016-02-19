
#include "octomap.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <fstream>
#include <cmath>
#include <cstring>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Dense>

using namespace std;
using namespace VoxelTrajectory;

class SimulatedLaser
{
private:
    ros::Subscriber _odom_sub;
    ros::Subscriber _blks_sub;
    ros::Subscriber _pts_sub;
    ros::Publisher _scan_pub;

    bool _has_odom = false;

    VoxelTrajectory::OctoMap * _map;
    nav_msgs::Odometry _odom;
    sensor_msgs::LaserScan _scan;
    int _sensing_rate = 100;                // Hz
    vector<float> _sensed_pts;
    double _sensing_range_inc = 0.1;

    //laser

    const double _PI = acos(-1.0);          // arc
    const double _EPS = 1e-7;
public:

    SimulatedLaser(ros::NodeHandle & handle, ros::NodeHandle & map_handle)
    {
        { // set up the inner map
            double bdy[_TOT_BDY], resolution;
            map_handle.param("map/boundary/lower_x", bdy[_BDY_x], -100.0);
            map_handle.param("map/boundary/upper_x", bdy[_BDY_X], 100.0);
            map_handle.param("map/boundary/lower_y", bdy[_BDY_y], -100.0);
            map_handle.param("map/boundary/upper_y", bdy[_BDY_Y], 100.0);
            map_handle.param("map/boundary/lower_z", bdy[_BDY_z], 0.0);
            map_handle.param("map/boundary/upper_z", bdy[_BDY_Z], 200.0);
            map_handle.param("map/resolution", resolution, 0.4);
            _map = new VoxelTrajectory::OctoMap(bdy, resolution);
        }

        { // settings 
            double angle_min, angle_max, angle_inc, range_min, range_max;
            handle.param("limited_sensing/rate", _sensing_rate, 20);
            handle.param("limited_sensing/angle/min", angle_min, 2.0 * _PI * 0 / 1800);
            handle.param("limited_sensing/angle/max", angle_max, 2.0 * _PI * 1799 / 1800);
            handle.param("limited_sensing/angle/increment", angle_inc, 2.0 * _PI * 1 / 1800);
            handle.param("limited_sensing/range/min", range_min, 0.0);
            handle.param("limited_sensing/range/max", range_max, 10.0);
            handle.param("limited_sensing/range/increment", _sensing_range_inc, 0.1);
            _scan.angle_min = angle_min;
            _scan.angle_max = angle_max;
            _scan.angle_increment = angle_inc;
            _scan.range_min = range_min;
            _scan.range_max = range_max;
        }

        { // set up publisher and subscribers
            _odom_sub = 
                handle.subscribe("odometry", 50, &SimulatedLaser::setOdometry, this);
            _blks_sub = 
                handle.subscribe("obstacle_blocks", 2, &SimulatedLaser::setObstacleBlocks, this);
            _pts_sub = 
                handle.subscribe("obstacle_points", 2, &SimulatedLaser::setObstaclePoints, this);

            _scan_pub = 
                handle.advertise<sensor_msgs::LaserScan>("scan", 10);
        }
    }

    int getRate()
    {
        return this->_sensing_rate;
    }

    void setOdometry(const nav_msgs::Odometry & odom)
    {
        _odom = odom;
        _has_odom = true;
    }
    
    void setObstaclePoints(const sensor_msgs::PointCloud & cloud)
    {
        vector<double> blk;
        blk.reserve(cloud.points.size() * _TOT_DIM);
        for (size_t idx = 0; idx < cloud.points.size(); idx+=1)
        {
            blk.push_back(cloud.points[idx].x - _EPS);
            blk.push_back(cloud.points[idx].x + _EPS);
            blk.push_back(cloud.points[idx].y - _EPS);
            blk.push_back(cloud.points[idx].y + _EPS);
            blk.push_back(cloud.points[idx].z - _EPS);
            blk.push_back(cloud.points[idx].z + _EPS);
        }
        _map->insertBlocks(blk);
    }

    void setObstacleBlocks(const sensor_msgs::PointCloud & cloud)
    {
        vector<double> blk;
        blk.reserve(cloud.points.size() * _TOT_DIM);
        for (size_t idx = 0; idx < cloud.points.size(); idx+=2)
        {
            blk.push_back(cloud.points[idx | 0].x - _EPS);
            blk.push_back(cloud.points[idx | 1].x + _EPS);
            blk.push_back(cloud.points[idx | 0].y - _EPS);
            blk.push_back(cloud.points[idx | 1].y + _EPS);
            blk.push_back(cloud.points[idx | 0].z - _EPS);
            blk.push_back(cloud.points[idx | 1].z + _EPS);
        }
        _map->insertBlocks(blk);
    }

    void pubLaserScan()
    {
        _scan.header.frame_id = "/quadrotor";
        _scan.header.stamp = _odom.header.stamp;

        _scan.ranges.clear();
        _scan.scan_time = 0.0;

        Eigen::Matrix3d rotation = 
            Eigen::Quaterniond(
                    _odom.pose.pose.orientation.w,
                    _odom.pose.pose.orientation.x,
                    _odom.pose.pose.orientation.y,
                    _odom.pose.pose.orientation.z
                    ).toRotationMatrix();

        Eigen::Vector3d trans(
                _odom.pose.pose.position.x,
                _odom.pose.pose.position.y,
                _odom.pose.pose.position.z);

        for (double yaw = _scan.angle_min; yaw <= _scan.angle_max + _EPS; yaw += _scan.angle_increment)
        {
            double r;
            for (r = _scan.range_min; r <= _scan.range_max + _EPS; r += 0.1)
            {
                Eigen::Vector3d p(r * cos(yaw), r * sin(yaw), 0.0);
                p = rotation * p + trans; 
                if (_map->testObstacle(p.data())) break;
            }
            _scan.ranges.push_back(r);
        }

        _scan_pub.publish(_scan);
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "simulated_laser");
    ros::NodeHandle handle("~"), map_handle("~");

    SimulatedLaser laser(handle, map_handle);


    ros::Rate loop_rate(laser.getRate());
    while (ros::ok())
    {
        laser.pubLaserScan();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

