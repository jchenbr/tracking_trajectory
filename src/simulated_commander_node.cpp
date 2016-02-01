#include "macro.h"
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <random>
#include "voxel_trajectory/CheckObstacleByPoints.h"
using namespace std;

class SimulatedCommander
{
private:
    ros::Duration _delay = ros::Duration(6.0);
    ros::Duration _cmd_drt = ros::Duration(30.0);
    vector<double> _bdy; 

    ros::Publisher _cmd_pub;
    ros::Timer _cmd_tmr;
    ros::ServiceClient _obs_clt;
    voxel_trajectory::CheckObstacleByPoints _obs_srv;
    
    std::default_random_engine _generator;
    std::uniform_real_distribution<double> _dsb_x, _dsb_y, _dsb_z;
public:
    SimulatedCommander(ros::NodeHandle & handle)
    {

        double delay, drt;
        handle.param("setting/delay", delay, _delay.toSec());
        _delay = ros::Duration(delay);
        
        handle.param("setting/command_cycle_time", drt, _cmd_drt.toSec());
        _cmd_drt = ros::Duration(drt);

        using namespace TrackingTrajectory;
        _bdy.resize(_TOT_BDY);
        handle.param("boundary/lower_x", _bdy[_BDY_L_X], -1.0);
        handle.param("boundary/upper_x", _bdy[_BDY_R_X], 1.0);
        handle.param("boundary/lower_y", _bdy[_BDY_L_Y], -1.0);
        handle.param("boundary/upper_y", _bdy[_BDY_R_Y], 1.0);
        handle.param("boundary/lower_z", _bdy[_BDY_L_Z], 0.0);
        handle.param("boundary/upper_z", _bdy[_BDY_R_Z], 1.0);
#if 0
        ROS_WARN("[bdyyyyyyyyyyyyyyyyyyyyyyyyy] %lf %lf %lf %lf %lf %lf", 
            _bdy[_BDY_L_X],_bdy[_BDY_R_X],_bdy[_BDY_L_Y],
            _bdy[_BDY_R_Y],_bdy[_BDY_L_Z],_bdy[_BDY_R_Z]);
        ROS_WARN("[delayyyyyyyyyyyyyyyyyyyyyyy] %lf", _delay.toSec());
        ROS_WARN("[cycle_timeeeeeeeeeeeeeeeeee] %lf", _cmd_drt.toSec());
#endif
        _dsb_x = uniform_real_distribution<double>(_bdy[_BDY_L_X], _bdy[_BDY_R_X]);
        _dsb_y = uniform_real_distribution<double>(_bdy[_BDY_L_Y], _bdy[_BDY_R_Y]);
        _dsb_z = uniform_real_distribution<double>(_bdy[_BDY_L_Z], _bdy[_BDY_R_Z]);

        _cmd_pub = handle.advertise<nav_msgs::Path>(
                "waypoints", 10);
        _cmd_tmr = handle.createTimer(_cmd_drt, &SimulatedCommander::pubCommand, this);
        string str;
        handle.getParam("check_points_path", str);
        _obs_clt = handle.serviceClient<voxel_trajectory::CheckObstacleByPoints>(str);
        _obs_srv.request.size = 1;
        _obs_srv.request.x.resize(1);
        _obs_srv.request.y.resize(1);
        _obs_srv.request.z.resize(1);
    }

    void pubCommand(const ros::TimerEvent &evt)
    {
        nav_msgs::Path wp;
        wp.header.frame_id = "/map";
        wp.header.stamp = ros::Time::now();


        geometry_msgs::PoseStamped pose;
        pose.header = wp.header;
        pose.pose.orientation.w = 1.0;
        do{
            pose.pose.position.x = _dsb_x(_generator);
            pose.pose.position.y = _dsb_y(_generator);
            pose.pose.position.y = _dsb_z(_generator);

            _obs_srv.request.x[0] = pose.pose.position.x; 
            _obs_srv.request.y[0] = pose.pose.position.y; 
            _obs_srv.request.z[0] = pose.pose.position.z; 
            _obs_clt.call(_obs_srv);

            ROS_WARN_STREAM("[ccccccccccccccccccccccommander]" << 
                    pose.pose.position.x << ", " <<
                    pose.pose.position.y << ", " <<
                    pose.pose.position.z << ", = " << (int)_obs_srv.response.is_occupied[0]);
        } while (_obs_srv.response.is_occupied.front());
        

        wp.poses.push_back(pose);

        _cmd_pub.publish(wp);
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "simulated_commander");
    ros::NodeHandle handle("~");
    SimulatedCommander cmd(handle);
    ros::spin();
    return 0;
}
