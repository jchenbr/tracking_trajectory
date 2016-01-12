
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <random>

using namespace std;

class SimulatedSensor
{
private:
    ros::Rate _pub_rate = ros::Rate(10);

    ros::Subscriber _odom_sub;

    ros::Publisher _ob_pub;

    ros::Timer _ob_tmr;

    nav_msgs::Odometry _odom;

    std::default_random_engine _r_engine;
    std::normal_distribution<double> _pos_uncertainty;

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
        }
        
        _odom_sub = handle.subscribe("target_odometry", 50, &SimulatedSensor::rcvOdometry, this);

        _ob_pub = handle.advertise<geometry_msgs::PoseStamped>("observed_pose", 50);

        _ob_tmr = handle.createTimer(_pub_rate, &SimulatedSensor::pubObservationCallback, this); 
    }

    void rcvOdometry(const nav_msgs::Odometry odom)
    {
        _odom = odom;
    }

    void pubObservation()
    {
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
