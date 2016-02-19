#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <random>
#include <math.h>
#include "octomap.h"
using namespace std;

class ObstacleGenerator
{
private:
    ros::NodeHandle _handle;
    string _type;

    /* publisher */
    ros::Publisher _obs_pub; // publish the obstacle block

    /* subscriber */
    ros::Subscriber _type_sub; // rcv the obstacle type

    /* useful messages */
    sensor_msgs::PointCloud _obs_msg;
    ros::Duration _delay = ros::Duration(3.0);
    const double _PI = acos(-1); 
    const double _EPS = 1e-5;
public:

    ObstacleGenerator(ros::NodeHandle  &handle)
        :_handle(handle)
    {
        _obs_pub = _handle.advertise
            <sensor_msgs::PointCloud>("obstacle_blocks", 10);

        _type_sub = _handle.subscribe
            ("obstacle_type", 10, &ObstacleGenerator::rcvObstacleType, this);
            
        double dl;
        _handle.param("delay", dl, _delay.toSec());
        _delay = ros::Duration(dl);

        _obs_msg.header.stamp = ros::Time::now();
        _obs_msg.header.frame_id = "/map";
    }

    void genObstacle()
    {
        if (_type == "forest") genObstacleForest();
    }

    void pubObstacle()
    {
        genObstacle();

        _delay.sleep();
        
        if (_obs_pub.getNumSubscribers() == 0)
        {
            ros::Duration max_wait(10.0);
            ros::Time beg_time = ros::Time::now();
            while (ros::ok() && ros::Time::now() - beg_time < max_wait &&
                    _obs_pub.getNumSubscribers() == 0)
                ros::Duration(0.5).sleep();
        }

        _obs_pub.publish(_obs_msg);
    }

    void rcvObstacleType(std_msgs::String type_msg)
    {
        _type = type_msg.data;
        pubObstacle();
    }

    void genObstacleForest()
    {
        vector<double> bdy(6, 0);
        // boundary of allowed area
        _handle.getParam("obstacle/boundary/lower_x", bdy[0]);
        _handle.getParam("obstacle/boundary/upper_x", bdy[1]);
        _handle.getParam("obstacle/boundary/lower_y", bdy[2]);
        _handle.getParam("obstacle/boundary/upper_y", bdy[3]);
        _handle.getParam("obstacle/boundary/lower_z", bdy[4]);
        _handle.getParam("obstacle/boundary/upper_z", bdy[5]);

        ROS_WARN_STREAM("The boundary of obstacle area:" << 
                bdy[0] << "-" << bdy[1] << ", " <<
                bdy[2] << "-" << bdy[3] << "," <<
                bdy[4] << "-" << bdy[5]); 

        int n_tree;
        double h_mean, h_var, r_mean, r_var; // the lower part of a tree
        double H_mean, H_var, R_mean, R_var; // the upper part of a tree
        double _dp = 0.2, _do = 3.0/180.0 * _PI, _dm = 0.2;
        bool is_rotated = false;

        _handle.getParam("obstacle/forest/number", n_tree);
        _handle.getParam("obstacle/forest/stem/height/mean", h_mean);
        _handle.getParam("obstacle/forest/stem/height/variance", h_var);
        _handle.getParam("obstacle/forest/stem/radius/mean", r_mean);
        _handle.getParam("obstacle/forest/stem/radius/variance", r_var);
        _handle.getParam("obstacle/forest/top/height/mean", H_mean);
        _handle.getParam("obstacle/forest/top/height/variance", H_var);
        _handle.getParam("obstacle/forest/top/radius/mean", R_mean);
        _handle.getParam("obstacle/forest/top/radius/variance", R_var);

        // for rotation
        _handle.param("obstacle/forest/is_rotated", is_rotated, is_rotated);
        _handle.param("obstacle/forest/resolution/position", _dp, _dp);
        _handle.param("obstacle/forest/resolution/orientation", _do, _do);
        _handle.param("obstacle/forest/resolution/map", _dm, _dm);
        ROS_WARN("[OBSTACLE_GENERATOR] n=%d , r=(%lf, %lf), R=(%lf,%lf), h=(%lf,%lf), H=(%lf,%lf) ", 
                    n_tree, r_mean, r_var, R_mean, R_var, h_mean, h_var, H_mean, H_var);
        VoxelTrajectory::OctoMap _map(bdy.data(), _dm);
        vector<double> blk;


        double x, y, h, H, r, R;
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_real_distribution<double> rand_x(bdy[0], bdy[1]);
        std::uniform_real_distribution<double> rand_y(bdy[2], bdy[3]);
        std::uniform_real_distribution<double> rand_o(0, 2.0 * _PI);

        std::normal_distribution<double> rand_h(h_mean, h_var);
        std::normal_distribution<double> rand_r(r_mean, r_var);
        std::normal_distribution<double> rand_H(H_mean, H_var);
        std::normal_distribution<double> rand_R(R_mean, R_var);

        auto add_rotated_obstacle = 
            [&](double x, double y, double z, double r_x, double r_y, double r_h, double yaw)->void
        {
            for (double _x = -r_x; _x <= r_x; _x += _dp)
                for (double _y = -r_y; _y <= r_y; _y += _dp)
                {

                    double X = x + _x * cos(yaw) - _y * sin(yaw);
                    double Y = y + _x * sin(yaw) + _y * cos(yaw);
                    blk.push_back(X - _EPS);
                    blk.push_back(X + _EPS);
                    blk.push_back(Y - _EPS);
                    blk.push_back(Y + _EPS);
                    blk.push_back(0);
                    blk.push_back(z + r_h + rand_H(eng) + _EPS);
                }
            return;
        };
        
        ROS_WARN("[OBSTACLE_GENERATOR] before the construction of obstacles.");
        if (is_rotated)
        {
            _obs_msg.points.clear();
            for (int idx = 0; idx < n_tree; ++idx)
            {
                double x = rand_x(eng), y = rand_y(eng), z = rand_h(eng);
                add_rotated_obstacle(x, y, 0.0,abs(rand_r(eng)), abs(rand_r(eng)), z, rand_o(eng));
                //add_rotated_obstacle(x, y, z,abs(rand_R(eng)), abs(rand_R(eng)), z + rand_H(eng), rand_o(eng));
            }
            #if 0
            _map.insertBlocks(blk);

            auto pts = _map.getPointCloud();
            geometry_msgs::Point32 p, P;
            for (size_t idx = 0; idx < pts.size(); idx += 3)
            {
                p.x = pts[idx + 0] - _EPS;
                p.y = pts[idx + 1] - _EPS;
                p.z = pts[idx + 2] - _EPS;
                
                P.x = pts[idx + 0] + _EPS;
                P.y = pts[idx + 1] + _EPS;
                P.z = pts[idx + 2] + _EPS;
                _obs_msg.points.push_back(p);
                _obs_msg.points.push_back(P);
            }
            #endif
            
            geometry_msgs::Point32 p, P;
            for (size_t idx = 0; idx < blk.size(); idx += 6)
            {
                p.x = blk[idx + 0] - _EPS;
                p.y = blk[idx + 2] - _EPS;
                p.z = blk[idx + 4] - _EPS;
                
                P.x = blk[idx + 1] + _EPS;
                P.y = blk[idx + 3] + _EPS;
                P.z = blk[idx + 5] + _EPS;
                _obs_msg.points.push_back(p);
                _obs_msg.points.push_back(P);
            }
            ROS_WARN("[OBSTACLE_GENERATOR] number of obstacles: %d.", (int)_obs_msg.points.size()/2);
        }
        else
        {
            _obs_msg.points.resize(n_tree << 2);

            for (int idx = 0; idx < n_tree; ++idx)
            {
                x = rand_x(eng);
                y = rand_y(eng);
                h = abs(rand_h(eng));
                r = abs(rand_r(eng));
                H = abs(rand_H(eng));
                R = abs(rand_R(eng));
                ROS_WARN("[OBSTACLE_GENERATOR] (%lf, %lf), r=%lf, R=%lf, h=%lf, H=%lf.", x, y, r, R, h, H);
                // the lower part of the tree
                _obs_msg.points[idx << 2 | 0].x = x - r;
                _obs_msg.points[idx << 2 | 1].x = x + r;
                _obs_msg.points[idx << 2 | 0].y = y - r;
                _obs_msg.points[idx << 2 | 1].y = y + r;
                _obs_msg.points[idx << 2 | 0].z = bdy[4];
                _obs_msg.points[idx << 2 | 1].z = bdy[4] + h;

                // the upper part of the tree
                _obs_msg.points[idx << 2 | 2].x = x - r - R;
                _obs_msg.points[idx << 2 | 3].x = x + r + R;
                _obs_msg.points[idx << 2 | 2].y = y - r - R;
                _obs_msg.points[idx << 2 | 3].y = y + r + R;
                _obs_msg.points[idx << 2 | 2].z = bdy[4] + h;
                _obs_msg.points[idx << 2 | 3].z = bdy[4] + h + H;
            }
        }
    }

    void setType(string str){_type = str;}
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ObstacleGenerator");
    ros::NodeHandle handle("~");

    ObstacleGenerator generator(handle);
    generator.setType("forest");
    generator.pubObstacle();

    ros::spin();

    return 0;
}
