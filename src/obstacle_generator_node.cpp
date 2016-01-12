#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <random>
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

        _handle.getParam("obstacle/forest/number", n_tree);
        _handle.getParam("obstacle/forest/stem/height/mean", h_mean);
        _handle.getParam("obstacle/forest/stem/height/variance", h_var);
        _handle.getParam("obstacle/forest/stem/radius/mean", r_mean);
        _handle.getParam("obstacle/forest/stem/radius/variance", r_var);
        _handle.getParam("obstacle/forest/top/height/mean", H_mean);
        _handle.getParam("obstacle/forest/top/height/variance", H_var);
        _handle.getParam("obstacle/forest/top/radius/mean", R_mean);
        _handle.getParam("obstacle/forest/top/radius/variance", R_var);
            ROS_WARN("[OBSTACLE_GENERATOR] n=%d , r=(%lf, %lf), R=(%lf,%lf), h=(%lf,%lf), H=(%lf,%lf) ", 
                    n_tree, r_mean, r_var, R_mean, R_var, h_mean, h_var, H_mean, H_var);


        double x, y, h, H, r, R;
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_real_distribution<double> rand_x(bdy[0], bdy[1]);
        std::uniform_real_distribution<double> rand_y(bdy[2], bdy[3]);

        std::normal_distribution<double> rand_h(h_mean, h_var);
        std::normal_distribution<double> rand_r(r_mean, r_var);
        std::normal_distribution<double> rand_H(H_mean, H_var);
        std::normal_distribution<double> rand_R(R_mean, R_var);

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
