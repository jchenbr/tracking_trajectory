
#include "macro.h"
#include "traj_utils.h"
#include "target_trajectory_estimator.h"
#include "octomap.h"
#include "trajectory_generator.h" 
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>

using namespace TrackingTrajectory;
using namespace std;

TrackingTrajectory::TargetObservation obs;

/* Publisher */
ros::Publisher traj_pub; // visualize path
ros::Publisher wp_pub; // visualize wapoints
ros::Publisher crd_pub; // visualize corridor
ros::Publisher icrd_pub;
ros::Publisher map_pub;
ros::Publisher tck_traj_pub;

/* Messages*/

visualization_msgs::MarkerArray crd_msg;

VoxelTrajectory::OctoMap * octomap;
double _safe_margin = 0.2;

void visTrackTraj()
{

    Eigen::MatrixXd coef = TrajectoryEstimator(4).estimateTrajectory(obs);

    // get the corridor
    pair<Eigen::MatrixXd, Eigen::MatrixXd> corridor;
    if (!octomap->retPathFromTraj(corridor, coef, 0.0, obs.size() * 1.0))
        return ;

    VoxelTrajectory::TrajectoryGenerator::TrackingCorridorConfig crd_config;

    crd_config.M = corridor.first.rows();
    crd_config.N = 6;
    crd_config.R = 3;
    crd_config.max_acc = 1e10;
    crd_config.max_vel = 1e10;
    crd_config.f_vel = 1.0;
    crd_config.f_acc = 1.0;

    crd_config.traj = coef;
    crd_config.traj_time.resize(crd_config.M);
    for (int i = 0; i < crd_config.M; ++i)
        crd_config.traj_time[i] = 
            make_pair(corridor.first(i, 6), corridor.first(i, 7));

    crd_config.face = corridor.second;

    crd_config.init_state = getStateFromTrajByTime(crd_config.traj, 
            crd_config.traj_time.front().first);
    crd_config.dest_state = getStateFromTrajByTime(crd_config.traj,
            crd_config.traj_time.back().second);

    crd_config.init_state.row(1) = Eigen::RowVectorXd::Zero(3);
    crd_config.init_state.row(2) = Eigen::RowVectorXd::Zero(3);

    {
        crd_config.uninflated_grid = corridor.first.block(0, 0, crd_config.M, _TOT_BDY);

        crd_config.grid = getInflatedCorridor(octomap, 
                crd_config.init_state.row(_STT_POS),
                crd_config.dest_state.row(_STT_POS),
                crd_config.uninflated_grid,
                crd_config.face);
        vector<double> rgba {0.1, 0.7, 0.0, 0.1};
        icrd_pub.publish(getCorridorMsgByMatrix(crd_config.grid, "track_traj/icrd", rgba));
    } 


    //ROS_WARN("[TRACK_TRAJ] ready to go.");
    VoxelTrajectory::TrajectoryGenerator::TrajectoryConfig traj_config
        = VoxelTrajectory::TrajectoryGenerator().getTrackingTrajectory(crd_config);

    //ROS_WARN_STREAM("\n [TRACK_TRAJ] coeff:\n" << traj_config.coef);
    //ROS_WARN_STREAM("\n [TRACK_TRAJ] time: \n" << traj_config.time);

    tck_traj_pub.publish(getPointCloudFromTraj(traj_config.coef, traj_config.time));
}

void pubMapBlocks()
{

    if (map_pub.getNumSubscribers() == 0)
    {
        ros::Duration(5.0).sleep();
    }

    map_pub.publish(getPointCloudFromStdVec(octomap->getPointCloud()));
}

void rcvBlockCloud(const sensor_msgs::PointCloud & cloud)
{

    octomap->insertBlocks(getStdVecFromBlockCloud(cloud));
    pubMapBlocks();
    ROS_WARN_STREAM("[RCV_OBSTACLE] work done.");
}

void visCorridor()
{
    // delete the previous corridor 
    for (auto & mk : crd_msg.markers) mk.action = visualization_msgs::Marker::DELETE;
    crd_pub.publish(crd_msg);

    // get the trajectory
    Eigen::MatrixXd coef = TrajectoryEstimator(4).estimateTrajectory(obs);

    // get the corridor
    pair<Eigen::MatrixXd, Eigen::MatrixXd> corridor;
    octomap->retPathFromTraj(corridor, coef, 0.0, obs.size() * 1.0);
    auto & grid = corridor.first; 

    ROS_WARN_STREAM("[visCorridor]\n" << corridor.first << "\n, \n" << corridor.second);

    vector<double> rbga {0.1, 0.0, 0.8, 0.1};
    crd_msg = getCorridorMsgByMatrix(grid, "track_traj/crd", rbga);
    crd_pub.publish(crd_msg);
}


void visTraj()
{
    TrackingTrajectory::TrajectoryEstimator estimator(4);

    double max_time = obs.size() * 1.0;

    Eigen::MatrixXd coeff = estimator.estimateTrajectory(obs);
    ROS_WARN_STREAM("[TEST_ESTIMATOR] coeff of trajectory polynomial:\n" << coeff);

    sensor_msgs::PointCloud cloud, waypoint;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/map";
    waypoint.header = cloud.header;

    geometry_msgs::Point32 p;

    for (auto & ob: obs)
    {
        p.x = ob.second(_DIM_X);
        p.y = ob.second(_DIM_Y);
        p.z = ob.second(_DIM_Z);
        waypoint.points.push_back(p);
    }
    wp_pub.publish(waypoint);


    traj_pub.publish(getPointCloudFromTraj(coeff, max_time));
}

void rcvWaypoints(const geometry_msgs::PoseStamped & pose)
{
    if (pose.pose.position.z < 0)
    {
        if (obs.empty())
            return ;
        else
        {
            visCorridor();
            visTraj();
            visTrackTraj();
            obs.clear();
        }
    }
    else
    {
        Eigen::VectorXd pt(3);
        pt << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
        obs.push_back(make_pair(obs.size() * 1.0, pt));
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_estimator");
    ros::NodeHandle handle("~");

    // parameter
    double _bdy[_TOT_BDY], resolution;
    handle.param("map/boundary/lower_x", _bdy[_BDY_L_X], -100.0);
    handle.param("map/boundary/upper_x", _bdy[_BDY_R_X], 100.0);
    handle.param("map/boundary/lower_y", _bdy[_BDY_L_Y], -100.0);
    handle.param("map/boundary/upper_y", _bdy[_BDY_R_Y], 100.0);
    handle.param("map/boundary/lower_z", _bdy[_BDY_L_Z], 0.0);
    handle.param("map/boundary/upper_z", _bdy[_BDY_R_Z], 200.0);
    handle.param("map/resolution", resolution, 0.008);
    
    double max_vel, max_acc, f_vel, f_acc;
    handle.param("setting/max_velocity", max_vel, 1.0);
    handle.param("setting/max_acceleration", max_acc, 1.0);
    handle.param("setting/flight_velocity", f_vel, 1.0);
    handle.param("setting/flight_acceleration", f_acc, 1.0);

    octomap = new VoxelTrajectory::OctoMap(_bdy, resolution);

    // publisher
    traj_pub = handle.advertise<sensor_msgs::PointCloud>("est_traj", 10);
    wp_pub = handle.advertise<sensor_msgs::PointCloud>("waypoint", 10);
    crd_pub = handle.advertise<visualization_msgs::MarkerArray>("corridor", 10);
    icrd_pub = handle.advertise<visualization_msgs::MarkerArray>("inflated_corridor", 10);
    map_pub = handle.advertise<sensor_msgs::PointCloud>("map_blocks", 10);
    tck_traj_pub = handle.advertise<sensor_msgs::PointCloud>("tck_traj", 10);

    // subscriber
    ros::Subscriber wp_sub = handle.subscribe("observed_position", 10, rcvWaypoints);
    ros::Subscriber obs_sub = handle.subscribe("obstacle_blocks", 10, rcvBlockCloud);

    ros::spin();
    return 0;
}

