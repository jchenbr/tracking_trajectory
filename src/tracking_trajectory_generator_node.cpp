#include "macro.h"
#include "traj_utils.h"
#include "target_trajectory_estimator.h"
#include "octomap.h"
#include "trajectory_generator.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

#include <functional>
#include <algorithm>
#include <vector>
#include <deque>

using namespace std;
using namespace TrackingTrajectory;


class TrackingTrajectoryGenerator
{
    // todo/issue:
    //  1. start from random position. (done)
    //  2. command to track and stop.
    //  3. keep a safe distance. (done)
    //  4. trajectory generation error code. (done)  
    //  5. observation uncertaninty. (done)
    //  6. strat point infeasible. (done)
    //  7. uneven weight to each observation. (which strategy?)
    //  8. best estimation polynomial oreder (3)
    //  9. destination constraints. (done)
    //  10. bug in the inflation part, or say in the corridor generation. (done)
    //  11. lambda choosing. (provieded the interface)
    //  12. close to the obstacle, or started from the obstacled region.
    //  13. any stopping policy ? (done, must stop at the end of the tracking traj)
    //  14. deal with observation lost? 
    //  15. test under random commander! (done)
    //  16. obstsacle surface problem. (will be done by multi-to-one path finding)
    //  17. tracking vs. stop policy (stop policy is used)
    //  18. orientation moment can not be taken into consider. (done by 20.)
    //  19. something wrong with the maintainness of the octree and graph. (done)
    //  20. replan disturbance. (done by using the desired state as initial state)
    //  21. distance between target. ()
    //  22. regeneration. ()
    //  23. laser message 
private:

    uint32_t _traj_id = 1;
    bool _flag_vis = true;

    ///> io-channel
    ros::Subscriber _cmd_sub;
    ros::Subscriber _odom_sub;  // odometry
    ros::Subscriber _blks_sub;  // map blocks cloud
    ros::Subscriber _pts_sub;   // map points cloud
    ros::Subscriber _obs_sub;   // the target observation

    ros::Publisher _traj_pub;   // flight trajectory 
    ros::Publisher _vis_est_pub; // visual target trajectory
    ros::Publisher _vis_trk_pub; // visual 
    ros::Publisher _vis_map_pub;
    ros::Publisher _vis_crd_pub;
    ros::Publisher _vis_obs_pub;

     ///> current innner states infomation
    nav_msgs::Odometry _odom;

    ///> sub-module
  
    // info about the target trajectory estimation
    TrackingTrajectory::TrajectoryEstimator * _estimator = NULL;

    deque<pair<double, Eigen::VectorXd> > _tgt_obs;
    int _sz_tgt_obs = 200;
    double _pdt_end = 5.0, _pdt_beg = 0.0;
    int _n_dgr_est = 4;
    visualization_msgs::MarkerArray _crd_msg;

    // info about map
    VoxelTrajectory::OctoMap * _map = NULL;

    ros::Timer _vis_map_tmr; 
    ros::Duration _vis_map_drt = ros::Duration(1.0);
    vector<double> _vis_crd_rbga = {0.1, 0.0, 0.7, 0.1};

    vector<double> _bdy {-100.0, 100.0, -100.0, 100.0, 0.0, 200.0};
    double _map_resolution = 0.2 * 0.2 * 0.2;
    double _safe_margin = 0.2;

    // info about the tracking trajectory generation
    VoxelTrajectory::TrajectoryGenerator * _generator = NULL;
    VoxelTrajectory::TrajectoryGenerator::TrackingCorridorConfig _crd_config;
    VoxelTrajectory::TrajectoryGenerator::TrajectoryConfig _traj_config;

    ros::Timer _reg_traj_tmr;
    ros::Duration _reg_traj_drt = ros::Duration(3.0);

    int _n_dgr_traj = 6, _n_dgr_min = 3;
    double _max_vel = 1.0, _max_acc = 1.0;
    double _flt_vel = 1.0, _flt_acc = 1.0;
    Eigen::RowVectorXd _kp_dst;

    quadrotor_msgs::PolynomialTrajectory _traj_msg;

    ///> internal mechanism
    void _initMap();

public:
    ///>
    TrackingTrajectoryGenerator(ros::NodeHandle & handle);

    void rcvCurrentOdometry(const nav_msgs::Odometry & odom);
    void rcvTargetObservation(const geometry_msgs::PoseStamped & pose);
    void rcvGlobalPointCloud(const sensor_msgs::PointCloud & cloud);
    void rcvGlobalBlockCloud(const sensor_msgs::PointCloud & cloud);

    void pubTrackingTrajectory();
    void pubVisualTargetTrajectory();
    void pubVisualTargetCorridor();
    void pubVisualMapGrids();
    void pubVisualTrackingTrajectory();
    void pubVisualTargetObservation();

    void regTrajectoryCallback(const ros::TimerEvent & evt);
    void visMapCallback(const ros::TimerEvent & evt);
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tracking_trajectory_generator");
    ros::NodeHandle handle("~");

    TrackingTrajectoryGenerator generator(handle);

    ros::spin();

    return 0;
}

void TrackingTrajectoryGenerator::_initMap()
{
    if (_map != NULL) delete _map;
    _map = new VoxelTrajectory::OctoMap(_bdy.data(), _map_resolution);
}

TrackingTrajectoryGenerator::TrackingTrajectoryGenerator(ros::NodeHandle & crt_handle)
{
    string param_namespace, msg_namespace;
    crt_handle.param<string>("parameter_namespace", param_namespace, "~");
    crt_handle.param<string>("message_namespace", msg_namespace, "~");

    ros::NodeHandle param_handle(param_namespace), msg_handle(msg_namespace);

    { ///> io topic
        _odom_sub = msg_handle.subscribe("odometry", 
                50, &TrackingTrajectoryGenerator::rcvCurrentOdometry, this,
                ros::TransportHints().tcpNoDelay());
        _obs_sub = msg_handle.subscribe("target_observation",
                50, &TrackingTrajectoryGenerator::rcvTargetObservation, this);
        _pts_sub = msg_handle.subscribe("obstacle_points",
                10, &TrackingTrajectoryGenerator::rcvGlobalPointCloud, this);
        _blks_sub = msg_handle.subscribe("obstacle_blocks",
                10, &TrackingTrajectoryGenerator::rcvGlobalBlockCloud, this);

        _traj_pub = msg_handle.advertise<quadrotor_msgs::PolynomialTrajectory>(
                "flight_trajectory", 10);
        _vis_trk_pub = msg_handle.advertise<sensor_msgs::PointCloud>(
                "visual_flight_trajectory", 10);
        _vis_est_pub = msg_handle.advertise<sensor_msgs::PointCloud>(
                "visual_estimated_trajetory", 10);
        _vis_crd_pub = msg_handle.advertise<visualization_msgs::MarkerArray>(
                "visual_flight_corridor", 10);
        _vis_map_pub = msg_handle.advertise<sensor_msgs::PointCloud>(
                "visual_map_grids", 10);
        _vis_obs_pub = msg_handle.advertise<sensor_msgs::PointCloud>(
                "visual_target_observation", 10);
    }

    { ///> load dynamic parameters
        param_handle.param("dynamic/trajectory_degree", _n_dgr_traj, _n_dgr_est);
        param_handle.param("dynamic/minimization_degree", _n_dgr_min, _n_dgr_min);
        param_handle.param("dynamic/max_velocity", _max_vel, _max_vel);
        param_handle.param("dynamic/max_acceleration", _max_acc, _max_acc);
        param_handle.param("dynamic/flight_velocity", _flt_vel, _flt_vel);
        param_handle.param("dynamic/flight_acceleration", _flt_acc, _flt_acc);

        _crd_config.N = _n_dgr_traj;
        _crd_config.R = _n_dgr_min;
        _crd_config.max_acc = _max_acc;
        _crd_config.max_vel = _max_vel;
        _crd_config.f_acc = _flt_acc;
        _crd_config.f_vel = _flt_vel;
    }

    { ///> load map configuration
        param_handle.param("map/boundary/lower_x", _bdy[_BDY_L_X], _bdy[_BDY_L_X]);
        param_handle.param("map/boundary/upper_x", _bdy[_BDY_R_X], _bdy[_BDY_R_X]);
        param_handle.param("map/boundary/lower_y", _bdy[_BDY_L_Y], _bdy[_BDY_L_Y]);
        param_handle.param("map/boundary/upper_y", _bdy[_BDY_R_Y], _bdy[_BDY_R_Y]);
        param_handle.param("map/boundary/lower_z", _bdy[_BDY_L_Z], _bdy[_BDY_L_Z]);
        param_handle.param("map/boundary/upper_z", _bdy[_BDY_R_Z], _bdy[_BDY_R_Z]);
        param_handle.param("map/resolution", _map_resolution, _map_resolution);
        param_handle.param("map/safe_margin", _safe_margin, _safe_margin);

        this->_initMap();
    }

    { ///> load basic setting;

        // duration of regenerating
        double drt;
        param_handle.param("setting/regeneration_duration", drt, _reg_traj_drt.toSec());
        _reg_traj_drt = ros::Duration(drt);
        _reg_traj_tmr = crt_handle.createTimer(_reg_traj_drt, 
                &TrackingTrajectoryGenerator::regTrajectoryCallback, this, false, true);
        vector<double> dst;
        param_handle.getParam("setting/maintained_distance", dst);
        _kp_dst.resize(_TOT_DIM);
        _kp_dst(_DIM_X) = dst[_DIM_X];
        _kp_dst(_DIM_Y) = dst[_DIM_Y];
        _kp_dst(_DIM_Z) = dst[_DIM_Z];

        param_handle.param("setting/lambda", _crd_config.lambda, 0.01);

        // the degree of estimatied trajectory
        param_handle.param("estimation/trajectory_degree", _n_dgr_est, _n_dgr_est);
        param_handle.param("estimation/observation_size", _sz_tgt_obs, _sz_tgt_obs);
        param_handle.param("estimation/predition_time", _pdt_end, _pdt_end);
        _estimator = new TrackingTrajectory::TrajectoryEstimator(_n_dgr_est);

        // trajectory generator
        _generator = new VoxelTrajectory::TrajectoryGenerator;


        // visualization setting
        param_handle.param("visualization/flag", _flag_vis, _flag_vis);
        param_handle.param("visualization/corridor/color_rbga", _vis_crd_rbga);
        param_handle.param("visualization/map/duration", drt, _vis_map_drt.toSec());
        _vis_map_drt = ros::Duration(drt);
        _vis_map_tmr = crt_handle.createTimer(_vis_map_drt,
                &TrackingTrajectoryGenerator::visMapCallback, this, false, true);
    }
}

void TrackingTrajectoryGenerator::rcvTargetObservation(const geometry_msgs::PoseStamped &pose)
{
    VectorXd pos(_TOT_DIM);
    pos << 
        pose.pose.position.x, 
        pose.pose.position.y,
        pose.pose.position.z;
    _tgt_obs.push_back(make_pair(pose.header.stamp.toSec(), pos));
    while ((int)_tgt_obs.size() > _sz_tgt_obs) _tgt_obs.pop_front();
}

void TrackingTrajectoryGenerator::rcvCurrentOdometry(const nav_msgs::Odometry & odom)
{
    _odom = odom;
}

void TrackingTrajectoryGenerator::rcvGlobalPointCloud(const sensor_msgs::PointCloud & cloud)
{
    _map->insertBlocks(getStdVecFromPointCloud(cloud, _safe_margin));
}

void TrackingTrajectoryGenerator::rcvGlobalBlockCloud(const sensor_msgs::PointCloud & cloud)
{
    _map->insertBlocks(getStdVecFromBlockCloud(cloud, _safe_margin));
}

void TrackingTrajectoryGenerator::pubTrackingTrajectory()
{
    {// estimation of the trajectory
        TrackingTrajectory::TargetObservation obs(_tgt_obs.begin(), _tgt_obs.end());
        double beg_time = _odom.header.stamp.toSec();
        for (auto & pr: obs) pr.first -= beg_time;
        _crd_config.traj = _estimator->estimateTrajectory(obs);
        _crd_config.t_beg = _pdt_beg;
        _crd_config.t_end = _pdt_end;

        //ROS_WARN("[TRK_TRAJ] obs size = %d, prediction time = %lf \n", _sz_tgt_obs, _pdt_end);
        this->pubVisualTargetTrajectory();
        this->pubVisualTargetObservation();
        _crd_config.traj.row(0) += _kp_dst;
    }

    ROS_WARN("[TRK_TRAJ] going to find the corridor");
    {// generation of the flight corridor
        pair<Eigen::MatrixXd, Eigen::MatrixXd> flt_crd;
        Eigen::MatrixXd init_state(_TOT_STT, _TOT_DIM);
        if ((_odom.header.stamp - _traj_msg.header.stamp).toSec() < _pdt_beg)
        {
            init_state << 
                _odom.pose.pose.position.x, 
                _odom.pose.pose.position.y, 
                _odom.pose.pose.position.z,
                _odom.twist.twist.linear.x,
                _odom.twist.twist.linear.y,
                _odom.twist.twist.linear.z,
                0.0, 0.0, 0.0;
        } 
        else
        {
            init_state << getStateFromTrajByTime(
                    _traj_config.coef,
                    _traj_config.time,
                    (_odom.header.stamp - _traj_msg.header.stamp).toSec());
        }
        if (!_map->retPathFromTraj(
                    flt_crd, init_state.row(_STT_POS).transpose(), _crd_config.traj, _pdt_beg, _pdt_end))
        {
            ROS_WARN_STREAM("\n[TRAJECTORY_GENERATOR] There is no legal corridor.");
            return ;
        }

        ROS_WARN_STREAM("\n[TRK_TRAJ]" << flt_crd.first.rows() << ", " << flt_crd.first.cols() 
                << ", " << flt_crd.second.rows() << ", " << flt_crd.second.cols());
        ROS_WARN_STREAM("\n grid :\n" << flt_crd.first << "\nface:\n" << flt_crd.second);

        ROS_WARN("[TRK_TRAJ] corridor found!");
        // get the segment number
        _crd_config.M = flt_crd.first.rows();

        // get the inital and final state
        _crd_config.init_state = init_state;
        _crd_config.dest_state = getStateFromTrajByTime(_crd_config.traj, _pdt_end);

        // get the flight time
        _crd_config.traj_time.reserve(_crd_config.M);
        for (auto i = 0; i < _crd_config.M; ++i) 
        {
            _crd_config.traj_time.push_back(
                    make_pair(flt_crd.first(i, _BDY_L_T), flt_crd.first(i, _BDY_R_T)));
        }

        // load the corridor
        _crd_config.face = flt_crd.second;
        _crd_config.uninflated_grid = 
            flt_crd.first.block(_ROW_BEG, _BDY_BEG, _crd_config.M, _TOT_BDY);
        ROS_WARN("[TRK_TRAJ] corridor is going to be inflated!");
        _crd_config.grid = getInflatedCorridor(_map, 
            _crd_config.init_state.row(_STT_POS), 
            _crd_config.dest_state.row(_STT_POS),
            _crd_config.uninflated_grid,
            _crd_config.face);
        ROS_WARN_STREAM("[TRK_TRAJ] corridor inflated! \n" << _crd_config.grid);

        this->pubVisualTargetCorridor();
    }


    {// generation of the flight trajectory
        ROS_WARN("[TRK_TRAJ] going to generate tracking traj!");
        auto tmp_config = _generator->getTrackingTrajectory(_crd_config);

        if (tmp_config.error_code)
        {
            ROS_WARN("\n[TRAJECTORY_GENERATOR] Fail to generate the trajectory.");
            return ;
        }
        swap(tmp_config, _traj_config);
        
        _traj_msg = getTrajMsgByMatrix(
                _traj_id, 
                quadrotor_msgs::PolynomialTrajectory::ACTION_ADD,
                _traj_config.coef,
                _traj_config.time,
                _odom.header.stamp);

        _traj_pub.publish(_traj_msg);

        this->pubVisualTrackingTrajectory();
    }
    return ;
}

void TrackingTrajectoryGenerator::pubVisualMapGrids()
{
    if (!_flag_vis) return ;
    _vis_map_pub.publish(getPointCloudFromStdVec(_map->getPointCloud()));
}

void TrackingTrajectoryGenerator::pubVisualTrackingTrajectory()
{
    if (!_flag_vis) return ;
    _vis_trk_pub.publish(getPointCloudFromTraj(_traj_config.coef, _traj_config.time));
}

void TrackingTrajectoryGenerator::pubVisualTargetTrajectory()
{
    if (!_flag_vis) return ;
    _vis_est_pub.publish(getPointCloudFromTraj(_crd_config.traj, _pdt_end));
}

void TrackingTrajectoryGenerator::pubVisualTargetObservation()
{
    if (!_flag_vis) return ;
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "/map";
    cloud.header.stamp = ros::Time::now();

    cloud.points.reserve(_tgt_obs.size());
    geometry_msgs::Point32 pt;
    for (auto ob: _tgt_obs) 
    {
        pt.x = ob.second(_DIM_X);
        pt.y = ob.second(_DIM_Y);
        pt.z = ob.second(_DIM_Z);
        cloud.points.push_back(pt);
    }

    _vis_obs_pub.publish(cloud);
}

void TrackingTrajectoryGenerator::pubVisualTargetCorridor()
{
    if (!_flag_vis) return ;
    for (auto & mk: _crd_msg.markers) mk.action = visualization_msgs::Marker::DELETE;
    _vis_crd_pub.publish(_crd_msg);
    _crd_msg = getCorridorMsgByMatrix(
                _crd_config.uninflated_grid, "tracking_trajectory/corridor", _vis_crd_rbga);
    _vis_crd_pub.publish(_crd_msg);
}

void TrackingTrajectoryGenerator::regTrajectoryCallback(const ros::TimerEvent & evt)
{
    pubTrackingTrajectory();
}
    
void TrackingTrajectoryGenerator::visMapCallback(const ros::TimerEvent & evt)
{
    pubVisualMapGrids();
}

