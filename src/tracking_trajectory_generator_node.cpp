#include "macro.h"
#include "traj_utils.h"
#include "target_trajectory_estimator.h"
#include "octomap.h"
#include "trajectory_generator.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
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
#include <sstream>

using namespace std;
using namespace TrackingTrajectory;


class TrackingTrajectoryGenerator
{
    // todo/issue:
    //  1. start from random position. (done)
    //  2. command to track and stop. (waiting for the actual needs)
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
    //  21. distance between target. (done)
    //  22. regeneration. (maybe not, there has been cycle-time generation.)
    //  23. laser message. (done, to be tested) 
    //  24. tuning the parameter. (1) lambda (2) estimation degree (3) prediction time
    //  25. penalty derivative on the estimation. (done with regulator)
    //  26. move to mosek solver. (done, to be tested) 
    //  27. bug of nan. (fixed)
    //  28. more error code from the solver. (done)
    //  29. advanced virtual commander. (done)
    //  30. camera calibration plus tag! (producer done, receiver not yet)
    //  31. incremental map visualization.
    //  32. yaw angle control. (done)
    //  33. bag with virual obstacle. (done)
    //  34. only trust the estimation when we have enough observation. (done)
    //  35. pop the old observation away. (done)
    //  36. bug: multi-starts A* with no starts points. (temporatory solved.)
    //  37. feature: allow infeasible end.
    //  38. bug: if diff of odom stamp and traj stamp. (done)
    //  39. visulize the history trajectory. (done)
    //  40. simulated laser
    //  41. improved target observer: obstacle aware, field of view. 
    //  42. stop trajectory if obstacle within the old trajectory. (done)

public:
    ///> the intialization!
    TrackingTrajectoryGenerator(ros::NodeHandle & handle);

    ///> callback function for receiving different messages 
    void rcvCurrentOdometry(const nav_msgs::Odometry & odom);
    void rcvTargetObservation(const geometry_msgs::PoseStamped & pose);
    void rcvTagObservation(const geometry_msgs::PoseStamped & pose);
    void rcvGlobalPointCloud(const sensor_msgs::PointCloud & cloud);
    void rcvGlobalBlockCloud(const sensor_msgs::PointCloud & cloud);
    void rcvLocalLaserScan(const sensor_msgs::LaserScan & scan);

    ///> the delievery of resulting trajectory
    void pubTrackingTrajectory();
    ///> the visual data 
    void pubVisualTargetTrajectory();
    void pubVisualTargetCorridor();
    void pubVisualMapGrids();
    void pubVisualTrackingTrajectory();
    void pubVisualTargetObservation();
    void pubVisualDesiredTrajectory();

    ///> cycled actions
    void regTrajectoryCallback(const ros::TimerEvent & evt);
    void visMapCallback(const ros::TimerEvent & evt);  
private:

    bool _is_simulation = true;

    uint32_t _traj_id = 1; // trajectory id
    bool _flag_vis = true; // switch of the visualization

    ///> io-channel
    ros::Subscriber _cmd_sub;
    ros::Subscriber _odom_sub;  // odometry
    ros::Subscriber _blks_sub;  // map blocks cloud
    ros::Subscriber _pts_sub;   // map points cloud
    ros::Subscriber _obs_sub;   // the target observation
    ros::Subscriber _tag_sub;
    ros::Subscriber _scan_sub;

    ros::Publisher _traj_pub;   // flight trajectory 
    ros::Publisher _yaw_pub;
    ros::Publisher _vis_est_pub; // visual target trajectory
    ros::Publisher _vis_trk_pub; // visual tracking trajectory
    ros::Publisher _vis_map_pub; // visual map
    ros::Publisher _vis_crd_pub; // visual corridor
    ros::Publisher _vis_obs_pub; // visual target observation 
    ros::Publisher _vis_dsd_pub; // visual desired trajectory
    ros::Publisher _vis_hst_pub; // visual history trajectory
    ros::Publisher _vis_blk_pub;
    ros::Publisher _vis_scan_pub;
    
    ros::Publisher _debug_pub; // publish debug & running info

     ///> current innner states infomation
    nav_msgs::Odometry _odom;
    size_t _odom_queue_size = 300;                              // odometry queue size
    deque<nav_msgs::Odometry> _odom_queue;                      // odometry queue for delayed observations

    ///> sub-module
  
    // info about the target trajectory estimation
    TrackingTrajectory::TrajectoryEstimator * _estimator = NULL;

    deque<pair<double, Eigen::VectorXd> > _tgt_obs;             // observations
    int _sz_tgt_obs = 200;                                      // observations size
    double _pdt_end = 5.0, _pdt_beg = 0.0;                      // prediction time
    int _n_dgr_est = 4;                                         // estimation traj order
    visualization_msgs::MarkerArray _crd_msg;                   // corridor vis message
    geometry_msgs::Pose _pose_cam_bd;
    ros::Duration _cam_drt = ros::Duration(0.001);
    ros::Time _cam_stp = ros::TIME_MIN;

    int _obs_valid_num  = 20;
    double _obs_rsv_drt = 5.0;
    double _valid_lost_drt = 1.0;
    

    bool is_fixed_height = true;

    // info about map
    VoxelTrajectory::OctoMap * _map = NULL;

    ros::Timer _vis_map_tmr; 
    ros::Duration _vis_map_drt = ros::Duration(1.0);            // cycle time of visualizing map
    vector<double> _vis_crd_rbga = {0.1, 0.0, 0.7, 0.1};        // color of visual corridor 

    vector<double> _bdy {-100.0, 100.0, -100.0, 100.0, 0.0, 200.0}; // boundary of the environment
    double _map_resolution = 0.2 * 0.2 * 0.2;                       // map grid resolution
    double _safe_margin = 0.2;                                      // safe margin adding to obstacle

    // laser message
    double _laser_resolution = 0.1;                                 // laser points resolution
    int _laser_count_thld = 3;                                      // threshold filtering outliers in laser
    double _laser_height_thld = 0.2;                                // threshold filtering ground points in laser
    double _laser_extra_height = 2.5;                               // adding extra heigh to generate pillar(2D laser)
    ros::Duration _laser_drt = ros::Duration(0.5);                  // cycle time of adding laser scan
    ros::Time _laser_stp = ros::TIME_MIN;                           // last stamp of adding laser scan

    // info about the tracking trajectory generation
    VoxelTrajectory::TrajectoryGenerator * _generator = NULL;
    VoxelTrajectory::TrajectoryGenerator::TrackingCorridorConfig _crd_config;   // config of corridor (for trajectory)
    VoxelTrajectory::TrajectoryGenerator::TrajectoryConfig _traj_config;        // config of generated trajectory

    ros::Timer _reg_traj_tmr;                                                   
    ros::Duration _reg_traj_drt = ros::Duration(0.25);                           // cycletime of regenration

    int _n_dgr_traj = 6;                                        // degree of tracking traj
    int _n_dgr_min = 3;                                         // degree of derivative to minimize
    double _max_vel = 1.0, _max_acc = 1.0;                      // dynamic limitations
    double _flt_vel = 1.0, _flt_acc = 1.0;                      // dynamic data for time allocation (not used for this traj)
    double _check_dt = 0.03;                                    // time step for check obstable along the traj
    Eigen::RowVectorXd _kp_dst;                                 // the distance vector to the target

    quadrotor_msgs::PolynomialTrajectory _traj_msg;
    sensor_msgs::PointCloud _hist_traj_msg;
    int _hist_traj_count = 0, _hist_traj_iterator = 0;

    ///> internal mechanism
    void _initMap();

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
        _tag_sub = msg_handle.subscribe("tag_pose_in_camera", 
                10, &TrackingTrajectoryGenerator::rcvTagObservation, this);
        _scan_sub = msg_handle.subscribe("laser_scan",
                10, &TrackingTrajectoryGenerator::rcvLocalLaserScan, this);

        _traj_pub = msg_handle.advertise<quadrotor_msgs::PolynomialTrajectory>(
                "flight_trajectory", 10);
        _yaw_pub = msg_handle.advertise<std_msgs::Float64>(
                "desired_yaw", 50);
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
        _vis_dsd_pub = msg_handle.advertise<sensor_msgs::PointCloud>(
                "visual_desired_trajectory", 10);
        _vis_hst_pub = msg_handle.advertise<sensor_msgs::PointCloud>(
                "visual_history_trajectory", 10);
        _vis_blk_pub = msg_handle.advertise<sensor_msgs::PointCloud>(
                "visual_blocks", 10);
        _debug_pub = msg_handle.advertise<std_msgs::String>(
                "debug_info", 50);
        _vis_scan_pub = msg_handle.advertise<sensor_msgs::LaserScan>(
            "visual_scan", 10);
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
        param_handle.param("setting/is_simulation", _is_simulation, _is_simulation);
        double drt;
        param_handle.param("setting/regeneration_duration", drt, _reg_traj_drt.toSec());
        _reg_traj_drt = ros::Duration(drt);
        _reg_traj_tmr = crt_handle.createTimer(_reg_traj_drt, 
                &TrackingTrajectoryGenerator::regTrajectoryCallback, this, false, true);
        vector<double> dst;
        param_handle.param("setting/is_fixed_height", is_fixed_height, is_fixed_height);
        param_handle.getParam("setting/maintained_distance", dst);
        _kp_dst.resize(_TOT_DIM);
        _kp_dst(_DIM_X) = dst[_DIM_X];
        _kp_dst(_DIM_Y) = dst[_DIM_Y];
        _kp_dst(_DIM_Z) = dst[_DIM_Z];

        param_handle.param("setting/lambda", _crd_config.lambda, 0.01);
        param_handle.param("setting/use_stop_policy", _crd_config.use_stop_policy, false);

        param_handle.param("setting/laser/resolution", _laser_resolution, _laser_resolution);
        param_handle.param("setting/laser/count_threshold", _laser_count_thld, _laser_count_thld);
        param_handle.param("setting/laser/laser_height_threshold", _laser_height_thld, _laser_height_thld);
        param_handle.param("setting/laser/extra_height", _laser_extra_height, _laser_extra_height);
        param_handle.param("setting/laser/cycle_duration", drt, _laser_drt.toSec());
        _laser_drt = ros::Duration(drt);

        // camera setting
        vector<double> pos, ort;
        param_handle.getParam("setting/camera/position", pos);
        param_handle.getParam("setting/camera/orientation", ort);
        param_handle.param("setting/camera/cycle_duration", drt, _cam_drt.toSec());
        _pose_cam_bd.position.x = pos[_DIM_X];
        _pose_cam_bd.position.y = pos[_DIM_Y];
        _pose_cam_bd.position.z = pos[_DIM_Z];
        _pose_cam_bd.orientation.w = ort[0]; 
        _pose_cam_bd.orientation.x = ort[1]; 
        _pose_cam_bd.orientation.y = ort[2]; 
        _pose_cam_bd.orientation.z = ort[3]; 
        ROS_WARN("[camera_translation] %lf, %lf, %lf", pos[0], pos[1], pos[2]);
        _cam_drt = ros::Duration(drt);

        // the degree of estimatied trajectory
        param_handle.param("estimation/trajectory_degree", _n_dgr_est, _n_dgr_est);
        param_handle.param("estimation/observation_size", _sz_tgt_obs, _sz_tgt_obs);
        param_handle.param("estimation/valid_size", _obs_valid_num, _obs_valid_num);
        param_handle.param("estimation/reserved_duration", _obs_rsv_drt, _obs_rsv_drt);
        param_handle.param("estimation/predition_time", _pdt_end, _pdt_end);
        param_handle.param("estimation/valid_lost_duration", _valid_lost_drt, _valid_lost_drt);
        double lambda = 0.01;
        param_handle.param("estimation/lambda", lambda, lambda);
        _estimator = new TrackingTrajectory::TrajectoryEstimator(_n_dgr_est);
        _estimator->setLambda(lambda);
        _estimator->setPredictionTime(_pdt_end);

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

    { // private variable initialization
        _traj_msg.header.stamp = ros::TIME_MIN; //used for judging the initial state
        _traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;

        _hist_traj_msg.header.frame_id = "/map";
        _hist_traj_msg.header.stamp = ros::Time::now();
    }
}

void TrackingTrajectoryGenerator::rcvTargetObservation(const geometry_msgs::PoseStamped &pose)
{
#if 1
    { // when observed target, control yaw accordingly
        std_msgs::Float64 yaw;
        double y = pose.pose.position.y - _odom.pose.pose.position.y;
        double x = pose.pose.position.x - _odom.pose.pose.position.x;
        if (abs(x) + abs(y) > 0.2)
        {
            yaw.data = atan2(y, x);
            _yaw_pub.publish(yaw);
        }
    }
#endif
    {
        VectorXd pos(_TOT_DIM);
        pos << 
            pose.pose.position.x, 
            pose.pose.position.y,
            pose.pose.position.z;
        _tgt_obs.push_back(make_pair(pose.header.stamp.toSec(), pos));
        while ((int)_tgt_obs.size() > _sz_tgt_obs) _tgt_obs.pop_front();
    }
}

void TrackingTrajectoryGenerator::rcvCurrentOdometry(const nav_msgs::Odometry & odom)
{
    _odom = odom;
    
    // store odometry for obsertion on the body frame
    _odom_queue.push_back(odom);
    while (_odom_queue.size() > _odom_queue_size) _odom_queue.pop_front();

    // abort all target observation if lost for a period
    if ((!_tgt_obs.empty()) &&
            (odom.header.stamp.toSec() - _tgt_obs.back().first) > _valid_lost_drt)
        _tgt_obs.clear();

    if (!_is_simulation)
    {
        // abort old old observation
        while (!_tgt_obs.empty() && 
            _tgt_obs.front().first + _obs_rsv_drt < odom.header.stamp.toSec()) _tgt_obs.pop_front();
    }


    // publish historical trajetory
    if (_flag_vis)
    {
        if (_hist_traj_count) 
            _hist_traj_count -= 1;
        else
        {
            _hist_traj_count = 10;
            geometry_msgs::Point32 p;
            p.x = odom.pose.pose.position.x;
            p.y = odom.pose.pose.position.y;
            p.z = odom.pose.pose.position.z;
            if (_hist_traj_msg.points.size() < 600) 
                _hist_traj_msg.points.push_back(p);
            else
                _hist_traj_msg.points[_hist_traj_iterator] = p;
            _hist_traj_iterator += 1;
            _hist_traj_iterator %= 600;
            _vis_hst_pub.publish(_hist_traj_msg);
        }
    }
    
    // if the taget is out of view, control yaw by predicted position
    if (    _odom.header.stamp.toSec() > _crd_config.stamp + _reg_traj_drt.toSec() * 1.5 &&
            _odom.header.stamp.toSec() < _crd_config.stamp + _crd_config.t_end)
    {
        auto state = getStateFromTrajByTime(_crd_config.traj, 
                _odom.header.stamp.toSec() - _crd_config.stamp);
        std_msgs::Float64 yaw;
        double y = state(_STT_POS, _DIM_Y) - _kp_dst[_DIM_Y] - _odom.pose.pose.position.y;
        double x = state(_STT_POS, _DIM_X) - _kp_dst[_DIM_X] - _odom.pose.pose.position.x;
        if (abs(x) + abs(y) > 0.05)
        {
            yaw.data = atan2(y, x);
            _yaw_pub.publish(yaw);
        }
    }
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
    ros::Duration est_drt, crd_drt, ift_drt, trj_drt;

    // if not enough observation, use existing predicted trajecotry, 
    if ((int)_tgt_obs.size() > _obs_valid_num)
    {// estimation of the trajectory
        TrackingTrajectory::TargetObservation obs(_tgt_obs.begin(), _tgt_obs.end());
        if (obs.empty()) return ;

        double beg_time = (_is_simulation) ? obs.back().first : _odom.header.stamp.toSec();

        // move to the relative time
        for (auto & pr: obs) pr.first -= beg_time;

        // fixed height 
        if (is_fixed_height) for (auto & pr: obs) pr.second(_DIM_Z) = 0.0;

        {// kernel call.
            ros::Time pre_est_stamp = ros::Time::now();
            _crd_config.traj = _estimator->estimateTrajectory(obs);
            est_drt = ros::Time::now() - pre_est_stamp;
        }
        _crd_config.t_beg = _pdt_beg;
        _crd_config.t_end = _pdt_end;
        _crd_config.stamp = _odom.header.stamp.toSec();

        //ROS_WARN("[TRK_TRAJ] obs size = %d, prediction time = %lf \n", _sz_tgt_obs, _pdt_end);
        this->pubVisualTargetTrajectory();
        // move the predicted trajectory to get desired trajectory
        _crd_config.traj.row(0) += _kp_dst;
        this->pubVisualDesiredTrajectory();
        this->pubVisualTargetObservation();
    }
    else
    {
#if 0
        ROS_WARN_STREAM("[ESTIMATION] use existing estimation!!!!!!!!!!!!!");
        ROS_WARN_STREAM("[ESTIMATION] time window = " <<
                _odom.header.stamp.toSec() - _crd_config.stamp <<  ", " <<
                _crd_config.t_end);
#endif
        // if the predicting is overtime, just abort it.
        if (_odom_queue.empty()) return ;
        if (_odom.header.stamp.toSec() > _crd_config.stamp + _crd_config.t_end) return ;
        // moving the prediction accordingly
        _crd_config.moveForward(_odom.header.stamp.toSec() - _crd_config.stamp);
        this->pubVisualDesiredTrajectory();
    }
    //ROS_WARN_STREAM("[TRK_TRAJ] estimated traj:\n" << _crd_config.traj);

    //ROS_WARN("[TRK_TRAJ] going to find the corridor");
    // allowed time end variable will be used if the corridor tail is obstacled.
    double allowed_t_end = _crd_config.t_end;
    {// generation of the flight corridor
        pair<Eigen::MatrixXd, Eigen::MatrixXd> flt_crd;
        Eigen::MatrixXd init_state(_TOT_STT, _TOT_DIM);

        // generate the inital state 
        if (_traj_msg.action != quadrotor_msgs::PolynomialTrajectory::ACTION_ADD
                ||(_odom.header.stamp - _traj_msg.header.stamp).toSec() > _crd_config.t_end)
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
        { // desired state can provied expected acceleration, so preferred
            init_state << getStateFromTrajByTime(
                    _traj_config.coef,
                    _traj_config.time,
                    (_odom.header.stamp - _traj_msg.header.stamp).toSec() + 0.03);
        }
        { // the kernel call
            ros::Time pre_crd_stamp = ros::Time::now();
            if (!_map->retPathFromTraj(
                        flt_crd, init_state.row(_STT_POS).transpose(), _crd_config.traj, 
                        0.0, _crd_config.t_end, allowed_t_end))
            {
                ROS_WARN_STREAM("\n[TRAJECTORY_GENERATOR] There is no legal corridor.");
                return ;
            }
            crd_drt = ros::Time::now() - pre_crd_stamp;
        }


       // ROS_WARN_STREAM("\n[TRK_TRAJ]" << flt_crd.first.rows() << ", " << flt_crd.first.cols() 
       //         << ", " << flt_crd.second.rows() << ", " << flt_crd.second.cols());
       // ROS_WARN_STREAM("\n grid :\n" << flt_crd.first << "\nface:\n" << flt_crd.second);

        //ROS_WARN("[TRK_TRAJ] corridor found!");
        // get the segment number
        _crd_config.M = flt_crd.first.rows();

        // get the inital and final state
        _crd_config.init_state = init_state;
        _crd_config.dest_state = getStateFromTrajByTime(_crd_config.traj, _crd_config.t_end);
        _crd_config.t_end = min(allowed_t_end, _crd_config.t_end);

        // load the corridor
        _crd_config.face = flt_crd.second;
        _crd_config.uninflated_grid = 
            flt_crd.first.block(_ROW_BEG, _BDY_BEG, _crd_config.M, _TOT_BDY);
        //ROS_WARN("[TRK_TRAJ] corridor is going to be inflated!");
        { // call the kernel
            ros::Time pre_ift_stamp = ros::Time::now();
            _crd_config.grid = getInflatedCorridor(_map, 
                _crd_config.init_state.row(_STT_POS), 
                _crd_config.dest_state.row(_STT_POS),
                _crd_config.uninflated_grid,
                _crd_config.face);
            ift_drt = ros::Time::now() - pre_ift_stamp;
        }
        //ROS_WARN_STREAM("[TRK_TRAJ] corridor inflated! \n" << _crd_config.grid);

        this->pubVisualTargetCorridor();
    }


    {// generation of the flight trajectory
        //ROS_WARN("[TRK_TRAJ] going to generate tracking traj!");
        { // call the kernel
            ros::Time pre_trj_stamp = ros::Time::now();
            auto tmp_config = _generator->getTrackingTrajectory(_crd_config);
            trj_drt = ros::Time::now() - pre_trj_stamp;

            if (tmp_config.error_code)
            {
                ROS_WARN("\n[TRAJECTORY_GENERATOR] Fail to generate the trajectory.");
                return ;
            }
            swap(tmp_config, _traj_config);
        }
        
        _traj_msg = getTrajMsgByMatrix(
                _traj_id, 
                quadrotor_msgs::PolynomialTrajectory::ACTION_ADD,
                _traj_config.coef,
                _traj_config.time,
                _odom.header.stamp);

        _traj_pub.publish(_traj_msg);

        this->pubVisualTrackingTrajectory();
    }

    {// publish the runtime info if trajectory is generated
        stringstream str_in;
        str_in << "The duration: " << 
            est_drt.toSec() << ", " << 
            crd_drt.toSec() << ", " << 
            ift_drt.toSec() << ", " << 
            trj_drt.toSec() << ".";
        std_msgs::String str_msg;
        str_msg.data = str_in.str();
        _debug_pub.publish(str_msg);
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
    _vis_est_pub.publish(getPointCloudFromTraj(_crd_config.traj, _crd_config.t_end));
}

void TrackingTrajectoryGenerator::pubVisualDesiredTrajectory()
{
    if (!_flag_vis) return ;
    _vis_dsd_pub.publish(getPointCloudFromTraj(_crd_config.traj, _crd_config.t_end));
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
    bool flag_abort = false;
    {// check obstacle along the old trajctory
        if (_traj_msg.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD) 
        {
            double _t_check_beg = (_odom.header.stamp - _traj_msg.header.stamp).toSec();
            double _t_check_end = _traj_config.time.sum();
            for (double t = _t_check_beg; t <= _t_check_end; t+= _check_dt)
            {
                auto state = getStateFromTrajByTime(
                        _traj_config.coef, _traj_config.time, t);
                vector<double> pos {
                    state(_STT_POS, _DIM_X), 
                    state(_STT_POS, _DIM_Y), 
                    state(_STT_POS, _DIM_Z)}; 
                if (_map->testObstacle(pos.data()))
                {
                    _traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;
                    flag_abort = true;
                    ROS_WARN_STREAM("[OBSTACLE_CHECK]The old trajectory is doomed!");
                    ROS_WARN_STREAM("[TRAJ_ACT] " << 
                        quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT);
                    break;
                }
            }
        }
    }
    pubTrackingTrajectory();
#if 1
    // no new obstacle, and the old is aborted
    if (flag_abort && _traj_msg.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT)
        _traj_pub.publish(_traj_msg);
#endif
}
    
void TrackingTrajectoryGenerator::visMapCallback(const ros::TimerEvent & evt)
{
    pubVisualMapGrids();
}

void TrackingTrajectoryGenerator::rcvLocalLaserScan(const sensor_msgs::LaserScan & scan)
{
    if (_odom_queue.empty()) return ;
    if (scan.header.stamp - _laser_stp < _laser_drt) return ;
    _laser_stp = scan.header.stamp;

    nav_msgs::Odometry laser_odom = _odom_queue.back();
    for (auto & odom: _odom_queue)
       if (odom.header.stamp >= _laser_stp)
       {
           laser_odom = odom;
           break;
       } 
    sensor_msgs::LaserScan _scan = scan;
    _scan.header.frame_id = "/map";
    _vis_scan_pub.publish(_scan);

    auto blk = getStdVecFromLaserScan(scan, laser_odom,
           _safe_margin, _laser_resolution, _laser_count_thld,
           _laser_height_thld, _laser_extra_height);

    {
        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = "/map";
        cloud.header.stamp = ros::Time::now();
        for (size_t idx = 0; idx * _TOT_BDY < blk.size(); ++idx)
        {
            geometry_msgs::Point32 p;
            p.x = 0.5 * (blk[idx * _TOT_BDY + _BDY_L_X] + blk[idx * _TOT_BDY + _BDY_R_X]);
            p.y = 0.5 * (blk[idx * _TOT_BDY + _BDY_L_Y] + blk[idx * _TOT_BDY + _BDY_R_Y]);
            p.z = 0.5 * (blk[idx * _TOT_BDY + _BDY_L_Z] + blk[idx * _TOT_BDY + _BDY_R_Z]);
            cloud.points.push_back(p);
        }
        _vis_blk_pub.publish(cloud);
    }
    //ROS_WARN("[LASER] %d number of points will be added.", (int)blk.size()/6);

    _map->insertBlocks(blk);
}

void TrackingTrajectoryGenerator::rcvTagObservation(const geometry_msgs::PoseStamped & pose)
{
    if (_odom_queue.empty()) return ;
    if (pose.header.stamp - _cam_stp < _cam_drt) return ;
    _cam_stp = pose.header.stamp;

    nav_msgs::Odometry cam_odom = _odom_queue.back();
    for (auto & odom: _odom_queue)
        if (odom.header.stamp > _cam_stp)
        {
            cam_odom = odom;
            break;
        }

    Eigen::VectorXd pt = getWorldPositionFromCam(
            pose.pose, _pose_cam_bd, cam_odom.pose.pose);

    _tgt_obs.push_back(make_pair(_cam_stp.toSec(), pt));

    while (!_tgt_obs.empty() && ((int)_tgt_obs.size() > _sz_tgt_obs))
        _tgt_obs.pop_front();
}
