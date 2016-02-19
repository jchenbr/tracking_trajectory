#ifndef _TRACKING_TRAJECTORY_TRAJ_UTILS_H_
#define _TRACKING_TRAJECTORY_TRAJ_UTILS_H_

#include "macro.h"
#include "target_trajectory_estimator.h"
#include "octomap.h"
#include "trajectory_generator.h"

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <eigen3/Eigen/Dense>
#include <assert.h>
using namespace std;
using namespace Eigen;
using namespace TrackingTrajectory;

///> Here are the declarations.
/* the declaration of methods */

Eigen::MatrixXd getInflatedCorridor(
        VoxelTrajectory::OctoMap * voxel_map,
        const Eigen::MatrixXd & path);

Eigen::MatrixXd getStateFromTrajByTime(
        const Eigen::MatrixXd & coef,
        const Eigen::VectorXd & time,
        double t);

Eigen::MatrixXd getStateFromTrajByTime(
        const Eigen::MatrixXd & coef,
        double t);

Eigen::Vector3d getWorldPositionFromCam(
        const geometry_msgs::Pose & pose_tgt_cam,
        const geometry_msgs::Pose & pose_cam_bd,
        const geometry_msgs::Pose & pose_bd_wd);
        
geometry_msgs::Pose getWorldPoseMsgFromCam(
        const geometry_msgs::Pose & pose_tgt_cam,
        const geometry_msgs::Pose & pose_cam_bd,
        const geometry_msgs::Pose & pose_bd_wd);

/* the declaration of message transform */

quadrotor_msgs::PolynomialTrajectory getTrajMsgByMatrix(
        uint32_t & id,
        uint32_t action,
        const Eigen::MatrixXd & coef,
        const Eigen::VectorXd & time,
        ros::Time stamp,
        string frame_id = "/map");

visualization_msgs::MarkerArray getCorridorMsgByMatrix(
        const Eigen::MatrixXd & grid,
        const string & name_space,
        const vector<double> &color);

sensor_msgs::PointCloud getPointCloudFromTraj(
        const Eigen::MatrixXd &traj,
        const Eigen::VectorXd &time,
        string frame_id = "/map",
        double d_t = 0.01);

sensor_msgs::PointCloud getPointCloudFromStdVec(
        const vector<double> & pt,
        string frame_id = "/map");

vector<double> getStdVecFromBlockCloud(
        const sensor_msgs::PointCloud &cloud,
        double margin = _EPS);

vector<double> getStdVecFromPointCloud(
        const sensor_msgs::PointCloud &cloud,
        double margin = _EPS);

vector<double> getStdVecFromLaserScan(
        const sensor_msgs::LaserScan & scan,
        const nav_msgs::Odometry & odom,
        double margin = _EPS,
        double resolution = _EPS,
        int count_thld = 0,
        double height_thld = _EPS,
        double extra_height = _EPS);
///> Here are the implementations.

visualization_msgs::MarkerArray getCorridorMsgByMatrix(
        const Eigen::MatrixXd & grid,
        const string & name_space,
        const vector<double> &color)
{
    visualization_msgs::MarkerArray ret;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "/map";
    mk.header.stamp = ros::Time::now();

    mk.ns = name_space;
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.color.r = color[0];
    mk.color.b = color[1];
    mk.color.g = color[2];
    mk.color.a = color[3];

    for (int idx = 0; idx < grid.rows(); ++idx)
    {
        mk.id = idx;
        mk.pose.position.x = (grid(idx, _BDY_L_X) + grid(idx, _BDY_R_X)) * 0.5;
        mk.pose.position.y = (grid(idx, _BDY_L_Y) + grid(idx, _BDY_R_Y)) * 0.5;
        mk.pose.position.z = (grid(idx, _BDY_L_Z) + grid(idx, _BDY_R_Z)) * 0.5;
        mk.scale.x = grid(idx, _BDY_R_X) - grid(idx, _BDY_L_X);
        mk.scale.y = grid(idx, _BDY_R_Y) - grid(idx, _BDY_L_Y);
        mk.scale.z = grid(idx, _BDY_R_Z) - grid(idx, _BDY_L_Z);

        ret.markers.push_back(mk);
    }
    return ret;
}

Eigen::MatrixXd getInflatedCorridor(
        VoxelTrajectory::OctoMap * voxel_map,
        const Eigen::VectorXd &init_pos,
        const Eigen::VectorXd &dest_pos,
        const Eigen::MatrixXd &grid,
        const Eigen::MatrixXd &face)
{
    MatrixXd tmp(grid.rows() << 1, _TOT_BDY);
    tmp << init_pos.transpose(), dest_pos.transpose(), grid, face;
    return getInflatedCorridor(voxel_map, tmp);
}

Eigen::MatrixXd getInflatedCorridor(
        VoxelTrajectory::OctoMap * voxel_map,
        const Eigen::MatrixXd & path)
{
    int m = path.rows() >> 1;
    Eigen::MatrixXd inflated_path = path.block(1, 0, m, path.cols());
#if 1
    auto within = [&](double pt[_TOT_DIM], double bdy[_TOT_BDY])
    {
        for (int dim = 0; dim < _TOT_BDY; ++dim)
        {
            if (abs(pt[dim >> 1] - bdy[dim]) < _EPS) return dim;
        }
        return 0;
        //clog << "!" << endl;
    };

    for (int iRow = 0; iRow < m; ++iRow)
    {
        int direction[_TOT_BDY] = {-1, 1, -1, 1, -1, 1};
        int neighbor[_TOT_BDY] = {0, 0, 0, 0, 0, 0};

        // original grid
        double bdy[_TOT_BDY] = 
        {
            inflated_path(iRow, _BDY_L_X), inflated_path(iRow, _BDY_R_X),
            inflated_path(iRow, _BDY_L_Y), inflated_path(iRow, _BDY_R_Y),
            inflated_path(iRow, _BDY_L_Z), inflated_path(iRow, _BDY_R_Z)
        };


        //clog << "[VOXEL_SERVER] iRow = " << iRow << endl; 
        if (m > 1)
        {
            if (iRow == 0)
            {
                double pt[_TOT_DIM] =  
                {
                    (path(m + iRow + 1, _BDY_L_X) + path(m + iRow + 1, _BDY_R_X)) * 0.5,
                    (path(m + iRow + 1, _BDY_L_Y) + path(m + iRow + 1, _BDY_R_Y)) * 0.5,
                    (path(m + iRow + 1, _BDY_L_Z) + path(m + iRow + 1, _BDY_R_Z)) * 0.5
                };
                //clog << " ? " << endl;
                neighbor[within(pt, bdy)] = 1;
            }
            else if (iRow + 1 == m)
            {
                double pt[_TOT_DIM] = 
                {
                    (path(m + iRow, _BDY_L_X) + path(m + iRow, _BDY_R_X)) * 0.5,
                    (path(m + iRow, _BDY_L_Y) + path(m + iRow, _BDY_R_Y)) * 0.5,
                    (path(m + iRow, _BDY_L_Z) + path(m + iRow, _BDY_R_Z)) * 0.5
                };
                neighbor[within(pt, bdy)] = 1;
            }
            else
            {
                double pt[_TOT_DIM + _TOT_DIM] = 
                {
                    (path(m + iRow, _BDY_L_X) + path(m + iRow, _BDY_R_X)) * 0.5,
                    (path(m + iRow, _BDY_L_Y) + path(m + iRow, _BDY_R_Y)) * 0.5,
                    (path(m + iRow, _BDY_L_Z) + path(m + iRow, _BDY_R_Z)) * 0.5,

                    (path(m + iRow + 1, _BDY_L_X) + path(m + iRow + 1, _BDY_R_X)) * 0.5,
                    (path(m + iRow + 1, _BDY_L_Y) + path(m + iRow + 1, _BDY_R_Y)) * 0.5,
                    (path(m + iRow + 1, _BDY_L_Z) + path(m + iRow + 1, _BDY_R_Z)) * 0.5
                };
                neighbor[within(pt, bdy)] = 1;
                neighbor[within(pt + _TOT_DIM, bdy)] = 1;
            }
        }

        //clog << "[VOXEL_SERVER] preparation done." << endl;
        // #1. inflate towards all direction inflation;
        voxel_map->inflateBdy(bdy, direction);
        //clog << "[VOXEL_SERVER] all direction inflation done." << endl;

        // #2. inflate towards labours;
        
        for (int dim = 0; dim < _TOT_BDY; ++dim)
        {
            neighbor[dim] *= direction[dim];
            //clog << neighbor[dim] << " ";
        }
        //clog << endl;

        voxel_map->inflateBdy(bdy, neighbor, 20);

        for (int dim = 0; dim < _TOT_BDY; ++dim)
            if (neighbor[dim] != 0)
            {
                neighbor[dim] = 0;
                voxel_map->inflateBdy(bdy, neighbor, 20);
                neighbor[dim] = (dim & 1) ? 1 : -1;
            }
        

        //clog << "[VOXEL_SERVER] all direction inflation done." << endl;
        // #3. inflate towards one direction each time;
        for (int drc = 0; drc < _TOT_BDY; ++drc)
        {
            if (neighbor[drc] != 0) continue;
            memset(direction, 0, sizeof(direction));
            direction[drc] = (drc & 1) ? 1 : -1;
            voxel_map->inflateBdy(bdy, direction, 20);
        }

        //clog << "[VOXEL_SERVER] all direction inflation done." << endl;
        inflated_path.row(iRow) << 
            bdy[_BDY_L_X], bdy[_BDY_R_X],
            bdy[_BDY_L_Y], bdy[_BDY_R_Y],
            bdy[_BDY_L_Z], bdy[_BDY_R_Z];
    }
#endif
    return inflated_path;
}

sensor_msgs::PointCloud getPointCloudFromTraj(
        const Eigen::MatrixXd & traj,
        const double t,
        string frame_id = "/map",
        double d_t = 0.01)
{
    VectorXd T(1);
    T << t;
    return getPointCloudFromTraj(traj, T, frame_id, d_t);
}

sensor_msgs::PointCloud getPointCloudFromTraj(
        const Eigen::MatrixXd &traj,
        const Eigen::VectorXd &time,
        string frame_id,
        double d_t)
{
    sensor_msgs::PointCloud ret;
    ret.header.frame_id = frame_id;
    ret.header.stamp = ros::Time::now();

    //ROS_WARN_STREAM("[POSE_UTILS] t = " << time.transpose());
    ret.points.reserve((time.sum() + d_t) / d_t);

    int m = time.rows(), n = traj.rows()/m;
    
    geometry_msgs::Point32 p;

    for (int i = 0; i < m; ++i)
    {
        double t_beg = 0.0, t_end = time(i);
        for (double t= t_beg; t < t_end; t+= d_t)
        {
            p.z = p.y = p.x = 0.0;
            double T = 1.0;
            for (int j = 0; j < n; ++j)
            {
                p.x += traj(i * n + j, _DIM_X) * T;
                p.y += traj(i * n + j, _DIM_Y) * T;
                p.z += traj(i * n + j, _DIM_Z) * T;
                T *= t;
            }
            ret.points.push_back(p);
        }
    }
    return ret;
}

sensor_msgs::PointCloud getPointCloudFromStdVec(
        const vector<double> & pts,
        string frame_id)
{
    sensor_msgs::PointCloud ret;
    ret.header.frame_id = frame_id;
    ret.header.stamp = ros::Time::now();
    ret.points.reserve(pts.size() / _TOT_DIM);
    geometry_msgs::Point32 p;
    for (size_t idx = 0; idx * _TOT_DIM < pts.size(); idx += 1)
    {
        p.x = pts[idx * _TOT_DIM + _DIM_X];
        p.y = pts[idx * _TOT_DIM + _DIM_Y];
        p.z = pts[idx * _TOT_DIM + _DIM_Z];
        ret.points.push_back(p);
    }
    return ret;
}

vector<double> getStdVecFromPointCloud(
        const sensor_msgs::PointCloud & cloud,
        double margin)
{
    vector<double> pts;
    pts.reserve(cloud.points.size() * _TOT_BDY);

    for (auto & p : cloud.points) 
    {
        pts.push_back(p.x - margin);
        pts.push_back(p.x + margin);
        pts.push_back(p.y - margin);
        pts.push_back(p.y + margin);
        pts.push_back(p.z - margin);
        pts.push_back(p.z + margin);
    }
    return pts;
}

vector<double> getStdVecFromBlockCloud(
        const sensor_msgs::PointCloud & cloud,
        double margin)
{
    vector<double> blks;
    blks.reserve(cloud.points.size() * _TOT_DIM);
    for (size_t idx = 0; idx < cloud.points.size(); idx += 2)
    {
        blks.push_back(cloud.points[idx].x - margin);
        blks.push_back(cloud.points[idx + 1].x + margin);
        blks.push_back(cloud.points[idx].y - margin);
        blks.push_back(cloud.points[idx + 1].y + margin);
        blks.push_back(cloud.points[idx].z - margin);
        blks.push_back(cloud.points[idx + 1].z + margin);
    }
    return blks;
}

Eigen::MatrixXd getStateFromTrajByTime(
        const Eigen::MatrixXd & coef,
        double t)
{
    assert(coef.cols() == _TOT_DIM);
    MatrixXd ret = MatrixXd::Zero(3, 3);
    double T = 1.0;
    int n = coef.rows();
    for (int j = 0, i = 0; j < n; ++j)
    {
        ret.row(0) += coef.row(i * n + j) * T;
        if (j + 1 < n)
            ret.row(1) += coef.row(i * n + j + 1) * T * (j + 1);
        if (j + 2 < n)
            ret.row(2) += coef.row(i * n + j + 1) * T * (j + 1) * (j + 2);
        T *= t;
    }
    return ret;
}

Eigen::MatrixXd getStateFromTrajByTime(
        const Eigen::MatrixXd & coef,
        const Eigen::VectorXd & time,
        double t)
{
    assert(time.rows() > 0 && coef.rows() > 0);
    assert(coef.cols() == _TOT_DIM);
    int m = time.rows(), n = coef.rows() / m;
    t = min(t, time.sum());
    t = max(0.0, t);

    MatrixXd ret = MatrixXd::Zero(3, _TOT_DIM);

    for (int i = 0; i < m; ++i)
    {
        if (i + 1  < m && t > time(i)) 
            t -= time(i);
        else
        {
#if 0
            double T = 1.0;
            for (int j = 0; j < n; ++j)
            {
                ret.row(0) += coef.row(i * n + j) * T;
                if (j + 1 < n)
                    ret.row(1) += coef.row(i * n + j + 1) * T * (j + 1);
                if (j + 2 < n)
                    ret.row(2) += coef.row(i * n + j + 2) * T * (j + 1) * (j + 2); 
                T *= t;
            }
#else
            //ROS_WARN_STREAM("[get state] " << n << ", " << m);
            Eigen::MatrixXd T = MatrixXd::Zero(3, n);
            //ROS_WARN_STREAM("[get state] " << n << ", " << m);
            VectorXd cum_t(n);
            cum_t(0) = 1.0;
            for (int j = 1; j < n; ++j) cum_t(j) = cum_t(j - 1) * t;
            for (int j = 0; j < n; ++j)
            {
                T(0, j) = cum_t(j);
                if (j > 0) T(1, j) = cum_t(j - 1) * j;
                if (j > 1) T(2, j) = cum_t(j - 2) * j * (j -1);
            }
            ret = T * coef.block(i * n, 0, n, 3);
#endif
            break;
        }
    }

    return ret;
}

// to be test
quadrotor_msgs::PolynomialTrajectory getTrajMsgByMatrix(
        uint32_t & id,
        uint32_t action,
        const Eigen::MatrixXd & coef,
        const Eigen::VectorXd & time,
        ros::Time stamp,
        string frame_id)
{
    quadrotor_msgs::PolynomialTrajectory ret;
    ret.header.stamp = stamp;
    ret.header.frame_id = frame_id;

    ret.trajectory_id = id++;
    ret.action = action;

    int m  = time.rows(), n = coef.rows()/ m;
    ret.num_segment = m;
    ret.num_order = n;

    ret.time = vector<double>(time.data(), time.data() + m);
    ret.coef_x = vector<double>(coef.data() + n * m * _DIM_X, coef.data() + n * m * _DIM_Y);
    ret.coef_y = vector<double>(coef.data() + n * m * _DIM_Y, coef.data() + n * m * _DIM_Z);
    ret.coef_z = vector<double>(coef.data() + n * m * _DIM_Z, coef.data() + n * m * _TOT_DIM);

    ret.start_yaw = 0.0;
    ret.final_yaw = 0.0;
    return ret;
}

vector<double> getStdVecFromLaserScan(
        const sensor_msgs::LaserScan & scan,
        const nav_msgs::Odometry & odom,
        double margin,
        double resolution,
        int count_thld,
        double height_thld,
        double extra_height)
{
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;
    projector.projectLaser(scan, cloud);

    Eigen::Quaterniond quad(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z);
    Eigen::Matrix3d rotate = quad.toRotationMatrix();

    Eigen::Vector3d trans(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z);

    vector<double> blk;

    if (cloud.points.empty()) return blk;
    
    auto getGlobalPoint = [&](geometry_msgs::Point32 pt)
    {
        Eigen::Vector3d ret(pt.x, pt.y, pt.z);
        return rotate * ret + trans;
    };

    blk.reserve(cloud.points.size() * _TOT_BDY);
    Eigen::Vector3d last_pt = getGlobalPoint(cloud.points.front());
    int count = -1;

    for (auto & lp: cloud.points)
    {
        auto pt = getGlobalPoint(lp);

        if ( (pt - last_pt).norm() < resolution)
            count += 1;
        else
        {
            if (count >= count_thld && pt(_DIM_Z) > height_thld)
            {
                blk.push_back(pt(_DIM_X) - margin);
                blk.push_back(pt(_DIM_X) + margin);
                blk.push_back(pt(_DIM_Y) - margin);
                blk.push_back(pt(_DIM_Y) + margin);
                blk.push_back(pt(_DIM_Z) - margin - extra_height);
                blk.push_back(pt(_DIM_Z) + margin + extra_height);
            }
            count = 0;
            last_pt = pt;
        }
    }
    if (count >= count_thld)
    {
        blk.push_back(last_pt(_DIM_X) - margin);
        blk.push_back(last_pt(_DIM_X) + margin);
        blk.push_back(last_pt(_DIM_Y) - margin);
        blk.push_back(last_pt(_DIM_Y) + margin);
        blk.push_back(last_pt(_DIM_Z) - margin);
        blk.push_back(last_pt(_DIM_Z) + margin + extra_height);
    }
    return blk;
}

Eigen::Vector3d getWorldPositionFromCam(
        const geometry_msgs::Pose & pose_tgt_cam,
        const geometry_msgs::Pose & pose_cam_bd,
        const geometry_msgs::Pose & pose_bd_wd)
{
    Eigen::Vector3d p_tgt_cam, p_cam_bd, p_bd_wd;
    Eigen::Matrix3d o_cam_bd, o_bd_wd;
    
    p_tgt_cam <<
        pose_tgt_cam.position.x,
        pose_tgt_cam.position.y,
        pose_tgt_cam.position.z;

    p_cam_bd << 
        pose_cam_bd.position.x, 
        pose_cam_bd.position.y, 
        pose_cam_bd.position.z; 

    p_bd_wd <<
        pose_bd_wd.position.x,
        pose_bd_wd.position.y,
        pose_bd_wd.position.x;


    o_cam_bd = Quaterniond(
        pose_cam_bd.orientation.w,
        pose_cam_bd.orientation.x,
        pose_cam_bd.orientation.y,
        pose_cam_bd.orientation.z).toRotationMatrix();

    o_bd_wd = Quaterniond(
        pose_bd_wd.orientation.w,
        pose_bd_wd.orientation.x,
        pose_bd_wd.orientation.y,
        pose_bd_wd.orientation.z).toRotationMatrix();

    return o_bd_wd * (o_cam_bd * p_tgt_cam + p_cam_bd) + p_bd_wd;
}


geometry_msgs::Pose getWorldPoseMsgFromCam(
        const geometry_msgs::Pose & pose_tgt_cam,
        const geometry_msgs::Pose & pose_cam_bd,
        const geometry_msgs::Pose & pose_bd_wd)
{    
    Eigen::Vector3d p_tgt_cam, p_cam_bd, p_bd_wd;
    Eigen::Matrix3d o_tgt_cam, o_cam_bd, o_bd_wd;
    
    p_tgt_cam <<
        pose_tgt_cam.position.x,
        pose_tgt_cam.position.y,
        pose_tgt_cam.position.z;

    p_cam_bd << 
        pose_cam_bd.position.x, 
        pose_cam_bd.position.y, 
        pose_cam_bd.position.z; 

    p_bd_wd <<
        pose_bd_wd.position.x,
        pose_bd_wd.position.y,
        pose_bd_wd.position.x;

    o_tgt_cam = Quaterniond(
        pose_tgt_cam.orientation.w,
        pose_tgt_cam.orientation.x,
        pose_tgt_cam.orientation.y,
        pose_tgt_cam.orientation.z).toRotationMatrix();

    o_cam_bd = Quaterniond(
        pose_cam_bd.orientation.w,
        pose_cam_bd.orientation.x,
        pose_cam_bd.orientation.y,
        pose_cam_bd.orientation.z).toRotationMatrix();

    o_bd_wd = Quaterniond(
        pose_bd_wd.orientation.w,
        pose_bd_wd.orientation.x,
        pose_bd_wd.orientation.y,
        pose_bd_wd.orientation.z).toRotationMatrix();

    Eigen::Vector3d pos = o_bd_wd * (o_cam_bd * p_tgt_cam + p_cam_bd) + p_bd_wd;
    Eigen::Quaterniond ort(o_bd_wd * o_cam_bd * o_tgt_cam);
    geometry_msgs::Pose ret;
    ret.position.x = pos(0);
    ret.position.y = pos(1);
    ret.position.z = pos(2);
    ret.orientation.w = ort.w();
    ret.orientation.x = ort.x();
    ret.orientation.y = ort.y();
    ret.orientation.z = ort.z();
    return ret;
}

#endif
