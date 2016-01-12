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
#include <eigen3/Eigen/Dense>
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
    int m = time.rows(), n = coef.rows() / m;
    t = min(t, time.sum());
    t = max(0.0, t);

    MatrixXd ret = MatrixXd::Zero(3, _TOT_DIM);

    for (int i = 0; i < m; ++i)
    {
        if (t > time(i)) 
            t -= time(i);
        else
        {
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

#endif
