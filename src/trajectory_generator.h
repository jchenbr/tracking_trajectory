
#ifndef _VOXEL_TRAJECTORY_TRAJECTORY_GENERATOR_H_
#define _VOXEL_TRAJECTORY_TRAJECTORY_GENERATOR_H_

#include "voxelmacro.h"

#include <eigen3/Eigen/Dense>
#include <vector>

namespace VoxelTrajectory
{
    class TrajectoryGenerator
    {
public:
        /* destination trajectory generation call */
        // generate the trajectory to reach the goal
        std::vector<double> qp_cost;
            std::pair<Eigen::MatrixXd,Eigen::VectorXd> genPolyCoeffTime(
                const Eigen::MatrixXd &PBE,
                const Eigen::MatrixXd &inflated_path,
                const Eigen::MatrixXd &vel,
                const Eigen::MatrixXd &acc,
                const double maxVel,
                const double maxAcc,
                const double fVel,
                const double fAcc,
                double & coeff_t);

        // Here comes the tracking code.
        // BTW by history reason, I use row based representation.
        //  Todo, change to colomn based repsentation.
        struct DimensionConfig;
        struct TrackingCorridorConfig;
        struct TrajectoryConfig;

        /* tracking trajectory generation call */
        // generate the tracking trajectory to follow the target
        TrajectoryConfig getTrackingTrajectory(
                TrackingCorridorConfig & config);
        
        struct DimensionConfig
        {
            Eigen::VectorXd traj;
            Eigen::VectorXd init_state, dest_state, time;
            Eigen::MatrixXd grid, uninflated_grid;
            Eigen::MatrixXd face; 

            int M, N, R;
            double max_vel, max_acc;
            double traj_time;
        };

        struct TrackingCorridorConfig
        {
            std::vector<std::pair<double, double> > traj_time;
            Eigen::MatrixXd traj; // estmated target trajectory

            Eigen::MatrixXd uninflated_grid;
            Eigen::MatrixXd grid; // the grid of the corridor
            Eigen::MatrixXd face; // the connecting region of two grid
            Eigen::VectorXd time; // the duration within specified grid

            Eigen::MatrixXd init_state, dest_state;
            
            int M, N, R; // M:#segments, N:#Order, R:#to minimize
            double max_vel, max_acc; // restriction
            double f_vel, f_acc; // the dynamic for time allocation
            double t_beg, t_end, stamp;
            double lambda;
            
            bool use_stop_policy = false;

            bool isLegal();
            DimensionConfig getDimensionConfig(int dim);
            void moveForward(double);
        };

        struct TrajectoryConfig
        {
            Eigen::MatrixXd coef;
            Eigen::VectorXd time;
            //double coeff_t;
            int error_code = 0;
        };
private:
   };


}


#endif
