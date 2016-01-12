#ifndef _TARGET_TRAJECTORY_ESTIMATOR_H_
#define _TARGET_TRAJECTORY_ESTIMATOR_H_
#include "macro.h"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace TrackingTrajectory 
{
    typedef std::vector<std::pair<double, Eigen::VectorXd > > TargetObservation;

    class TrajectoryEstimator
    {

        private:

        public:
            const int n_poly;

            TrajectoryEstimator(int trajectory_polynomial_order)
                :n_poly(trajectory_polynomial_order)
            {
            }

            Eigen::MatrixXd estimateTrajectory(const TargetObservation & obs)
            {
                for (auto & item : obs) std::clog << item.first << " " << item.second.transpose() << std::endl;
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_poly, n_poly);
                Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n_poly, _DIM_NUM);
                Eigen::VectorXd T(n_poly);

                // std::clog << "[ESTIMATOR] already to go" << std::endl;

                double w = 0;
                for (const auto & ob: obs)
                {
                    T(0) = 1.0;
                    for (int i = 1; i < n_poly; ++i) T(i) = T(i - 1) * ob.first;

                    w = 1.0;
                    A += T * T.transpose() * w;
                    b += T * ob.second.transpose() * w;
                }
                 std::clog << "[ESTIMATOR] A = : \n" << A << std::endl;
                 std::clog << "[ESTIMATOR] b = : \n" << b << std::endl;

                Eigen::MatrixXd A_tmp = A.transpose() * A;
                Eigen::MatrixXd b_tmp = A.transpose() * b;
                Eigen::MatrixXd ret =  A_tmp.ldlt().solve(b_tmp);
                // std::clog << "[ESTIMATOR] coeff = \n" << ret << std::endl;
                return ret;
            }
    };
}

#endif
