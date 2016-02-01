#ifndef _TARGET_TRAJECTORY_ESTIMATOR_H_
#define _TARGET_TRAJECTORY_ESTIMATOR_H_
#include "macro.h"
#include <ros/console.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace TrackingTrajectory 
{
    typedef std::vector<std::pair<double, Eigen::VectorXd > > TargetObservation;

    class TrajectoryEstimator
    {

        private:
            double _lambda = 0.01;
            double _pdt_time = 5;
            const int n_poly, n_min = 2;

        // some meta programming here!

        public:
            const Eigen::MatrixXd _coef;

            TrajectoryEstimator(int trajectory_polynomial_order)
                :n_poly(trajectory_polynomial_order) 
            {
            }

            void setLambda(double lambda) {_lambda = lambda;}
            void setPredictionTime(double time) {_pdt_time = time;}

            Eigen::MatrixXd estimateTrajectory(const TargetObservation & obs)
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_poly, n_poly);
                Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n_poly, _DIM_NUM);
                Eigen::VectorXd T(n_poly);
                
                //for (auto & pr : obs) ROS_WARN_STREAM("{estimator}"  << pr.first << ": " << pr.second.transpose());

                { // the estimation error
                    double w = 0;
                    for (const auto & ob: obs)
                    {
                        T(0) = 1.0;
                        for (int i = 1; i < n_poly; ++i) T(i) = T(i - 1) * ob.first;

                        w = 1.0;
                        A += T * T.transpose() * w;
                        b += T * ob.second.transpose() * w;
                    }
                    A /= w;
                    b /= w;
                }

                { // regulator
                    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n_poly, n_poly);
                    auto power = [] (double x, int n) -> double
                    {
                        double ret = 1.0;
                        while (n)
                        {
                            if (n & 1) ret *= x;
                            x *= x;
                            n >>= 1;
                        }
                        return ret;
                    };

                    auto NDiffK = [](int n, int k) -> int
                    {
                        static int factorial[] = {1, 1, 2, 6, 24, 120, 720, 5040};
                        return factorial[n]/factorial[n - k];
                    };

                    for (int i = n_min; i < n_poly; ++i)
                        for (int j = n_min; j < n_poly; ++j)
                            C(i, j) = NDiffK(i, n_min) * NDiffK(j, n_min) * 
                                (power(_pdt_time, i + j - n_min - n_min + 1) - 
                                power(obs.front().first, i + j - n_min - n_min + 1))/
                                (i + j - n_min - n_min + 1);
                    A += _lambda * C * (obs.size() * 0.01);
                }

                Eigen::MatrixXd A_tmp = A.transpose() * A;
                Eigen::MatrixXd b_tmp = A.transpose() * b;
                Eigen::MatrixXd ret =  A_tmp.ldlt().solve(b_tmp);
                // std::clog << "[ESTIMATOR] coeff = \n" << ret << std::endl;
                return ret;
            }
    };
}

#endif
