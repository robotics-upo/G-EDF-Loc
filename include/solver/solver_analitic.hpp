#ifndef __DLL6DSOLVER_HPP__
#define __DLL6DSOLVER_HPP__

#include <vector>
#include <iostream>
#include <cmath>
#include <iomanip>

// Ceres & Eigen
#include "ceres/ceres.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include "rclcpp/rclcpp.hpp"

#include "gaussian_map/gaussian_map.hpp" 

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using gaussian_map::GaussianMap;

// -------------------------------------------------------------------------
// Analytic Cost Function
// -------------------------------------------------------------------------
class DLL6DAnalyticCostFunction : public ceres::SizedCostFunction<1, 3, 4> {
public:
    DLL6DAnalyticCostFunction(double px, double py, double pz, const GaussianMap& grid, double w = 1.0)
        : px_(px), py_(py), pz_(pz), grid_(grid), weight_(w) {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override {
        
        // 1. Extraction and Transformation
        const double tx = parameters[0][0];
        const double ty = parameters[0][1];
        const double tz = parameters[0][2];
        const double qw = parameters[1][0];
        const double qx = parameters[1][1];
        const double qy = parameters[1][2];
        const double qz = parameters[1][3];

        Eigen::Quaterniond Q(qw, qx, qy, qz);
        Q.normalize(); 
        
        Eigen::Vector3d P_local(px_, py_, pz_);
        Eigen::Vector3d P_world = Q * P_local + Eigen::Vector3d(tx, ty, tz);

        // 2. Map Query
        auto map_res = grid_.evaluateWithGradient(P_world.x(), P_world.y(), P_world.z());

        // 3. Out-of-bounds Check
        if (map_res.value >= 19.0) {
            residuals[0] = weight_ * 5.0;
            if (jacobians != NULL) {
                if (jacobians[0]) std::fill_n(jacobians[0], 3, 0.0);
                if (jacobians[1]) std::fill_n(jacobians[1], 4, 0.0);
            }
            return true;
        }

        // 4. Residual Calculation
        residuals[0] = weight_ * map_res.value;

        // 5. Jacobians
        if (jacobians != NULL) {
            Eigen::Vector3d g = weight_ * map_res.gradient;

            // Gradient Clamping
            double g_norm = g.norm();
            double max_grad_norm = 10.0;
            
            if (g_norm > max_grad_norm) {
                g = g * (max_grad_norm / g_norm);
            }

            // Translation Jacobian
            if (jacobians[0] != NULL) {
                jacobians[0][0] = g.x();
                jacobians[0][1] = g.y();
                jacobians[0][2] = g.z();
            }

            // Rotation Jacobian
            if (jacobians[1] != NULL) {
                Eigen::Vector3d v(Q.x(), Q.y(), Q.z());
                Eigen::Vector3d p = P_local;

                Eigen::Vector3d p = P_local;
                Eigen::Vector3d v_cross_p = v.cross(p);
                jacobians[1][0] = g.dot(2.0 * v_cross_p);

                jacobians[1][0] = g.dot(2.0 * v_cross_p);
                double _2x = 2.0 * Q.x();
                double _2y = 2.0 * Q.y();
                double _2z = 2.0 * Q.z();
                double _2w = 2.0 * Q.w();

                double _2w = 2.0 * Q.w();
                Eigen::Vector3d dRx;
                dRx[0] = 0.0 * p.x() + _2y * p.y() + _2z * p.z();
                dRx[1] = _2y * p.x() - 2.0 * _2x * p.y() - _2w * p.z();
                dRx[2] = _2z * p.x() + _2w * p.y() - 2.0 * _2x * p.z();
                jacobians[1][1] = g.dot(dRx);

                jacobians[1][1] = g.dot(dRx);
                Eigen::Vector3d dRy;
                dRy[0] = -2.0 * _2y * p.x() + _2x * p.y() + _2w * p.z();
                dRy[1] = _2x * p.x() + 0.0 * p.y() + _2z * p.z();
                dRy[2] = -_2w * p.x() + _2z * p.y() - 2.0 * _2y * p.z();
                jacobians[1][2] = g.dot(dRy);

                jacobians[1][2] = g.dot(dRy);
                Eigen::Vector3d dRz;
                dRz[0] = -2.0 * _2z * p.x() - _2w * p.y() + _2x * p.z();
                dRz[1] = _2w * p.x() - 2.0 * _2z * p.y() + _2y * p.z();
                dRz[2] = _2x * p.x() + _2y * p.y() + 0.0 * p.z();
                jacobians[1][3] = g.dot(dRz);
            }
        }
        return true;
    }
private:
    double px_, py_, pz_;
    const GaussianMap& grid_;
    double weight_;
};

// -------------------------------------------------------------------------
// Main Solver
// -------------------------------------------------------------------------
class DLL6DSolver
{
  private:
    GaussianMap &_grid;
    int _max_num_iterations;
    int _max_threads;
    double _robusKernelScale;
    int _last_num_iterations;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DLL6DSolver(GaussianMap *grid) : _grid(*grid)
    {
        _max_num_iterations = 100; 
        _max_threads = 20;
        _robusKernelScale = 1.0; 
        _last_num_iterations = 0;
    }

    ~DLL6DSolver(void) {} 

    bool setMaxIterations(int n) { _max_num_iterations = n; return true; }
    bool setMaxThreads(int n) { _max_threads = n; return true; }
    bool setRobustKernelScale(double s) { _robusKernelScale = s; return true; }
    int getFinalNumIterations() const { return _last_num_iterations; }

    // Debugging Tool
    void analyzeAlignment(const std::vector<pcl::PointXYZ>& cloud, 
                          double tx, double ty, double tz, 
                          const Eigen::Quaterniond& q) 
    {
    }

    // Solver
    bool solve(const std::vector<pcl::PointXYZ> &cloud, double &tx, double &ty, double &tz, 
                                                        Eigen::Quaterniond &q_in_out)
    {
        double t_param[3] = { tx, ty, tz };
        double t_param[3] = { tx, ty, tz };
        q_in_out.normalize();
        double q_param[4] = { q_in_out.w(), q_in_out.x(), q_in_out.y(), q_in_out.z() };

        _last_num_iterations = 0;

        // Stage 1: Coarse
        {
            Problem problem_coarse;
            problem_coarse.AddParameterBlock(t_param, 3);

            #if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
                ceres::Manifold* quaternion_manifold = new ceres::QuaternionManifold();
                problem_coarse.AddParameterBlock(q_param, 4, quaternion_manifold);
            #else
                ceres::LocalParameterization* quaternion_manifold = new ceres::QuaternionParameterization();
                problem_coarse.AddParameterBlock(q_param, 4, quaternion_manifold);
            #endif

            double coarse_scale = std::max(_robusKernelScale, 3.0);
            double coarse_scale = std::max(_robusKernelScale, 3.0);
            ceres::LossFunction* loss_coarse = new ceres::CauchyLoss(coarse_scale);

            for(const auto& point : cloud) {
                CostFunction* cost = new DLL6DAnalyticCostFunction(point.x, point.y, point.z, _grid, 1.0);
                problem_coarse.AddResidualBlock(cost, loss_coarse, t_param, q_param);
            }

            Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.num_threads = _max_threads;
            options.trust_region_strategy_type = ceres::DOGLEG;
            options.minimizer_progress_to_stdout = false;
            
            options.minimizer_progress_to_stdout = false;
            
            options.max_num_iterations = 20; 
            options.function_tolerance = 1e-2; 
            options.gradient_tolerance = 1e-4; 
            Solver::Summary summary;
            Solve(options, &problem_coarse, &summary);
            _last_num_iterations += summary.num_successful_steps;
        }

        // Stage 2: Fine
        {
            Problem problem_fine;
            problem_fine.AddParameterBlock(t_param, 3);
            
            #if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
                ceres::Manifold* quaternion_manifold = new ceres::QuaternionManifold();
                problem_fine.AddParameterBlock(q_param, 4, quaternion_manifold);
            #else
                ceres::LocalParameterization* quaternion_manifold = new ceres::QuaternionParameterization();
                problem_fine.AddParameterBlock(q_param, 4, quaternion_manifold);
            #endif

            ceres::LossFunction* loss_fine = new ceres::CauchyLoss(0.5); 
            ceres::LossFunction* loss_fine = new ceres::CauchyLoss(0.5); 

            for(const auto& point : cloud) {
                CostFunction* cost = new DLL6DAnalyticCostFunction(point.x, point.y, point.z, _grid, 1.0);
                problem_fine.AddResidualBlock(cost, loss_fine, t_param, q_param);
            }

            Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.num_threads = _max_threads;
            options.trust_region_strategy_type = ceres::DOGLEG;
            
            options.trust_region_strategy_type = ceres::DOGLEG;
            
            options.max_num_iterations = 15;        
            options.function_tolerance = 1e-6;      
            options.gradient_tolerance = 1e-10;
            options.parameter_tolerance = 1e-8;

            Solver::Summary summary;
            Solve(options, &problem_fine, &summary);
            _last_num_iterations += summary.num_successful_steps;
        }

        tx = t_param[0];
        tx = t_param[0];
        ty = t_param[1];
        tz = t_param[2];
        q_in_out = Eigen::Quaterniond(q_param[0], q_param[1], q_param[2], q_param[3]);
        q_in_out.normalize();

        return true; 
    }
};

#endif