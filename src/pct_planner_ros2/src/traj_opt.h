#ifndef TRAJ_OPT_H
#define TRAJ_OPT_H

#include <Eigen/Dense>

namespace pct_planner {

class GPMPOptimizer {
public:
    GPMPOptimizer();
    ~GPMPOptimizer();
    
    Eigen::MatrixXd getOptInitValue();
    Eigen::VectorXd getOptInitLayer();
    Eigen::MatrixXd getResultMatrix();
    Eigen::VectorXd getLayers();
    Eigen::VectorXd getHeights();
    
private:
    Eigen::MatrixXd opt_init_value_;
    Eigen::VectorXd opt_init_layer_;
    Eigen::MatrixXd result_matrix_;
    Eigen::VectorXd layers_;
    Eigen::VectorXd heights_;
};

} // namespace pct_planner

#endif // TRAJ_OPT_H