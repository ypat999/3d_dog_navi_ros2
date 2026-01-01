#include "traj_opt.h"
#include <Eigen/Dense>

namespace pct_planner {

GPMPOptimizer::GPMPOptimizer() {
    // 构造函数
}

GPMPOptimizer::~GPMPOptimizer() {
    // 析构函数
}

Eigen::MatrixXd GPMPOptimizer::getOptInitValue() {
    // 返回优化初始值
    return opt_init_value_;
}

Eigen::VectorXd GPMPOptimizer::getOptInitLayer() {
    // 返回优化初始层
    return opt_init_layer_;
}

Eigen::MatrixXd GPMPOptimizer::getResultMatrix() {
    // 返回结果矩阵
    return result_matrix_;
}

Eigen::VectorXd GPMPOptimizer::getLayers() {
    // 返回层数
    return layers_;
}

Eigen::VectorXd GPMPOptimizer::getHeights() {
    // 返回高度
    return heights_;
}

} // namespace pct_planner