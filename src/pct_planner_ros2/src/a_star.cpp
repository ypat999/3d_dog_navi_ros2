#include "a_star.h"
#include <vector>
#include <Eigen/Dense>

namespace pct_planner {

Astar::Astar() {
    // 构造函数
}

Astar::~Astar() {
    // 析构函数
}

void Astar::init() {
    // 初始化A*算法
}

std::vector<Eigen::Vector3d> Astar::getResult() {
    return path_;
}

Eigen::MatrixXd Astar::getResultMatrix() {
    if (path_.empty()) {
        return Eigen::MatrixXd(0, 0);
    }
    
    Eigen::MatrixXd matrix(path_.size(), 3);
    for (size_t i = 0; i < path_.size(); ++i) {
        matrix(i, 0) = path_[i](0);
        matrix(i, 1) = path_[i](1);
        matrix(i, 2) = path_[i](2);
    }
    
    return matrix;
}

} // namespace pct_planner