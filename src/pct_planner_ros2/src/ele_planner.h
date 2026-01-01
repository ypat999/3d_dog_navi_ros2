#ifndef ELE_PLANNER_H
#define ELE_PLANNER_H

#include <Eigen/Dense>
#include "a_star.h"

namespace pct_planner {

class OfflineElePlanner {
public:
    OfflineElePlanner(double max_heading_rate = 10.0, bool use_quintic = true);
    ~OfflineElePlanner();
    
    void init_map(
        int max_iteration, 
        int max_duration, 
        double resolution, 
        int n_slice, 
        double min_clearance,
        const Eigen::MatrixXd& trav,
        const Eigen::MatrixXd& elev_g,
        const Eigen::MatrixXd& elev_c,
        const Eigen::MatrixXi& gateway,
        const Eigen::MatrixXd& trav_gy,
        const Eigen::MatrixXd& trav_gx
    );
    
    void plan(const Eigen::Vector3i& start, const Eigen::Vector3i& end, bool visualize = false);
    
    Astar* getPathFinder();
    void* getTrajectoryOptimizer();  // 返回void*以匹配原始接口
    void* getTrajectoryOptimizerWNoj();  // 返回void*以匹配原始接口

private:
    double max_heading_rate_;
    bool use_quintic_;
    Astar path_finder_;
};

} // namespace pct_planner

#endif // ELE_PLANNER_H