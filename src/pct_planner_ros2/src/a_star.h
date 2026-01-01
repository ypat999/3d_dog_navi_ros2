#ifndef A_STAR_H
#define A_STAR_H

#include <vector>
#include <Eigen/Dense>

namespace pct_planner {

class Astar {
public:
    Astar();
    ~Astar();
    
    void init();
    std::vector<Eigen::Vector3d> getResult();
    Eigen::MatrixXd getResultMatrix();
    
private:
    std::vector<Eigen::Vector3d> path_;
};

} // namespace pct_planner

#endif // A_STAR_H