#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "a_star.h"
#include "ele_planner.h"
#include "traj_opt.h"

namespace py = pybind11;
using namespace pct_planner;

PYBIND11_MODULE(pct_planner_cpp, m) {
    m.doc() = "PCT Planner C++ bindings for Python";

    py::class_<Astar>(m, "Astar")
        .def(py::init<>())
        .def("init", &Astar::init)
        .def("get_result", &Astar::getResult)
        .def("get_result_matrix", &Astar::getResultMatrix);

    py::class_<OfflineElePlanner>(m, "OfflineElePlanner")
        .def(py::init<double, bool>(), 
             py::arg("max_heading_rate") = 10.0, 
             py::arg("use_quintic") = true)
        .def("init_map", &OfflineElePlanner::init_map,
             py::arg("max_iteration"),
             py::arg("max_duration"), 
             py::arg("resolution"),
             py::arg("n_slice"),
             py::arg("min_clearance"),
             py::arg("trav"),
             py::arg("elev_g"),
             py::arg("elev_c"),
             py::arg("gateway"),
             py::arg("trav_gy"),
             py::arg("trav_gx"))
        .def("plan", &OfflineElePlanner::plan,
             py::arg("start"), py::arg("end"), py::arg("visualize") = false)
        .def("get_path_finder", &OfflineElePlanner::getPathFinder,
             py::return_value_policy::reference)
        .def("get_trajectory_optimizer", &OfflineElePlanner::getTrajectoryOptimizer)
        .def("get_trajectory_optimizer_wnoj", &OfflineElePlanner::getTrajectoryOptimizerWNoj);

    py::class_<GPMPOptimizer>(m, "GPMPOptimizer")
        .def(py::init<>())
        .def("get_opt_init_value", &GPMPOptimizer::getOptInitValue)
        .def("get_opt_init_layer", &GPMPOptimizer::getOptInitLayer)
        .def("get_result_matrix", &GPMPOptimizer::getResultMatrix)
        .def("get_layers", &GPMPOptimizer::getLayers)
        .def("get_heights", &GPMPOptimizer::getHeights);
}