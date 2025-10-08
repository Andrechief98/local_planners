// -----------------------------------------------------------------------------
// FILE: include/mpc_local_planner/dynamic_reconfig.h
// -----------------------------------------------------------------------------
#pragma once

#include <functional>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace mpc_local_planner {

// Minimal dynamic reconfigure shim. In a full implementation this would wrap
// dynamic_reconfigure::Server<YourConfig>. Here we provide a simple polling style
// configuration that reads from the parameter server and calls the registered callback
// when values change.

class DynamicReconfig {
public:
    struct Config {
        Eigen::VectorXd q_weights; // flattened state weights
        Eigen::VectorXd r_weights; // flattened control weights
        int horizon = 10;
        double dt = 0.1;
    };

    DynamicReconfig();

    // Set callback to be invoked on parameter update. Callback runs from caller thread
    void setCallback(std::function<void(const Config&)> cb);

    // Poll parameters once (call from planner main loop or a timer)
    void poll(ros::NodeHandle& nh);

private:
    Config last_config_;
    std::function<void(const Config&)> cb_ = nullptr;
};

} // namespace mpc_local_planner