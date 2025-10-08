// -----------------------------------------------------------------------------
// FILE: src/dynamic_reconfig.cpp
// -----------------------------------------------------------------------------
#include "mpc_planner/dynamic_reconfig.h"

namespace mpc_planner {

DynamicReconfig::DynamicReconfig() = default;

void DynamicReconfig::setCallback(std::function<void(const Config&)> cb) {
    cb_ = std::move(cb);
}

void DynamicReconfig::poll(ros::NodeHandle& nh) {
    Config cfg;
    
    // read an array of doubles for q and r (if not present use defaults)
    std::vector<double> qv, rv;
    nh.getParam("~q_weights", qv);
    nh.getParam("~r_weights", rv);
    nh.param("~horizon", cfg.horizon, last_config_.horizon);
    nh.param("~dt", cfg.dt, last_config_.dt);

    if (!qv.empty()) {
        cfg.q_weights = Eigen::VectorXd::Map(qv.data(), qv.size());
    } else {
        cfg.q_weights = last_config_.q_weights;
    }
    if (!rv.empty()) {
        cfg.r_weights = Eigen::VectorXd::Map(rv.data(), rv.size());
    } else {
        cfg.r_weights = last_config_.r_weights;
    }

    // call the callback if changed
    bool changed = false;
    if (cfg.horizon != last_config_.horizon || cfg.dt != last_config_.dt) changed = true;
    if (cfg.q_weights.size() != last_config_.q_weights.size()) changed = true;
    if (cfg.r_weights.size() != last_config_.r_weights.size()) changed = true;
    if (!changed && cfg.q_weights.size() > 0) {
        if (!cfg.q_weights.isApprox(last_config_.q_weights)) changed = true;
    }
    if (!changed && cfg.r_weights.size() > 0) {
        if (!cfg.r_weights.isApprox(last_config_.r_weights)) changed = true;
    }

    if (changed && cb_) {
        cb_(cfg);
        last_config_ = cfg;
    }
}

} // namespace mpc_planner