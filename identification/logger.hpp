#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

extern "C" {
    #include "extApi.h"
}

/*
 * StepLogger — lightweight CSV recorder for joint identification using Coppelia simulation time.
 *
 * Each row:   t_ms, qdot_cmd, qdot_simu
 *
 *   t_ms      : milliseconds from Coppelia simulation time (via simxGetLastCmdTime)
 *               This respects the real-time factor set in Coppelia, unlike wall-clock time.
 *   qdot_cmd  : commanded velocity (or position)
 *   qdot_simu : measured joint velocity (or position) from the simulator
 *
 * Usage:
 *   StepLogger log("joint2_step.csv", joint_idx, clientID);
 *   log.record(qdot_cmd_vec, qdot_simu_vec);   // call once per control tick
 *   log.flush();                          // write to disk at end
 */
class StepLogger
{
public:
    StepLogger(const std::string& filename, int joint_index, int clientID)
        : joint_idx_(joint_index), filename_(filename), clientID_(clientID)
    {
        // Store initial simulation time from Coppelia
        t0_ms_ = simxGetLastCmdTime(clientID_);
        // Reserve space to avoid reallocation during the experiment
        rows_.reserve(4096);
    }

    // Record one control tick.
    // qdot_cmd  : full joint command vector (only joint_idx_ column is written)
    // qdot_simu : full measured joint vector
    void record(const Eigen::VectorXd& qdot_cmd, const Eigen::VectorXd& qdot_simu)
    {
        // Get current simulation time from Coppelia (in milliseconds)
        simxInt now_ms = simxGetLastCmdTime(clientID_);
        long t_ms = now_ms - t0_ms_;

        Row r;
        r.t_ms     = t_ms;
        r.qdot_cmd   = qdot_cmd(joint_idx_);
        r.qdot_simu  = qdot_simu(joint_idx_);
        rows_.push_back(r);
    }

    // Write all buffered rows to disk.
    void flush() const
    {
        std::ofstream f(filename_);
        if (!f.is_open()) {
            fprintf(stderr, "StepLogger: cannot open %s\n", filename_.c_str());
            return;
        }
        f << "t_ms,qdot_cmd,qdot_simu\n";
        for (const auto& r : rows_)
            f << r.t_ms  << "," << r.qdot_cmd << "," << r.qdot_simu << "\n";
        fprintf(stdout, "StepLogger: wrote %zu rows to %s\n",
                rows_.size(), filename_.c_str());
    }

private:
    struct Row { long t_ms; double qdot_cmd; double qdot_simu; };

    int                  joint_idx_;
    std::string          filename_;
    int                  clientID_;
    simxInt              t0_ms_;
    std::vector<Row>     rows_;
};

#endif // LOGGER_HPP
