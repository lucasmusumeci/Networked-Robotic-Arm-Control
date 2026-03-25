#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <string>
#include <vector>
#include <sys/time.h>
#include <Eigen/Dense>

/*
 * StepLogger — lightweight CSV recorder for joint identification.
 *
 * Each row:   t_us, joint_index, q_cmd, q_simu
 *
 *   t_us      : microseconds since the experiment started (from gettimeofday)
 *   joint_index: which joint is under excitation (0–5)
 *   q_cmd     : commanded value (velocity or position)
 *   q_simu    : measured joint value from the simulator
 *
 * Usage:
 *   StepLogger log("joint2_step.csv", 2);
 *   log.record(q_cmd_vec, q_simu_vec);   // call once per control tick
 *   log.flush();                          // write to disk at end
 */
class StepLogger
{
public:
    StepLogger(const std::string& filename, int joint_index)
        : joint_idx_(joint_index), filename_(filename)
    {
        gettimeofday(&t0_, nullptr);
        // Reserve space to avoid reallocation during the experiment
        rows_.reserve(4096);
    }

    // Record one control tick.
    // qdot_cmd  : full joint command vector (only joint_idx_ column is written)
    // qdot_simu : full measured joint vector
    void record(const Eigen::VectorXd& qdot_cmd, const Eigen::VectorXd& qdot_simu)
    {
        struct timeval now;
        gettimeofday(&now, nullptr);
        long t_us = (now.tv_sec  - t0_.tv_sec)  * 1000000L
                  + (now.tv_usec - t0_.tv_usec);

        Row r;
        r.t_us    = t_us;
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
        f << "t_us,joint,qdot_cmd,qdot_simu\n";
        for (const auto& r : rows_)
            f << r.t_us << "," << joint_idx_ << ","
              << r.qdot_cmd << "," << r.qdot_simu << "\n";
        fprintf(stdout, "StepLogger: wrote %zu rows to %s\n",
                rows_.size(), filename_.c_str());
    }

private:
    struct Row { long t_us; double qdot_cmd; double qdot_simu; };

    int                  joint_idx_;
    std::string          filename_;
    struct timeval       t0_;
    std::vector<Row>     rows_;
};

#endif // LOGGER_HPP
