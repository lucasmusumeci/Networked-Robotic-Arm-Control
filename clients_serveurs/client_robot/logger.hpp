#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <sys/time.h>

#define NB_JOINTS 6
typedef struct {
	int cmdType;
	double cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	double qdot_simu[NB_JOINTS];
	struct timeval time;
}msg_t;


/*
 * Time functions
 */
long long getTimeElapsed_ms(struct timeval * t0)
{
    struct timeval t;
    gettimeofday(&t, NULL);

    return (t.tv_sec - t0->tv_sec)*1000 + (t.tv_usec - t0->tv_usec)/1000;
}

long long diffTime_ms(struct timeval * t0, struct timeval * t)
{
    return (t->tv_sec - t0->tv_sec)*1000 + (t->tv_usec - t0->tv_usec)/1000;
}

long long timeval2ms(struct timeval *t)
{
    return t->tv_sec*1000 + t->tv_usec/1000;
}

/*
 * Logger — lightweight CSV recorder
 *
 * Each row:   t_ms, cmd[NB_JOINTS], q_simu[NB_JOINTS]
 *
 *   t_ms         : time in milliseconds when the command was sent by the client 
 *                  /!\ Simulation real-time factor must be very close to 1 for this to be accurate. If not, increase dt.
 *   cmd[n]    : command for joint n
 *   q_simu[n] : measured joint position for joint n from the simulator
 *
 * Usage:
 *   Logger log("name.csv");
 *   log.record(&msg);   // call once every time we receive back a message from the server
 *   log.flush();        // write to disk at end
 */
class Logger
{
public:
    Logger(const std::string& filename)
        : filename_(filename)
    {
        // Store initial simulation time for relative timing
        gettimeofday(&t0_, NULL);

        // Reserve space to avoid reallocation during the experiment
        rows_.reserve(4096);
    }

    // Record one control tick. Call this every time we receive a message from the server
    void record(const msg_t* msg)
    {
        // Get time relative to the start of the experiment
        long long t_ms = getTimeElapsed_ms(&t0_);

        Row r;
        r.t_ms   = t_ms;
        for( int i = 0; i < NB_JOINTS; i++)
        {
            r.cmd[i]    = msg->cmd[i];
            r.q_simu[i] = msg->q_simu[i];
        }
        rows_.push_back(r);
    }

    // Wait for pending responses (max 2s delay) before flushing
    // This ensures all sent commands have received responses
    void waitForPending(int socket, struct sockaddr_in* sock_addr, socklen_t* addr_len, msg_t* latest_msg, int timeout_ms = 2000)
    {
        struct timeval start;
        gettimeofday(&start, NULL);
        struct timeval now;
        gettimeofday(&now, NULL);
        
        msg_t tmp = {};
        while (diffTime_ms(&start, &now) < timeout_ms)  // Max timeout_ms milliseconds
        {
            // Try to receive any pending messages
            if (recvfrom(socket, &tmp, sizeof(tmp), 0, (struct sockaddr*)sock_addr, addr_len) != -1)
            {
                // Update with the latest message
                if (diffTime_ms(&latest_msg->time, &tmp.time) > 0)
                    *latest_msg = tmp;
                
                // Record it
                record(&tmp);
            }
            
            usleep(100);  // Sleep 100ns between checks to avoid busy waiting
            gettimeofday(&now, NULL);
        }
    }

    // Write all buffered rows to disk
    void flush() const
    {
        std::ofstream f(filename_);
        if (!f.is_open()) {
            fprintf(stderr, "StepLogger: cannot open %s\n", filename_.c_str());
            return;
        }
        // Write header: t_ms, cmd_1, cmd_2, ..., cmd_6, q_simu_1, q_simu_2, ..., q_simu_6
        f << "t_ms";
        for (int i = 0; i < NB_JOINTS; i++) f << ",cmd_" << (i+1);
        for (int i = 0; i < NB_JOINTS; i++) f << ",q_simu_" << (i+1);
        f << "\n";
        
        // Write data rows
        for (const auto& r : rows_) {
            f << r.t_ms;
            for (int i = 0; i < NB_JOINTS; i++) f << "," << r.cmd[i];
            for (int i = 0; i < NB_JOINTS; i++) f << "," << r.q_simu[i];
            f << "\n";
        }
        fprintf(stdout, "StepLogger: wrote %zu rows to %s\n",
                rows_.size(), filename_.c_str());
    }

private:
    struct Row { long t_ms; double cmd[NB_JOINTS]; double q_simu[NB_JOINTS]; };

    std::string          filename_;
    struct timeval       t0_;
    std::vector<Row>     rows_;
};

#endif // LOGGER_HPP
