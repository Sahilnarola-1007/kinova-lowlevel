// =============================================================================
// low_level_feedback_loop.cpp   --   Phase 0, Step 5b
//
// Purpose:  Validate 1 kHz low-level tracking of a time-varying trajectory
//           using COMMANDED-ANCHOR integration, and measure the joint servo's
//           proportional gain K from the steady-state tracking gap.
//
//           A small sine is commanded on joint 7 (wrist roll); the other six
//           joints hold their start positions.
//
// Integration scheme:
//           q_send[k] = q_send[k-1] + q_dot_des(t) * dt          (q_send[0] = q_hold)
//
//           Summing the ideal velocity from the ideal start position rebuilds
//           the ideal trajectory: q_send == q_ideal, to integration error.
//           So (q_send - q_meas) IS the tracking error, and the arm's internal
//           servo -- which drives q_meas toward q_send -- closes the loop on it.
//           The host integrator is a REFERENCE GENERATOR, not the controller.
//
// Why not measured-anchor (Step 5a, superseded):
//           q_send[k] = q_meas[k] + q_dot_des*dt makes q_meas cancel:
//           the gap is identically q_dot_des*dt every cycle, on-trajectory or
//           not. It is an increment, not an error, so no correction path exists
//           and the servo only ever realizes K*dt of the demanded velocity.
//           Measured on hardware: 5.0 deg commanded -> 0.1741 deg tracked
//           (96.52 % amplitude leak). Inverting 3.48 % at dt = 1 ms gives
//           K ~ 35 /s. Correct arithmetic, wrong unit of meaning at the
//           set_position() interface, which takes a DESTINATION not a step.
//
// Windup -- the property measured-anchor was chosen for:
//           Re-anchoring suppressed integrator windup implicitly, by never
//           permitting a gap to exist. That also removed the actuation. Here
//           the same safety property is made EXPLICIT: GAP_ABORT_DEG monitors
//           |q_send - q_meas| and stops the run if the reference runs away
//           from the arm (physical block, joint limit, fault).
//
// Measurement discipline:
//           q_ideal is still computed independently from the closed-form sine.
//           Under commanded-anchor q_send should equal it; logging both lets
//           the integration error be seen rather than assumed.
//
// Deliverables of this run:
//           (1) measured amplitude -- expect >= 4.9 deg if the model holds
//           (2) K, from a through-origin LEAST-SQUARES fit of q_dot_des
//               against gap, excluding the first K_FIT_SKIP_CYCLES cycles.
//               A peak-ratio estimate is not used: it divides by a single
//               sample, so one scheduler stall corrupts it outright.
//           (3) the breakaway cycle -- when the joint first responds at all.
//
// Note on K ~ 35 above: that figure inverts the measured-anchor leak under the
// assumption gap = q_dot*dt. It is a consistency check on the failure mode, not
// a measurement of the servo, and it is NOT comparable to the commanded-anchor
// fit (63.8 /s, n=3). The two numbers were never measuring the same quantity.
//
// Standalone binary -- NOT a ROS2 node. Runs directly against the arm.
// Hardware-only -- no USE_KORTEX_MOCK path (mock has no UDP transport).
// Build with -DUSE_KORTEX_MOCK=OFF.
//
// Relation to existing code:
//   KinovaInterface creates BaseCyclicClient on TCP (port 10000) -- fine for
//   high-level RefreshFeedback() wrench reads.  This file creates it on UDP
//   (port 10001) for the 1 kHz real-time cyclic path.
// =============================================================================

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <atomic>
#include <fstream>
#include <iomanip>

// POSIX socket headers -- must come before Kortex.
// The SDK's OS detection fails on Ubuntu 24.04 / GCC 13 ("#warning Unknown OS type!")
// so it never includes these itself. Without them, sockaddr_in is incomplete.
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// Kortex API headers -- same set as KinovaInterface.hpp + UDP transport
#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace k_api = Kinova::Api;

// =============================================================================
// Configuration
// =============================================================================
constexpr const char* IP_ADDRESS     = "192.168.1.10";   // Same as kinova_wrapper
constexpr uint32_t    TCP_PORT       = 10000;
constexpr uint32_t    UDP_PORT       = 10001;            // Real-time cyclic path
constexpr uint32_t    NUM_JOINTS     = 7;
constexpr double      LOOP_PERIOD_US = 1000.0;           // 1 ms target
constexpr uint32_t    NUM_CYCLES     = 10000;            // 10 seconds at 1 kHz
constexpr double      JITTER_WARN_US = 1100.0;           // Flag cycles > 1.1 ms

constexpr uint32_t    SINE_JOINT     = 6;                // Joint 7 (0-indexed) -- wrist roll
constexpr double      SINE_AMPLITUDE = 5.0;              // [deg] -- small until validated
constexpr double      SINE_FREQ_HZ   = 0.2;              // [Hz]  -- 1 cycle / 5 s, 2 cycles in 10 s
constexpr double      TWO_PI         = 2.0 * M_PI;

// Per-cycle STEP guard [deg per 1 ms cycle]. This bounds the INCREMENT added
// to the integrator each cycle -- it must never be applied to the gap.
//   - Sine's peak legitimate step = A*2*pi*f*dt = 5*2*pi*0.2*0.001 = 0.00628 deg.
//     0.05 is ~8x above that, so normal motion never touches the clamp.
//   - Kinova velocity-error ceiling: large actuators 120 deg/s -> 0.12 deg/ms,
//     small actuators 200 deg/s -> 0.20 deg/ms. 0.05 is ~2.4x below the LOWER
//     of the two. UNVERIFIED: joint 7's actuator class -- confirm from Kortex docs.
constexpr double      MAX_STEP_DEG   = 0.05;

// Runaway guard on |q_send - q_meas| [deg]. This is a DETECTOR, not a rate
// limiter, and sizing it too low silently re-creates the Step 5a leak: any
// threshold below the legitimate gap caps velocity at K*threshold and
// strangles the motion.
//
// Two lower bounds must both be cleared:
//   (a) steady-state gap  = q_dot_peak / K = 6.283 / 63.8 ~ 0.10 deg
//   (b) STARTUP TRANSIENT peak gap ~ 0.66 deg (measured, n=3)
//       The joint does not respond for the first ~104 cycles while q_send
//       integrates away from it. (b) dominates (a) by ~6.5x and is therefore
//       the binding constraint. A threshold of 0.8 deg would abort every run.
//
// 1.5 deg is ~2.3x the startup transient and ~15x the steady-state gap --
// clear of all normal operation, small enough that a genuine stall is caught
// within ~0.24 s of demanded travel.
//
// NOTE: K ~ 63.8 /s is INFERRED from hardware (n=3, spread 1.5%), not a Kortex
// spec. See design.md "Servo model".
constexpr double      GAP_ABORT_DEG  = 1.5;

// Cycles excluded from the head of the K least-squares fit.
// The joint is unresponsive for ~104 cycles at loop start (cause UNVERIFIED --
// see design.md "Startup transient"). During that window the gap grows to
// ~0.66 deg while the true steady-state gap is ~0.10 deg, so those samples lie
// far off the v = K*gap line. Including them dragged the fitted slope from
// 63.8 down to 48.9 /s -- a 30% error, reproducible across all three runs.
//
// Least squares gives no warning when the data does not obey the model; it
// simply averages the offending points in. 300 is ~3x the observed transient.
// If the transient length changes, re-derive this rather than assuming.
constexpr size_t      K_FIT_SKIP_CYCLES = 300;

// Motion threshold [deg] used to report when the joint first responds. Set well
// above encoder noise and well below the first cycle's legitimate travel.
constexpr double      BREAKAWAY_DEG  = 0.02;

// Guard band for the continuous-rotation wrap risk on joint 7. If the reported
// hold position sits this close to the 0/360 boundary, a wrap mid-run would
// corrupt the (q_ideal - q_meas) error by 360 deg and destroy the leak result.
// NOTE: whether Kortex wraps position() to [0,360) is UNVERIFIED -- confirm from
// the Kortex ActuatorFeedback documentation. This check is cheap insurance.
constexpr double      WRAP_GUARD_DEG = 15.0;

// =============================================================================
// Clean shutdown via SIGINT (Ctrl+C)
// =============================================================================
std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running.store(false);
}

// =============================================================================
// Timing + tracking instrumentation
//
// All five vectors are pushed exactly once per completed cycle, so index i
// refers to the same cycle in every vector. Cycle 0 has no previous cycle to
// difference against, so it records a sentinel 0.0 for cycle_dt rather than
// being skipped -- skipping would shift cycle_dt by one row relative to the
// position columns in the CSV.
// =============================================================================
struct TimingStats {
    std::vector<double> cycle_dt_us;   // Time between consecutive cycle starts
    std::vector<double> work_dt_us;    // Time inside Refresh() only (UDP round-trip)
    std::vector<double> q_ideal_deg;   // Independent closed-form reference
    std::vector<double> q_meas_deg;    // Measured position for SINE_JOINT
    std::vector<double> error_deg;     // q_ideal - q_meas
    std::vector<double> q_send_deg;    // Integrator state actually commanded
    std::vector<double> gap_deg;       // q_send - q_meas  == what the servo acts on
    std::vector<double> drift_deg;     // q_send - q_ideal == forward-Euler integration drift

    void reserve(size_t n) {
        cycle_dt_us.reserve(n);
        work_dt_us.reserve(n);
        q_ideal_deg.reserve(n);
        q_meas_deg.reserve(n);
        error_deg.reserve(n);
        q_send_deg.reserve(n);
        gap_deg.reserve(n);
        drift_deg.reserve(n);
    }

    void record_cycle(double dt) { cycle_dt_us.push_back(dt); }
    void record_work(double dt)  { work_dt_us.push_back(dt); }
    void record_position(double q_ideal, double q_send, double q_meas) {
        q_ideal_deg.push_back(q_ideal);
        q_send_deg.push_back(q_send);
        q_meas_deg.push_back(q_meas);
        error_deg.push_back(q_ideal - q_meas);
        gap_deg.push_back(q_send - q_meas);
        drift_deg.push_back(q_send - q_ideal);
    }

    void print_summary() const {
        // skip_first: cycle 0's cycle_dt is a sentinel, exclude it from stats.
        auto summarize = [](const std::string& label,
                            const std::vector<double>& v,
                            bool skip_first) {
            if (v.empty()) return;
            auto begin = v.begin() + (skip_first && v.size() > 1 ? 1 : 0);
            if (begin == v.end()) return;

            const auto  n    = static_cast<double>(std::distance(begin, v.end()));
            const double sum  = std::accumulate(begin, v.end(), 0.0);
            const double mean = sum / n;
            const double mn   = *std::min_element(begin, v.end());
            const double mx   = *std::max_element(begin, v.end());

            double sq_sum = 0.0;
            for (auto it = begin; it != v.end(); ++it) sq_sum += (*it - mean) * (*it - mean);
            const double stddev = std::sqrt(sq_sum / n);

            const int misses = static_cast<int>(std::count_if(begin, v.end(),
                [](double t) { return t > JITTER_WARN_US; }));

            std::cout << "\n--- " << label << " ---\n"
                      << "  Cycles:          " << static_cast<size_t>(n) << "\n"
                      << std::fixed << std::setprecision(1)
                      << "  Mean:            " << mean   << " us\n"
                      << "  Min:             " << mn     << " us\n"
                      << "  Max:             " << mx     << " us\n"
                      << "  Stddev:          " << stddev << " us\n"
                      << "  Misses (>1.1ms): " << misses << " ("
                      << std::setprecision(2)
                      << (100.0 * misses / n) << "%)\n";
        };

        std::cout << "\n========== TIMING SUMMARY ==========" << std::endl;
        summarize("Cycle-to-cycle dt (should be ~1000 us)", cycle_dt_us, true);
        summarize("Work time per cycle (Refresh() round-trip)", work_dt_us, false);

        if (error_deg.empty()) return;

        // --- Tracking error ---
        const double max_err = *std::max_element(error_deg.begin(), error_deg.end(),
            [](double a, double b) { return std::abs(a) < std::abs(b); });
        double sum_sq = 0.0;
        for (auto e : error_deg) sum_sq += e * e;
        const double rms = std::sqrt(sum_sq / static_cast<double>(error_deg.size()));

        // --- Amplitude leak: the Step 5 deliverable ---
        // Peak-to-peak of the measured signal vs the ideal, expressed as the
        // measured single-sided amplitude. Peak-to-peak is used (not max-minus-
        // hold) so a DC offset in the hold position cannot bias the result.
        const double meas_pp =
            *std::max_element(q_meas_deg.begin(), q_meas_deg.end()) -
            *std::min_element(q_meas_deg.begin(), q_meas_deg.end());
        const double meas_amp  = 0.5 * meas_pp;
        const double leak_deg  = SINE_AMPLITUDE - meas_amp;
        const double leak_pct  = 100.0 * leak_deg / SINE_AMPLITUDE;

        std::cout << "\n--- Sinusoid tracking (joint index " << SINE_JOINT << ") ---\n"
                  << std::fixed << std::setprecision(4)
                  << "  Ideal amplitude:    " << SINE_AMPLITUDE << " deg\n"
                  << "  Measured amplitude: " << meas_amp << " deg\n"
                  << "  AMPLITUDE LEAK:     " << leak_deg << " deg  ("
                  << std::setprecision(2) << leak_pct << " %)\n"
                  << std::setprecision(4)
                  << "  Max |error|:        " << std::abs(max_err) << " deg\n"
                  << "  RMS error:          " << rms << " deg\n";

        // --- Servo gain estimate (least squares) ---
        // Model: v = K * gap. At steady state the joint moves at the demanded
        // velocity, so  q_dot_des = K * gap  and K is the slope of q_dot_des
        // against gap through the origin:   K = sum(qd*gap) / sum(gap^2).
        //
        // A max-based estimate (qd_peak / |gap|_peak) is NOT used: a single
        // scheduler stall produces one outsized gap sample and biases K low by
        // a large factor. Least squares over all cycles is insensitive to that.
        //
        // The fit EXCLUDES the first K_FIT_SKIP_CYCLES cycles -- see that
        // constant. Both fits are printed: the difference between them is the
        // startup transient's contamination, and it is worth seeing every run
        // rather than trusting that the skip window is still adequate.
        //
        // CAVEAT: v = K*gap is an INFERRED first-order model, not a documented
        // Kortex control law. It reproduces both the measured-anchor failure and
        // the commanded-anchor success and is repeatable to 1.5% across three
        // runs, but it is not the servo's actual control law.
        if (gap_deg.size() > K_FIT_SKIP_CYCLES) {
            const double w = TWO_PI * SINE_FREQ_HZ;

            // Accumulate the through-origin fit twice: over all cycles, and over
            // cycles >= K_FIT_SKIP_CYCLES only.
            double num_all = 0.0, den_all = 0.0;
            double num_ss  = 0.0, den_ss  = 0.0;
            for (size_t i = 0; i < gap_deg.size(); ++i) {
                const double t  = static_cast<double>(i) * LOOP_PERIOD_US * 1e-6;
                const double qd = SINE_AMPLITUDE * w * std::cos(w * t);   // [deg/s]
                const double g  = gap_deg[i];
                num_all += qd * g;
                den_all += g * g;
                if (i >= K_FIT_SKIP_CYCLES) {
                    num_ss += qd * g;
                    den_ss += g * g;
                }
            }
            const double n_ss = static_cast<double>(gap_deg.size() - K_FIT_SKIP_CYCLES);
            const double gap_rms = std::sqrt(den_ss / n_ss);

            std::cout << std::setprecision(4)
                      << "  Peak demanded vel:  " << (SINE_AMPLITUDE * w) << " deg/s\n"
                      << "  RMS gap (steady):   " << gap_rms << " deg\n";

            if (den_ss > 1e-12) {
                const double K = num_ss / den_ss;
                std::cout << "  K (skip " << K_FIT_SKIP_CYCLES << "):        " << K
                          << " /s   (tau ~ " << std::setprecision(1)
                          << (1000.0 / K) << " ms)\n" << std::setprecision(4);
            }
            if (den_all > 1e-12) {
                std::cout << "  K (all cycles):     " << (num_all / den_all)
                          << " /s   <-- transient-contaminated, do not quote\n";
            }
        }

        // --- Startup transient: when did the joint actually begin to move? ---
        // The command integrates from cycle 0, but the joint has been observed
        // to stay put for ~104 cycles. Cause UNVERIFIED. Reported every run so
        // that a change in this number is noticed rather than absorbed.
        if (!q_meas_deg.empty()) {
            const double q0 = q_meas_deg.front();
            size_t first = q_meas_deg.size();
            for (size_t i = 0; i < q_meas_deg.size(); ++i) {
                if (std::abs(q_meas_deg[i] - q0) > BREAKAWAY_DEG) { first = i; break; }
            }
            if (first < q_meas_deg.size()) {
                std::cout << "  Breakaway cycle:    " << first << "  ("
                          << std::setprecision(1)
                          << (static_cast<double>(first) * LOOP_PERIOD_US * 1e-3)
                          << " ms, gap at breakaway " << std::setprecision(4)
                          << gap_deg[first] << " deg)\n";
            } else {
                std::cout << "  Breakaway cycle:    NONE -- joint never moved\n";
            }
        }

        // --- Integration drift: q_send vs the closed-form reference ---
        // Forward Euler accumulates a small error. This must stay far below the
        // amplitude leak, or the leak measurement is contaminated by the
        // integrator rather than by the servo.
        if (!drift_deg.empty()) {
            const double max_drift = std::abs(*std::max_element(
                drift_deg.begin(), drift_deg.end(),
                [](double a, double b) { return std::abs(a) < std::abs(b); }));
            std::cout << std::setprecision(4)
                      << "  Max |q_send-q_ideal|: " << max_drift << " deg (Euler drift)\n";
        }
    }

    void write_csv(const std::string& filename) const {
        std::ofstream f(filename);
        if (!f) {
            std::cerr << "Failed to open " << filename << " for writing." << std::endl;
            return;
        }
        f << "cycle,cycle_dt_us,work_dt_us,q_ideal_deg,q_send_deg,q_meas_deg,error_deg,gap_deg,drift_deg\n";

        // All vectors are the same length by construction; min() is a cheap
        // guard in case the loop broke on an exception mid-cycle.
        const size_t n = std::min({cycle_dt_us.size(), work_dt_us.size(),
                                   q_ideal_deg.size(), q_send_deg.size(),
                                   q_meas_deg.size(), error_deg.size(),
                                   gap_deg.size(), drift_deg.size()});
        for (size_t i = 0; i < n; ++i) {
            f << i << ","
              << std::fixed << std::setprecision(1)
              << cycle_dt_us[i] << "," << work_dt_us[i] << ","
              << std::setprecision(4)
              << q_ideal_deg[i] << "," << q_send_deg[i] << ","
              << q_meas_deg[i]  << "," << error_deg[i]  << ","
              << gap_deg[i]  << "," << drift_deg[i] << "\n";
        }
        std::cout << "Tracking log written to: " << filename
                  << " (" << n << " rows)" << std::endl;
    }
};

// =============================================================================
// Main
// =============================================================================
int main()
{
    std::signal(SIGINT, signal_handler);

    // =========================================================================
    // LAYER 1: Session plumbing
    //
    // KinovaInterface::connect() sets up:
    //   TransportClientTcp -> RouterClient -> SessionManager -> BaseClient
    //                                                        -> BaseCyclicClient (TCP)
    //
    // For low-level servoing we add a second transport path:
    //   TransportClientUdp -> RouterClient -> SessionManager -> BaseCyclicClient (UDP)
    //
    // Base services (SetServoingMode, ClearFaults, etc.) stay on TCP.
    // BaseCyclic (Refresh / RefreshFeedback at 1 kHz) moves to UDP.
    // =========================================================================

    // --- Transports ---
    auto transport_tcp = std::make_unique<k_api::TransportClientTcp>();
    auto transport_udp = std::make_unique<k_api::TransportClientUdp>();

    auto error_callback = [](k_api::KError err) {
        std::cerr << "[Kortex error] " << err.toString() << std::endl;
    };

    // Real SDK RouterClient takes (transport*, error_callback).
    // The mock RouterClient only takes (transport*) -- but this file is hardware-only.
    auto router_tcp = std::make_unique<k_api::RouterClient>(transport_tcp.get(), error_callback);
    auto router_udp = std::make_unique<k_api::RouterClient>(transport_udp.get(), error_callback);

    std::cout << "Connecting to Gen3 at " << IP_ADDRESS << " ..." << std::endl;

    try {
        transport_tcp->connect(IP_ADDRESS, TCP_PORT);
        transport_udp->connect(IP_ADDRESS, UDP_PORT);
    } catch (std::exception& e) {
        std::cerr << "Connection failed: " << e.what() << "\n"
                  << "Checklist:\n"
                  << "  1. Arm powered on and reachable at " << IP_ADDRESS << "?\n"
                  << "  2. No other session holding the arm?" << std::endl;
        return 1;
    }

    // --- Sessions on both transports ---
    // P2 rule: MUST set both timeouts or sessions die silently.
    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username("admin");
    session_info.set_password("admin");
    session_info.set_session_inactivity_timeout(60000);   // 60 s
    session_info.set_connection_inactivity_timeout(2000); //  2 s

    auto session_tcp = std::make_unique<k_api::SessionManager>(router_tcp.get());
    auto session_udp = std::make_unique<k_api::SessionManager>(router_udp.get());

    session_tcp->CreateSession(session_info);
    session_udp->CreateSession(session_info);
    std::cout << "Sessions established (TCP + UDP)" << std::endl;

    // --- Service clients ---
    auto base = std::make_unique<k_api::Base::BaseClient>(router_tcp.get());

    // BaseCyclicClient on the UDP router (NOT TCP like the current wrapper).
    // This is the critical difference for 1 kHz performance.
    auto base_cyclic = std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(router_udp.get());

    // --- Clear faults ---
    try {
        base->ClearFaults();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Faults cleared" << std::endl;
    } catch (k_api::KDetailedException& ex) {
        std::cerr << "Warning: ClearFaults: " << ex.what() << std::endl;
    }

    // =========================================================================
    // Mandatory startup order (arm firmware watchdog):
    //   1. RefreshFeedback()          -- seed the command with current state
    //   2. SetServoingMode(LOW_LEVEL) -- watchdog starts counting NOW
    //   3. enter the loop immediately -- any delay here faults the arm
    // Everything that can be done before step 2 is done before step 2.
    // =========================================================================
    auto feedback_init = base_cyclic->RefreshFeedback();

    // Seed the command: every joint holds where it currently is.
    k_api::BaseCyclic::Command command;
    command.set_frame_id(0);
    for (uint32_t i = 0; i < NUM_JOINTS; ++i) {
        const double q_hold = feedback_init.actuators(i).position();  // [deg]
        command.add_actuators();
        command.mutable_actuators(static_cast<int>(i))->set_command_id(0);
        command.mutable_actuators(static_cast<int>(i))->set_position(q_hold);
    }

    // Fixed reference for the sine joint. q_ideal is built on top of this.
    const double joint7_hold_deg = feedback_init.actuators(SINE_JOINT).position();

    std::cout << "Joint index " << SINE_JOINT << " hold position: "
              << std::fixed << std::setprecision(2) << joint7_hold_deg << " deg" << std::endl;
    std::cout << "Sinusoid: A=" << SINE_AMPLITUDE << " deg, f=" << SINE_FREQ_HZ
              << " Hz, peak vel=" << std::setprecision(2)
              << (SINE_AMPLITUDE * TWO_PI * SINE_FREQ_HZ) << " deg/s"
              << ", peak step=" << std::setprecision(5)
              << (SINE_AMPLITUDE * TWO_PI * SINE_FREQ_HZ * LOOP_PERIOD_US * 1e-6)
              << " deg/cycle (guard " << MAX_STEP_DEG << ")" << std::endl;

    // --- Wrap-risk check (see WRAP_GUARD_DEG) ---
    // Joint 7 is continuous-rotation. If position() is reported wrapped to
    // [0,360) and we start near the seam, the sine will cross it and the error
    // column jumps by 360 deg, invalidating the leak measurement.
    // Refuse to run rather than produce a corrupt dataset.
    if (joint7_hold_deg < WRAP_GUARD_DEG || joint7_hold_deg > (360.0 - WRAP_GUARD_DEG)) {
        std::cerr << "\nABORT: hold position " << joint7_hold_deg
                  << " deg is within " << WRAP_GUARD_DEG
                  << " deg of the 0/360 wrap seam.\n"
                  << "Jog joint " << (SINE_JOINT + 1)
                  << " to mid-range (e.g. ~90 deg) and re-run." << std::endl;
        session_tcp->CloseSession();
        session_udp->CloseSession();
        return 1;
    }

    // --- Enter low-level servoing. Watchdog is live after this call. ---
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // =========================================================================
    // LAYER 2: 1 kHz servo loop
    //
    // Per cycle:
    //   1. Refresh(command)  -- send last cycle's command, receive feedback
    //   2. Read q_meas from the feedback just received
    //   3. Compute desired velocity from the closed-form sine at time t
    //   4. q_send_prev += q_dot*dt        <- commanded-anchor integration
    //   5. Check |q_send_prev - q_meas| against GAP_ABORT_DEG (windup monitor)
    //   6. Stage q_send + bump command IDs; it goes out on the NEXT Refresh
    //   7. Log q_ideal, q_send, q_meas, gap
    //
    // Note the one-cycle pipeline: the command computed at the end of cycle k
    // is transmitted at the start of cycle k+1, i.e. a fixed 1 ms lag. At
    // 0.2 Hz that is 0.0063 deg of travel -- far below the expected 0.18 deg
    // steady-state gap, so it does not confound K, but it is real and is
    // stated in design.md.
    // =========================================================================

    TimingStats stats;
    stats.reserve(NUM_CYCLES);

    // THE integrator state. Seeded at the hold position so that summing the
    // ideal velocity reconstructs the ideal trajectory. This variable is never
    // reassigned from q_meas -- that reassignment was the Step 5a defect.
    double q_send_prev = joint7_hold_deg;

    std::cout << "\nStarting sinusoid tracking loop (" << NUM_CYCLES
              << " cycles, ~" << NUM_CYCLES / 1000 << " seconds)...\n" << std::endl;

    // Absolute-time target for drift-free pacing.
    auto t_next = std::chrono::steady_clock::now();
    auto t_prev = t_next;

    for (uint32_t cycle = 0; cycle < NUM_CYCLES && g_running.load(); ++cycle)
    {
        const auto t_cycle_start = std::chrono::steady_clock::now();

        // Cycle 0 has no predecessor: record a sentinel so every vector stays
        // index-aligned. print_summary() excludes it; the CSV keeps it as 0.0.
        const double cycle_dt = (cycle == 0)
            ? 0.0
            : std::chrono::duration<double, std::micro>(t_cycle_start - t_prev).count();
        stats.record_cycle(cycle_dt);
        t_prev = t_cycle_start;

        try {
            // Ideal elapsed time -- derived from the cycle counter, not the wall
            // clock, so scheduler jitter cannot distort the reference signal.
            const double t  = cycle * LOOP_PERIOD_US * 1e-6;   // [s]
            const double dt = LOOP_PERIOD_US * 1e-6;           // [s]

            // ----- Bracket ONLY the Refresh() round-trip -----
            // work_dt is defined as time inside the Kortex API call (UDP
            // round-trip), nothing else. Widening this bracket would make the
            // number incomparable to the Step 3 baseline (~350 us mean).
            const auto t_work_start = std::chrono::steady_clock::now();
            auto feedback = base_cyclic->Refresh(command);
            const auto t_work_end   = std::chrono::steady_clock::now();

            const double work_dt = std::chrono::duration<double, std::micro>(
                t_work_end - t_work_start).count();
            stats.record_work(work_dt);

            // ----- Commanded-anchor integration -----
            const double q_meas = feedback.actuators(SINE_JOINT).position();  // [deg]

            // Desired joint velocity [deg/s] = d/dt[ A*sin(w*t) ] = A*w*cos(w*t)
            const double qd_dot = SINE_AMPLITUDE * TWO_PI * SINE_FREQ_HZ
                                  * std::cos(TWO_PI * SINE_FREQ_HZ * t);

            // One integration step, magnitude-limited. clamp() preserves sign.
            // This bounds the INCREMENT only. Applying a limit to the gap below
            // would cap velocity at K*limit and re-create the Step 5a leak.
            const double delta = std::clamp(qd_dot * dt, -MAX_STEP_DEG, MAX_STEP_DEG);

            // Accumulate. Whatever the servo failed to close last cycle stays
            // in q_send_prev, so the gap grows until K*gap == qd_dot. That
            // residual is the integral term measured-anchor was discarding.
            q_send_prev += delta;

            // ----- Explicit windup / runaway monitor -----
            // Replaces the implicit suppression that re-anchoring provided.
            const double gap = q_send_prev - q_meas;   // == tracking error here
            if (std::abs(gap) > GAP_ABORT_DEG) {
                std::cerr << "\nABORT at cycle " << cycle << ": |gap| = "
                          << std::fixed << std::setprecision(4) << std::abs(gap)
                          << " deg exceeds GAP_ABORT_DEG (" << GAP_ABORT_DEG
                          << ").\nReference has run away from the arm -- check for a"
                          << " physical block, joint limit, or actuator fault."
                          << std::endl;
                break;
            }

            command.mutable_actuators(SINE_JOINT)->set_position(q_send_prev);

            // Bump frame + per-actuator command IDs so the arm can detect
            // stale or dropped packets.
            command.set_frame_id(command.frame_id() + 1);
            for (uint32_t i = 0; i < NUM_JOINTS; ++i) {
                auto* act = command.mutable_actuators(static_cast<int>(i));
                act->set_command_id(act->command_id() + 1);
            }

            // Independent reference -- closed form, never derived from q_send.
            // Under commanded-anchor q_send should converge to this; the two
            // columns together expose any integration drift.
            const double q_ideal = joint7_hold_deg
                                 + SINE_AMPLITUDE * std::sin(TWO_PI * SINE_FREQ_HZ * t);
            stats.record_position(q_ideal, q_send_prev, q_meas);

            if (cycle % 1000 == 0) {
                std::cout << "[cycle " << std::setw(5) << cycle << "]"
                          << "  meas="  << std::fixed << std::setprecision(3) << q_meas
                          << "  ideal=" << q_ideal
                          << "  err="   << std::setprecision(4) << (q_ideal - q_meas)
                          << "  gap="   << gap
                          << " | work: " << std::setprecision(0) << work_dt << " us"
                          << std::endl;
            }

        } catch (k_api::KDetailedException& ex) {
            std::cerr << "Refresh error at cycle " << cycle
                      << ": " << ex.what() << std::endl;
            break;
        }

        // Pace to the next absolute target (drift-free).
        t_next += std::chrono::microseconds(static_cast<int64_t>(LOOP_PERIOD_US));
        std::this_thread::sleep_until(t_next);
    }

    // =========================================================================
    // SHUTDOWN
    // =========================================================================
    std::cout << "\nLoop finished. Cleaning up..." << std::endl;

    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    stats.print_summary();
    stats.write_csv("sinusoid_tracking.csv");

    // Close sessions & transports (reverse order, same RAII discipline as the wrapper).
    session_tcp->CloseSession();
    session_udp->CloseSession();
    router_tcp->SetActivationStatus(false);
    router_udp->SetActivationStatus(false);
    transport_tcp->disconnect();
    transport_udp->disconnect();

    std::cout << "Clean shutdown." << std::endl;
    return 0;
}
