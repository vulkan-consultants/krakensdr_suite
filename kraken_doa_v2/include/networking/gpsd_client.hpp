#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <cstdint>

// GpsdClient
// ----------
// Lightweight client for gpsd (the standard Linux GPS daemon). Speaks the gpsd
// JSON protocol directly over a TCP socket (default 127.0.0.1:2947) so we don't
// take a hard build dependency on libgps. It WATCHes the daemon and keeps a
// single thread-safe snapshot of the current fix that the rest of kraken_doa
// reads via get().
//
// Heading policy (matches the DOA_value.html / API contract):
//   * If the GPS reports a compass/IMU heading (gpsd ATT report), that magnetic
//     heading is authoritative -> heading_source() == "Compass".
//   * Otherwise the heading is inferred from movement: gpsd's course-over-ground
//     (TPV "track"), with a great-circle bearing between fixes as a fallback for
//     receivers that don't emit "track" -> heading_source() == "GPS".
//
// When gpsd is unreachable or no device is attached, the snapshot stays at its
// zero defaults (connected=false) and every consumer renders "no GPS" / 0's.

// Immutable snapshot of GPS state. Trivially copyable; copied out under lock.
struct GpsFix {
    bool    connected = false;       // gpsd socket is up
    bool    device_present = false;  // gpsd reports at least one GPS device
    int     mode = 0;                // gpsd TPV mode: 0/1 = no fix, 2 = 2D, 3 = 3D
    double  lat = 0.0;               // degrees (0 when no fix)
    double  lon = 0.0;               // degrees (0 when no fix)
    double  alt = 0.0;               // metres
    double  speed = 0.0;             // ground speed, m/s
    double  track = 0.0;             // course over ground (movement heading), deg true
    bool    track_valid = false;     // true once a trustworthy movement heading exists
    double  compass_heading = 0.0;   // heading from a compass/IMU (gpsd ATT), deg
    bool    has_compass = false;     // true once a compass heading has been seen
    int     sats_used = 0;           // satellites used in the solution
    int     sats_visible = 0;        // satellites in view
    int64_t timestamp_ms = 0;        // GPS fix time, epoch ms (0 if unknown)
    int64_t receipt_monotonic_ns = 0; // local monotonic epoch when TPV time was received

    bool has_fix() const { return mode >= 2; }

    // Heading to publish: compass if the device has one, else movement-derived.
    double heading() const {
        if (has_compass) return compass_heading;
        if (track_valid) return track;
        return 0.0;
    }

    // Which sensor the published heading came from (DOA_value.html field 13).
    const char* heading_source() const {
        if (has_compass) return "Compass";
        if (track_valid) return "GPS";
        return "None";
    }

    // Human-readable fix state for the UI status panel.
    const char* fix_str() const {
        if (!connected) return "Disconnected";
        if (!device_present) return "No GPS";
        if (mode >= 3) return "3D Fix";
        if (mode == 2) return "2D Fix";
        return "No Fix";
    }
};

class GpsdClient {
public:
    // Spawn the background connection thread. Idempotent; no-op if disabled.
    void start();
    // Signal the thread to stop and join it. Safe to call if never started.
    void stop();

    // Thread-safe snapshot of the current fix.
    GpsFix get() const;

private:
    void run();                                   // thread body: connect + read loop
    void process_line(const std::string& line);   // parse one gpsd JSON object
    void reset_disconnected();                     // clear snapshot to the disconnected state

    std::thread        thread_;
    std::atomic<bool>  running_{false};
    mutable std::mutex mutex_;
    GpsFix             fix_;

    // State for the movement-heading fallback (used only when "track" is absent).
    double last_lat_ = 0.0;
    double last_lon_ = 0.0;
    bool   have_last_pos_ = false;
};
