#include "doa_service.hpp"
#include "globals.hpp"
#include "networking/gpsd_client.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace {
std::atomic<uint64_t> sequence{0};
std::string esc(const std::string& s){std::string o; for(char c:s){if(c=='"'||c=='\\')o+='\\';o+=c;}return o;}
}

std::vector<std::string> build_doa_service_messages(const std::vector<DoaRecord>& records) {
    std::vector<std::string> out; out.reserve(records.size());
    const GpsFix fix = gps_client.get();
    const int64_t now_mono = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    for (const auto& r : records) {
        const int64_t mono = r.source_monotonic_ns ? r.source_monotonic_ns : now_mono;
        const int64_t sys_ns = (r.source_stamp_ms ? r.source_stamp_ms : r.timestamp_ms) * 1000000LL;
        bool gnss_valid = fix.has_fix() && fix.timestamp_ms > 0 && fix.receipt_monotonic_ns > 0;
        int64_t gnss_ns = 0;
        uint64_t sigma_ns = 0;
        const char* quality = "unsynced";
        if (gnss_valid) {
            gnss_ns = fix.timestamp_ms * 1000000LL + (mono - fix.receipt_monotonic_ns);
            const int64_t age = std::llabs(now_mono - fix.receipt_monotonic_ns);
            sigma_ns = 50000000ULL + static_cast<uint64_t>(age / 1000); // 50 ms floor + 1 us/ms holdover
            quality = age < 2000000000LL ? "locked" : "holdover";
        }
        const uint64_t seq = ++sequence;
        std::ostringstream id; id << "kraken-doa-" << r.decimator_id << "-" << r.source_stamp_ms << "-" << seq;
        std::ostringstream p;
        p << std::setprecision(12)
          << "{\"schema\":\"rf.service.v1\",\"type\":\"doa_estimate\",\"source\":\"kraken_doa_v2\",\"sequence\":" << seq
          << ",\"time\":{\"system_utc_ns\":" << sys_ns << ",\"monotonic_ns\":" << mono
          << ",\"gnss_ns\":" << gnss_ns << ",\"gnss_valid\":" << (gnss_valid?"true":"false")
          << ",\"quality\":\"" << quality << "\",\"sigma_ns\":" << sigma_ns << "},\"payload\":{"
          << "\"observation_id\":\"" << esc(id.str()) << "\",\"decimator_id\":" << r.decimator_id
          << ",\"center_frequency_hz\":" << r.freq_hz << ",\"bandwidth_hz\":" << r.bandwidth_hz
          << ",\"azimuth_deg\":" << r.app_bearing << ",\"elevation_deg\":" << r.elevation_deg
          << ",\"doa_sigma_deg\":" << r.doa_sigma_deg << ",\"confidence\":" << r.confidence
          << ",\"signal_power_db\":" << r.power_db << ",\"snr_db\":0.0"
          << ",\"calibration_valid\":" << (!doa_is_calibrating()?"true":"false")
          << ",\"squelch_open\":" << (r.squelch_open?"true":"false") << ",\"decimation\":" << r.decimation
          << ",\"flags\":0}}";
        out.push_back(p.str());
    }
    return out;
}
