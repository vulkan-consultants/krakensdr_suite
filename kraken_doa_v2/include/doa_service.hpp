#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "doa_logger.hpp"

// Phase-2 rf.service.v1 adapter.  This is intentionally independent of the
// legacy binary DoA stream and /DOA_value.html compatibility interface.
std::vector<std::string> build_doa_service_messages(const std::vector<DoaRecord>& records);
