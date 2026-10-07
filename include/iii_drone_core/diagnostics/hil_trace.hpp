#pragma once

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unistd.h>

namespace iii_drone::diagnostics {

/** Small, opt-in JSONL trace for bounded split-host HIL diagnostics.
 *
 * Events are built on hot paths (reference streams, mode status, tree
 * transitions), so with tracing off (III_HIL_DIAGNOSTIC_TRACE unset, read
 * once) an event formats nothing.
 */
class HilTrace {
public:
    class Event {
    public:
        explicit Event(std::string type) : enabled_(HilTrace::enabled()) {
            if (enabled_) {
                type_ = std::move(type);
            }
        }

        Event &text(const std::string &key, const std::string &value) {
            if (enabled_) {
                field(key, escape(value), false);
            }
            return *this;
        }

        Event &number(const std::string &key, uint64_t value) {
            if (enabled_) {
                field(key, std::to_string(value), true);
            }
            return *this;
        }

        Event &signed_number(const std::string &key, int64_t value) {
            if (enabled_) {
                field(key, std::to_string(value), true);
            }
            return *this;
        }

        Event &decimal(const std::string &key, double value) {
            if (enabled_) {
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                field(key, stream.str(), true);
            }
            return *this;
        }

        Event &boolean(const std::string &key, bool value) {
            if (enabled_) {
                field(key, value ? "true" : "false", true);
            }
            return *this;
        }

        void commit() {
            if (committed_) {
                return;
            }
            committed_ = true;
            if (enabled_) {
                HilTrace::write(type_, fields_);
                fields_.clear();
            }
        }

        ~Event() { commit(); }

    private:
        void field(const std::string &key, const std::string &value, bool raw) {
            if (!fields_.empty()) {
                fields_ += ',';
            }
            fields_ += '"' + escape(key) + "\":";
            if (raw) {
                fields_ += value;
            } else {
                fields_ += '"' + value + '"';
            }
        }

        bool enabled_;
        std::string type_;
        std::string fields_;
        bool committed_{false};
    };

    static Event event(const std::string &type) { return Event(type); }

    /** Whether III_HIL_DIAGNOSTIC_TRACE names a trace file. */
    static bool enabled() { return !trace_path().empty(); }

private:
    static const std::string &trace_path() {
        static const std::string path = []() {
            const char *value = std::getenv("III_HIL_DIAGNOSTIC_TRACE");
            return std::string(value != nullptr ? value : "");
        }();
        return path;
    }

    static uint64_t process_start_monotonic_ns() {
        static const uint64_t value = []() -> uint64_t {
            std::ifstream input("/proc/self/stat");
            std::string stat;
            std::getline(input, stat);
            const auto command_end = stat.rfind(')');
            if (command_end == std::string::npos) {
                return uint64_t{0};
            }

            std::istringstream fields(stat.substr(command_end + 2));
            std::string field;
            for (int index = 0; index < 19; ++index) {
                if (!(fields >> field)) {
                    return uint64_t{0};
                }
            }
            uint64_t start_ticks = 0;
            if (!(fields >> start_ticks)) {
                return uint64_t{0};
            }
            const long ticks_per_second = ::sysconf(_SC_CLK_TCK);
            if (ticks_per_second <= 0) {
                return uint64_t{0};
            }
            return (start_ticks * 1000000000ULL) / static_cast<uint64_t>(ticks_per_second);
        }();
        if (value != 0) {
            return value;
        }
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    static std::string escape(const std::string &value) {
        std::string escaped;
        escaped.reserve(value.size());
        for (const char character : value) {
            switch (character) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += character; break;
            }
        }
        return escaped;
    }

    static void write(const std::string &type, const std::string &fields) {
        static std::atomic<uint64_t> event_count{0};
        constexpr uint64_t kMaxEvents = 200000;
        if (event_count.fetch_add(1, std::memory_order_relaxed) >= kMaxEvents) {
            return;
        }

        const std::string &path = trace_path();
        if (path.empty()) {
            return;
        }

        static std::mutex mutex;
        static std::ofstream output;
        std::lock_guard<std::mutex> lock(mutex);
        if (!output.is_open()) {
            try {
                const std::filesystem::path trace_path(path);
                std::filesystem::create_directories(trace_path.parent_path());
                output.open(trace_path, std::ios::out | std::ios::app);
            } catch (...) {
                return;
            }
        }
        if (!output.good()) {
            return;
        }

        const auto monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
        const auto process_generation = process_start_monotonic_ns();
        const auto thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
        const char *process = std::getenv("III_HIL_DIAGNOSTIC_PROCESS");
        output << "{\"monotonic_ns\":" << monotonic_ns
               << ",\"process_generation\":" << process_generation
               << ",\"pid\":" << static_cast<long long>(::getpid())
               << ",\"thread_id\":" << thread_id
               << ",\"process\":\"" << escape(process == nullptr ? "unknown" : process)
               << "\",\"event\":\"" << escape(type) << "\""
               << (fields.empty() ? "" : "," + fields) << "}\n";
        output.flush();
    }
};

}  // namespace iii_drone::diagnostics
