#ifndef MIVIA_ROVER_PLATFORM__RT_CONTROL_LOOP_PROFILER_HPP_
#define MIVIA_ROVER_PLATFORM__RT_CONTROL_LOOP_PROFILER_HPP_

#include <time.h>
#include <sys/resource.h>
#include <cstdint>
#include <vector>
#include <fstream>
#include <string>

namespace mivia_rover_platform
{

// Struttura dati per le metriche di un singolo ciclo di controllo RT
struct RtControlLoopSample {
  std::uint64_t t_start_read_ns;
  std::uint64_t dt_period_ns;
  std::uint64_t read_exec_ns;
  std::uint64_t rx_data_age_ns;   // t_end_read - t_rx_origin (latenza End-to-End RX)
  std::uint64_t write_latency_ns; // t_end_write - t_start_write (latenza End-to-End TX)
  std::uint64_t cycle_cpu_C_ns;   // Puro tempo CPU (CLOCK_THREAD_CPUTIME_ID) -> Per stima MOET/WCET
  std::uint64_t cycle_wall_w_ns;  // Durata Wall-Clock (CLOCK_MONOTONIC_RAW) -> Per Response Time
  std::uint32_t can_frames_popped;
  std::int32_t  minor_faults;     // Minor page faults in QUESTO ciclo (deve essere 0 con mlockall)
  std::int32_t  major_faults;     // Major page faults in QUESTO ciclo
  std::int32_t  invol_cs;         // Preemption subite in QUESTO ciclo (deve essere 0 con SCHED_FIFO/isolcpus)
};

class RtControlLoopProfiler {
public:
  void init(std::size_t max_samples, const std::string & filepath) {
    filepath_ = filepath;
    buffer_.clear();
    buffer_.reserve(max_samples);
    // Pre-touch delle pagine RAM per forzare il mapping fisico ed evitare fault a runtime
    buffer_.resize(max_samples);
    buffer_.clear();
    prev_start_ns_ = 0ULL;
  }

  ~RtControlLoopProfiler() {
    flush_to_disk();
  }

  static inline std::uint64_t now_monotonic_ns() noexcept {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
  }

  static inline std::uint64_t now_thread_cpu_ns() noexcept {
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
  }

  inline void begin_cycle() noexcept {
    t_read_start_ = now_monotonic_ns();
    t_cpu_start_  = now_thread_cpu_ns();
    getrusage(RUSAGE_THREAD, &ru_start_);
    valid_cycle_ = true;
  }

  inline void end_read(std::uint64_t latest_rx_origin_ns, std::uint32_t popped_count) noexcept {
    t_read_end_ = now_monotonic_ns();
    rx_age_ns_ = (latest_rx_origin_ns > 0ULL && t_read_end_ >= latest_rx_origin_ns) ?
                 (t_read_end_ - latest_rx_origin_ns) : 0ULL;
    popped_ = popped_count;
  }

  inline void begin_write() noexcept {
    t_write_start_ = now_monotonic_ns();
  }

  inline void end_write_and_record() noexcept {

    if (!valid_cycle_) return;
    const std::uint64_t t_write_end = now_monotonic_ns();
    const std::uint64_t t_cpu_end   = now_thread_cpu_ns();
    struct rusage ru_end;
    getrusage(RUSAGE_THREAD, &ru_end);

    if (buffer_.size() >= buffer_.capacity()) return; // Evita allocazioni dinamiche a runtime

    const std::uint64_t dt_ns = (prev_start_ns_ == 0ULL) ? 0ULL : (t_read_start_ - prev_start_ns_);
    prev_start_ns_ = t_read_start_;

    buffer_.push_back(RtControlLoopSample{
      t_read_start_,
      dt_ns,
      t_read_end_ - t_read_start_,
      rx_age_ns_,
      t_write_end - t_write_start_,
      t_cpu_end - t_cpu_start_,
      t_write_end - t_read_start_,
      popped_,
      static_cast<std::int32_t>(ru_end.ru_minflt - ru_start_.ru_minflt),
      static_cast<std::int32_t>(ru_end.ru_majflt - ru_start_.ru_majflt),
      static_cast<std::int32_t>(ru_end.ru_nivcsw - ru_start_.ru_nivcsw)
    });
    valid_cycle_ = false;
  }

  void flush_to_disk() {
    if (buffer_.empty() || filepath_.empty()) return;
    std::ofstream out(filepath_, std::ios::out | std::ios::trunc);
    out << "t_start_ns,dt_period_ns,read_exec_ns,rx_data_age_ns,write_latency_ns,"
           "cycle_cpu_C_ns,cycle_wall_w_ns,can_popped,minor_faults,major_faults,invol_cs\n";
    for (const auto & s : buffer_) {
      out << s.t_start_read_ns << "," << s.dt_period_ns << "," << s.read_exec_ns << ","
          << s.rx_data_age_ns << "," << s.write_latency_ns << "," << s.cycle_cpu_C_ns << ","
          << s.cycle_wall_w_ns << "," << s.can_frames_popped << "," << s.minor_faults << ","
          << s.major_faults << "," << s.invol_cs << "\n";
    }
    out.close();
    buffer_.clear();
  }

private:
  std::vector<RtControlLoopSample> buffer_;
  std::string filepath_;
  std::uint64_t prev_start_ns_{0ULL}, t_read_start_{0ULL}, t_read_end_{0ULL};
  std::uint64_t t_write_start_{0ULL}, t_cpu_start_{0ULL}, rx_age_ns_{0ULL};
  std::uint32_t popped_{0U};
  struct rusage ru_start_{};
  bool valid_cycle_{false};
};

} // namespace mivia_rover_platform
#endif // MIVIA_ROVER_PLATFORM__RT_CONTROL_LOOP_PROFILER_HPP_