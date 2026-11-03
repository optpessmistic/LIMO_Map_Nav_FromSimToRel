#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using Steady = std::chrono::steady_clock;
using System = std::chrono::system_clock;
using Nanoseconds = std::chrono::nanoseconds;
static_assert(Steady::is_steady);

double seconds(Nanoseconds value) {return static_cast<double>(value.count()) / 1e9;}
template<class Duration> double as_seconds(Duration value) {
  return seconds(std::chrono::duration_cast<Nanoseconds>(value));
}

struct ClockPair {
  std::int64_t steady_ns;
  std::int64_t realtime_ns;
  std::int64_t bracket_ns;
  std::int64_t offset_ns() const {return realtime_ns-steady_ns;}
};

ClockPair clocks() {
  const auto before=Steady::now();
  const auto realtime=System::now();
  const auto after=Steady::now();
  return {std::chrono::duration_cast<Nanoseconds>((before+(after-before)/2).time_since_epoch()).count(),
    std::chrono::duration_cast<Nanoseconds>(realtime.time_since_epoch()).count(),
    std::chrono::duration_cast<Nanoseconds>(after-before).count()};
}

struct Sample {
  double start_s;
  double end_s;
  double sleep_s;
  bool returned;
  ClockPair before;
  ClockPair after;
};

struct Result {
  std::string name;
  rcl_clock_type_t type;
  std::vector<Sample> samples;
  std::string error;
};

std::string json_string(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c:value) {
    if (c=='"' || c=='\\') out << '\\' << c;
    else if (c=='\n') out << "\\n";
    else if (c=='\r') out << "\\r";
    else if (c=='\t') out << "\\t";
    else if (c<0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    else out << c;
  }
  out << '"';
  return out.str();
}

void measure(rclcpp::Rate &rate, Result &result, std::shared_future<void> gate,
             const Steady::time_point &start, const Steady::time_point &deadline) {
  result.samples.reserve(4000);
  gate.wait();
  try {
    while (Steady::now()<deadline) {
      const auto clock_before=clocks();
      const auto before=Steady::now();
      const bool returned=rate.sleep();
      const auto after=Steady::now();
      const auto clock_after=clocks();
      result.samples.push_back({as_seconds(before-start),as_seconds(after-start),
                               as_seconds(after-before),returned,clock_before,clock_after});
      if (result.samples.size()>20000) throw std::runtime_error("bounded sample count exceeded");
    }
  } catch(const std::exception &error) {result.error=error.what();}
}

void csv(const std::filesystem::path &path, const Result &result, const ClockPair &baseline) {
  std::ofstream out(path);
  out.exceptions(std::ios::badbit|std::ios::failbit);
  out << "index,start_steady_elapsed_s,end_steady_elapsed_s,sleep_steady_s,sleep_returned,"
      << "before_realtime_ns,before_steady_ns,before_bracket_ns,after_realtime_ns,after_steady_ns,after_bracket_ns,"
      << "realtime_minus_steady_before_ns,realtime_minus_steady_after_ns,offset_change_from_baseline_after_s,offset_jump_during_sample_s\n";
  out << std::setprecision(12);
  for (std::size_t i=0;i<result.samples.size();++i) {
    const auto &s=result.samples[i];
    out << i << ',' << s.start_s << ',' << s.end_s << ',' << s.sleep_s << ',' << s.returned << ','
        << s.before.realtime_ns << ',' << s.before.steady_ns << ',' << s.before.bracket_ns << ','
        << s.after.realtime_ns << ',' << s.after.steady_ns << ',' << s.after.bracket_ns << ','
        << s.before.offset_ns() << ',' << s.after.offset_ns() << ','
        << seconds(Nanoseconds(s.after.offset_ns()-baseline.offset_ns())) << ','
        << seconds(Nanoseconds(s.after.offset_ns()-s.before.offset_ns())) << '\n';
  }
}

std::string summary(const Result &result, const ClockPair &baseline) {
  double maximum=0, minimum=1e30, sum=0;
  std::int64_t min_offset=baseline.offset_ns(), max_offset=min_offset, max_bracket=baseline.bracket_ns;
  std::size_t over100=0, returned_false=0, jumps=0;
  std::vector<double> durations;
  for (const auto &s:result.samples) {
    maximum=std::max(maximum,s.sleep_s);minimum=std::min(minimum,s.sleep_s);sum+=s.sleep_s;
    over100+=s.sleep_s>.1; returned_false+=!s.returned;
    jumps+=std::abs(s.after.offset_ns()-s.before.offset_ns())>100000000;
    min_offset=std::min({min_offset,s.before.offset_ns(),s.after.offset_ns()});
    max_offset=std::max({max_offset,s.before.offset_ns(),s.after.offset_ns()});
    max_bracket=std::max({max_bracket,s.before.bracket_ns,s.after.bracket_ns});
    durations.push_back(s.sleep_s);
  }
  std::sort(durations.begin(),durations.end());
  std::ostringstream out;out << std::setprecision(12);
  out << "{\"name\":" << json_string(result.name) << ",\"clock_type\":" << int(result.type)
      << ",\"clock_name\":" << json_string(result.type==RCL_SYSTEM_TIME?"RCL_SYSTEM_TIME":result.type==RCL_STEADY_TIME?"RCL_STEADY_TIME":"other")
      << ",\"samples\":" << result.samples.size() << ",\"sleep_returned_false\":" << returned_false
      << ",\"sleep_gt_0_1_s_count\":" << over100 << ",\"within_sleep_offset_jump_gt_0_1_s_count\":" << jumps
      << ",\"max_pair_bracket_s\":" << seconds(Nanoseconds(max_bracket))
      << ",\"observed_realtime_minus_steady_range_s\":" << seconds(Nanoseconds(max_offset-min_offset));
  if (!durations.empty()) {
    out << ",\"minimum_sleep_s\":" << minimum << ",\"maximum_sleep_s\":" << maximum
        << ",\"mean_sleep_s\":" << sum/durations.size()
        << ",\"median_sleep_s\":" << durations[durations.size()/2]
        << ",\"p99_sleep_s\":" << durations[(durations.size()-1)*99/100]
        << ",\"first_sample_start_s\":" << result.samples.front().start_s
        << ",\"last_sample_end_s\":" << result.samples.back().end_s;
  }
  out << ",\"long_sleeps\":[";
  bool comma=false;
  for (std::size_t i=0;i<result.samples.size();++i) {
    const auto &s=result.samples[i];
    if (s.sleep_s<=.1) continue;
    if (comma) out << ',';
    comma=true;
    out << "{\"index\":" << i << ",\"start_s\":" << s.start_s << ",\"end_s\":" << s.end_s
        << ",\"sleep_s\":" << s.sleep_s << ",\"offset_jump_during_sample_s\":"
        << seconds(Nanoseconds(s.after.offset_ns()-s.before.offset_ns())) << '}';
  }
  out << "],\"error\":" << (result.error.empty()?"null":json_string(result.error)) << '}';
  return out.str();
}

int main(int argc,char **argv) {
  if (argc!=2) {std::cerr << "usage: rate_probe NEW_OUTPUT_DIRECTORY\n";return 2;}
  const std::filesystem::path output(argv[1]);
  try {
    if (!std::filesystem::create_directory(output)) throw std::runtime_error("refuse existing output directory");
    // Initializes only the shared context needed by Rate::sleep; no Node exists.
    rclcpp::init(0,nullptr,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
    rclcpp::Rate rate(50.0);
    rclcpp::WallRate wall_rate(50.0);
    Result a{"Rate50",rate.get_type(),{},{}},b{"WallRate50",wall_rate.get_type(),{},{}};
    std::promise<void> promise;
    const auto gate=promise.get_future().share();
    Steady::time_point start,deadline;
    std::thread first(measure,std::ref(rate),std::ref(a),gate,std::cref(start),std::cref(deadline));
    std::thread second(measure,std::ref(wall_rate),std::ref(b),gate,std::cref(start),std::cref(deadline));
    const auto baseline=clocks();
    start=Steady::now();deadline=start+std::chrono::seconds(65);
    std::cout << "START two independent 50Hz Rate threads; steady duration 65s; ROS nodes=0\n" << std::flush;
    promise.set_value();
    first.join();second.join();
    const auto elapsed=as_seconds(Steady::now()-start);
    rclcpp::shutdown();
    csv(output/"rate-sleeps.csv",a,baseline);
    csv(output/"wallrate-sleeps.csv",b,baseline);
    const std::string report="{\"schema_version\":1,\"requested_steady_seconds\":65,\"actual_steady_seconds\":"+
        std::to_string(elapsed)+",\"ros_nodes_created\":0,\"system_time_modified\":false,\"rate\":"+summary(a,baseline)+
        ",\"wallrate\":"+summary(b,baseline)+
        ",\"scope\":\"Same-host concurrent measurement; sleep durations include scheduler delays. Realtime/steady offsets are midpoint pairs with recorded brackets. Natural clock corrections are observed, never induced. This is not a SLAM/Gazebo success test.\"}\n";
    std::ofstream file(output/"summary.json");file.exceptions(std::ios::badbit|std::ios::failbit);file<<report;file.close();
    std::cout << report << std::flush;
    return a.error.empty() && b.error.empty() && !a.samples.empty() && !b.samples.empty()?0:1;
  } catch (const std::exception &error) {std::cerr << "ERROR " << error.what() << '\n';return 1;}
}
