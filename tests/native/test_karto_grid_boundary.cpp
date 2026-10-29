// Offline regression using the selected upstream Karto header and library.
// No ROS node, simulator, planner, velocity publisher, or geometry truth input.
#include <karto_sdk/Karto.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

struct Bounds {
  const char *name;
  double xmin, xmax, ymin, ymax;
};

int main(int argc, char **argv) {
  bool stock = false;
  if (argc == 2 && std::string(argv[1]) == "--expect-stock-failure") stock = true;
  else if (argc != 1) {
    std::cerr << "Usage: test_karto_grid_boundary [--expect-stock-failure]\n";
    return 2;
  }
  using namespace karto;
  const double resolution = 0.05;
  const std::vector<Bounds> cases = {
    {"integer_span_symmetric", -.4, .4, -.4, .4},
    {"run6_like_fractional_span", -.4330285492420276, 3.612, -.7379159661809404, .4359},
    {"fractional_round_down_negative_origin", -1.375, -.61, -2.46, -1.632},
    {"fractional_positive_origin", .125, .812, .335, 1.081},
    {"fractional_negative_coordinates", -3.141, -2.415, -2.718, -1.904}
  };
  // Keep upstream sensor registration chatter separate from the JSON evidence.
  std::ostringstream upstream_messages;
  auto *old_stdout = std::cout.rdbuf(upstream_messages.rdbuf());
  auto *laser = LaserRangeFinder::CreateLaserRangeFinder(
    LaserRangeFinder_Custom, Name("inclusive_grid_regression"));
  laser->SetRangeThreshold(9.0);
  laser->SetMinimumRange(.01);
  laser->SetMaximumRange(10.0);
  SensorManager::GetInstance()->RegisterSensor(laser);
  std::cout.rdbuf(old_stdout);

  int failing_cases = 0;
  std::cout << "{\"expect_stock_failure\":" << (stock ? "true" : "false") << ",\"cases\":[";
  for (size_t i = 0; i < cases.size(); ++i) {
    const auto &b = cases[i];
    const double cx = (b.xmin + b.xmax) / 2, cy = (b.ymin + b.ymax) / 2;
    // Both signs of each axis, corners, negative and nonintegral origins/spans.
    const PointVectorDouble world_points = {
      {b.xmin, b.ymin}, {b.xmax, b.ymin}, {b.xmin, b.ymax}, {b.xmax, b.ymax},
      {b.xmin, cy}, {b.xmax, cy}, {cx, b.ymin}, {cx, b.ymax}
    };
    PointVectorDouble local_points;
    RangeReadingsVector ranges;
    for (const auto &p : world_points) {
      local_points.emplace_back(p.GetX() - cx, p.GetY() - cy);
      ranges.push_back(std::hypot(p.GetX() - cx, p.GetY() - cy));
    }
    LocalizedRangeScanWithPoints scan(Name("inclusive_grid_regression"), ranges, local_points);
    scan.SetCorrectedPose(Pose2(cx, cy, 0));
    LocalizedRangeScanVector scans(4, &scan);
    // This is the production public entry point: ComputeDimensions, allocation,
    // AddScan/RayTrace and Update all execute from the selected upstream header.
    std::unique_ptr<OccupancyGrid> grid(OccupancyGrid::CreateFromScans(scans, resolution, 2, .1));
    int invalid = 0, non_occupied = 0;
    for (const auto &p : world_points) {
      const auto index = grid->WorldToGrid(p);
      if (!grid->IsValidGridIndex(index)) ++invalid;
      else if (*grid->GetDataPointer(index) != GridStates_Occupied) ++non_occupied;
    }
    const auto center = grid->WorldToGrid(Vector2<kt_double>(cx, cy));
    const bool center_free = grid->IsValidGridIndex(center) && *grid->GetDataPointer(center) == GridStates_Free;
    const auto offset = grid->GetCoordinateConverter()->GetOffset();
    const bool origin_preserved = std::abs(offset.GetX() - b.xmin) < 1e-12 &&
      std::abs(offset.GetY() - b.ymin) < 1e-12;
    const bool passed = invalid == 0 && non_occupied == 0 && center_free && origin_preserved;
    if (!passed) ++failing_cases;
    if (i) std::cout << ',';
    std::cout << "{\"name\":\"" << b.name << "\",\"width\":" << grid->GetWidth()
      << ",\"height\":" << grid->GetHeight() << ",\"boundary_endpoints\":" << world_points.size()
      << ",\"invalid_endpoints\":" << invalid << ",\"valid_non_occupied_endpoints\":" << non_occupied
      << ",\"center_free\":" << (center_free ? "true" : "false")
      << ",\"origin_preserved\":" << (origin_preserved ? "true" : "false")
      << ",\"passed\":" << (passed ? "true" : "false") << '}';
  }
  const bool expected = stock ? failing_cases == static_cast<int>(cases.size()) : failing_cases == 0;
  std::cout << "],\"failing_cases\":" << failing_cases
    << ",\"expectation_satisfied\":" << (expected ? "true" : "false") << "}\n";
  return expected ? 0 : 1;
}
