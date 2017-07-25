#pragma once

#include <ros/time.h>

namespace franka_hw {

class TriggerRate {
 public:
  explicit TriggerRate(double rate = 30.0);
  bool operator()();

 private:
  ros::Time time_stamp_;
  double period_;
};

};  // namespace franka_hw