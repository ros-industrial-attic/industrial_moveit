#ifndef ROBOT_STATE_TEST_CASES_H
#define ROBOT_STATE_TEST_CASES_H

#include <moveit/robot_state/robot_state.h>

class RobotStateTestCases
{
public:
  RobotStateTestCases(moveit::core::RobotModelConstPtr model);

  void load(const std::string& filename);

  void save(const std::string& filename) const;

  void append(const moveit::core::RobotState& state);

  const std::vector<moveit::core::RobotState>& states() const
  {
    return states_;
  }

private:
  std::vector<moveit::core::RobotState> states_;
  moveit::core::RobotModelConstPtr model_;
};

#endif // ROBOT_STATE_TEST_CASES_H
