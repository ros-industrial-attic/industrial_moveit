#include <industrial_moveit_benchmarking/robot_state_test_cases.h>
#include <moveit/robot_state/conversions.h>
#include <ros/console.h>
#include <fstream>

// Utilities for saving states to/from files; based on similar MoveIt components but
// with an API that allows multiple joint states in one file
namespace util
{

bool robotStateToStream(const moveit::core::RobotState& state, std::ostream& os)
{
  for (std::size_t i = 0; i < state.getVariableCount(); ++i)
  {
    os << state.getVariablePositions()[i] << " ";
  }
  os << std::endl;
  return static_cast<bool>(os);
}

bool streamToRobotState(std::istream& is, moveit::core::RobotState& state)
{
  for (std::size_t i = 0; i < state.getVariableCount(); ++i)
  {
    is >> state.getVariablePositions()[i];
  }

  state.update(true);

  return static_cast<bool>(is);
}

} // end util namespace

RobotStateTestCases::RobotStateTestCases(moveit::core::RobotModelConstPtr model)
  : model_(model)
{
}

void RobotStateTestCases::load(const std::string &filename)
{
  std::ifstream ifh (filename.c_str());
  if (!ifh)
    throw std::invalid_argument("Could not open file for reading: " + filename);

  std::string line;
  while (std::getline(ifh, line))
  {
    std::istringstream ss (line);

    moveit::core::RobotState state (model_);
    if (!util::streamToRobotState(ss, state))
    {
      ROS_ERROR("Could not parse robot state");
      break;
    }
    append(state);
  }
}

void RobotStateTestCases::save(const std::string &filename) const
{
  std::ofstream ofh (filename.c_str());
  if (!ofh)
    throw std::invalid_argument("Could not open file for writing: " + filename);

  for (const auto& state: states_)
  {
    if (!util::robotStateToStream(state, ofh))
    {
      ROS_ERROR("Could not serialize robot state to file");
      break;
    }
  }
}

void RobotStateTestCases::append(const moveit::core::RobotState &state)
{
  states_.push_back(state);
}
