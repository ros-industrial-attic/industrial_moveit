#include <constrained_ik/enum_types.h>
#include <ros/console.h>

using namespace constrained_ik::constraint_types;

const std::string ConstraintType::names_[] = {"Primary", "Auxiliary", "Inactive"};
const std::map<std::string, ConstraintTypes> ConstraintType::name_to_enum_map_ = boost::assign::map_list_of ("Primary", Primary) ("Auxiliary", Auxiliary) ("Inactive", Inactive);

ConstraintTypes ConstraintType::stringToEnum(std::string constraint_type_name)
{
  std::map<std::string, ConstraintTypes>::const_iterator it;
  it = name_to_enum_map_.find(constraint_type_name);
  if (it != name_to_enum_map_.end())
    return it->second;
  else
    ROS_ERROR("No enumerator named %s. Valid names: Primary, Auxiliary, Inactive", constraint_type_name.c_str());
}
