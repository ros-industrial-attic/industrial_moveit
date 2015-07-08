#include <constrained_ik/enum_types.h>
#include <ros/console.h>

using namespace constrained_ik::constraint_types;

const std::string ConstraintType::names_[] = {"Primary", "Auxiliary", "Inactive"};
const std::map<std::string, ConstraintTypes> ConstraintType::name_to_enum_map_ = boost::assign::map_list_of ("primary", Primary) ("auxiliary", Auxiliary) ("inactive", Inactive);

ConstraintTypes ConstraintType::stringToEnum(std::string constraint_type_name)
{
  std::map<std::string, ConstraintTypes>::const_iterator it;
  std::transform(constraint_type_name.begin(), constraint_type_name.end(), constraint_type_name.begin(), ::tolower);
  it = name_to_enum_map_.find(constraint_type_name);
  if (it != name_to_enum_map_.end())
    return it->second;
  else
    ROS_ERROR("No enumerator named %s. Valid names (Not case sensitive): Primary, Auxiliary, Inactive", constraint_type_name.c_str());
}
