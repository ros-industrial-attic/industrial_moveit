/**
 * @file enum_types.cpp
 * @brief Enum that identifies a constraint type.
 *
 * Primary - These constraints must be satisfied
 * Auxiliary - These constraints try to manipulate the null space to be
 *             satisfied.
 *
 * @author Levi Armstrong
 * @date May 4, 2015
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2015, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
