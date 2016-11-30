/**
 * @file master_ik.h
 * @brief This the master IK solver which loads all constraints from a yaml file.
 *
 * @author Levi Armstrong
 * @date Febuary 15, 2015
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
#ifndef MASTER_IK_H
#define MASTER_IK_H

#include "constrained_ik/constrained_ik.h"

namespace constrained_ik
{

/**
 * @brief Master IK Solver, This will add contraints defined as plugins in a yaml file.
 * @todo Remove this class it is no longer needed.
 */
class MasterIK : public Constrained_IK
{
public:
  /**
   * @brief MasterIK
   * @param group_name
   */
  MasterIK(std::string group_name): Constrained_IK()
  {
    std::string constraint_param = "constrained_ik_solver/" + group_name + "/constraints";
    addConstraintsFromParamServer(constraint_param);
  }
}; // class MasterIK

} // namespace constrained_ik

#endif // MASTER_IK_H

