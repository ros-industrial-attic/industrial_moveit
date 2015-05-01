/*
 * enum_types.h
 *
 *  Created on: May 4, 2015
 *      Author: Levi Armstrong
 */
/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ENUM_TYPES_H
#define ENUM_TYPES_H

namespace constrained_ik
{
namespace constraint_types
{
  /**
   * @brief Enum that identifies a constraint type.
   * Primary - These constraints must be satisfied
   * Auxiliary - These constraints try to manipulate the null space to be
   *             satisfied.
   */
  enum ConstraintType {primary,auxiliary};
}// namespace constraint_types
}// namespace constrained_ik
#endif // ENUM_TYPES_H
