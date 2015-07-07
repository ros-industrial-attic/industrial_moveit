/**
 * @file enum_types.h
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
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
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
    enum ConstraintType {Primary,Auxiliary};

  }// namespace constraint_types

  namespace initialization_state
  {
    /**
     * @brief Enum that identifies the state of the solver.
     */
    enum InitializationState {PrimaryOnly, AuxiliaryOnly, PrimaryAndAuxiliary, NothingInitialized};
  }// namespace initialization_state

  typedef constraint_types::ConstraintType ConstraintTypes;
  typedef initialization_state::InitializationState InitializationState;
}// namespace constrained_ik
#endif // ENUM_TYPES_H
