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
#include <boost/assign/list_of.hpp>
#include <map>

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
    enum ConstraintTypes {Primary,Auxiliary,Inactive};
    struct ConstraintType
    {
      ConstraintType(ConstraintTypes constraint_type):type_(constraint_type) {}
      ConstraintType() {}

      ~ConstraintType(){}

      inline std::string toString() const { return names_[type_]; }

      inline ConstraintTypes getType() const { return type_; }

      inline void setType(ConstraintTypes constraint_type) { type_ = constraint_type; }

      inline void setType(std::string constraint_type_name) { type_ = stringToEnum(constraint_type_name); }

      inline static std::string enumToString(ConstraintTypes constraint_type) { return names_[constraint_type]; }

      static ConstraintTypes stringToEnum(std::string constraint_type_name);

    protected:
      ConstraintTypes type_;
      static const std::string names_[];
      static const std::map<std::string, ConstraintTypes> name_to_enum_map_;
    };
  }// namespace constraint_types

  namespace initialization_state
  {
    /**
     * @brief Enum that identifies the state of the solver.
     */
    enum InitializationState {PrimaryOnly, AuxiliaryOnly, PrimaryAndAuxiliary, NothingInitialized};
  }// namespace initialization_state

  typedef constraint_types::ConstraintTypes ConstraintTypes;
  typedef initialization_state::InitializationState InitializationState;
}// namespace constrained_ik
#endif // ENUM_TYPES_H
