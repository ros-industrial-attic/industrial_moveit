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

    /**
     * @brief A structure for ConstraintTypes enum providing addition functionality
     * like enumToString and stringToEnum.
     */
    struct ConstraintType
    {
      /**
       * @brief ConstraintType constructor
       * @param constraint_type, ConstraintTypes Enumerator value
       */
      ConstraintType(ConstraintTypes constraint_type):type_(constraint_type) {}
      ConstraintType() {}

      ~ConstraintType(){}

      /**
       * @brief A string representation of the enumerator
       * @return std::string
       */
      inline std::string toString() const { return names_[type_]; }

      /**
       * @brief Gets the enumerator
       * @return ConstraintTypes
       */
      inline ConstraintTypes getType() const { return type_; }

      /**
       * @brief Sets the enumerator type
       * @param constraint_type, Enumerator value (Primary, Auxiliary, Inactive)
       */
      inline void setType(ConstraintTypes constraint_type) { type_ = constraint_type; }

      /**
       * @brief Sets the enumerator type given the string representation.
       * @param constraint_type_name, Enumerator string value (Primary, Auxiliary, Inactive)
       */
      inline void setType(std::string constraint_type_name) { type_ = stringToEnum(constraint_type_name); }

      /**
       * @brief Coverts ConstraintTypes value to its string representation.
       * @param constraint_type, Enumerator value (Primary, Auxiliary, Inactive)
       * @return std::string
       */
      inline static std::string enumToString(ConstraintTypes constraint_type) { return names_[constraint_type]; }

      /**
       * @brief Converts string to ConstraintTypes value
       * @param constraint_type_name, Enumerator string value (Primary, Auxiliary, Inactive)
       * @return ConstraintTypes
       */
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
