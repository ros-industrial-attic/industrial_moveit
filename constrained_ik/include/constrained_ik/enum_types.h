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
#ifndef ENUM_TYPES_H
#define ENUM_TYPES_H
#include <boost/assign/list_of.hpp>
#include <map>

namespace constrained_ik
{
  namespace constraint_types
  {
    /** @brief Enum that identifies a constraint type. */
    enum ConstraintTypes
    {
      Primary, /**< These constraints must be satisfied .*/
      Auxiliary, /**< These constraints try to manipulate the null space to be satisfied. */
      Inactive, /**< These constraints have been disabled */
    };

    /**
     * @brief A structure for ConstraintTypes enum providing addition functionality
     * like enumToString and stringToEnum.
     */
    struct ConstraintType
    {
      /**
       * @brief ConstraintType constructor
       * @param constraint_type enumerator value (Primary, Auxiliary, Inactive)
       */
      ConstraintType(ConstraintTypes constraint_type):type_(constraint_type) {}
      ConstraintType() {}

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
       * @param constraint_type enumerator value (Primary, Auxiliary, Inactive)
       */
      inline void setType(ConstraintTypes constraint_type) { type_ = constraint_type; }

      /**
       * @brief Sets the enumerator type given the string representation.
       * @param constraint_type_name enumerator string value (Primary, Auxiliary, Inactive)
       */
      inline void setType(std::string constraint_type_name) { type_ = stringToEnum(constraint_type_name); }

      /**
       * @brief Coverts ConstraintTypes value to its string representation.
       * @param constraint_type type enumerator value (Primary, Auxiliary, Inactive)
       * @return std::string
       */
      inline static std::string enumToString(ConstraintTypes constraint_type) { return names_[constraint_type]; }

      /**
       * @brief Converts string to ConstraintTypes value
       * @param constraint_type_name enumerator string value (Primary, Auxiliary, Inactive)
       * @return ConstraintTypes
       */
      static ConstraintTypes stringToEnum(std::string constraint_type_name);

    protected:
      ConstraintTypes type_; /**< constraint type */
      static const std::string names_[]; /**< list of string names for ConstraintTypes */
      static const std::map<std::string, ConstraintTypes> name_to_enum_map_; /**< Map of ConstraintTypes string name to ConstraintType */
    };
  }// namespace constraint_types

  namespace initialization_state
  {
    /** @brief Enum that identifies the state of the solver. */
    enum InitializationState
    {
      PrimaryOnly,         /**< Solver is initialized and only contains primary constraints */
      AuxiliaryOnly,       /**< Solver is initialized and only contains auxiliary constraints */
      PrimaryAndAuxiliary, /**< Solver is initialized and contain both primary and auxiliary constraints */
      NothingInitialized,  /**< Solver is not initialized */
    };
  }// namespace initialization_state

  typedef constraint_types::ConstraintTypes ConstraintTypes;             /**< Typedef for ConstraintTypes in constrained_ik namespace */
  typedef initialization_state::InitializationState InitializationState; /**< Typedef for InitializationState in constrained_ik namespace */
}// namespace constrained_ik
#endif // ENUM_TYPES_H
