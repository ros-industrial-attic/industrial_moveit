/**
 * @file constrained_ik_utils.h
 * @brief Utility package
 *
 * @author Levi Armstrong
 * @date October 21, 2015
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef CONSTRAINED_IK_UTILS_H
#define CONSTRAINED_IK_UTILS_H

#include <XmlRpc.h>
#include <eigen3/Eigen/Core>
#include <constrained_ik/CLIKDynamicConfig.h>

namespace constrained_ik
{
  /** @brief The constrained IK solver configuration parameters */
  struct ConstrainedIKConfiguration
  {
    bool debug_mode;                   /**< Set the solver to a debug state. */
    bool allow_joint_convergence;      /**< Allow the solver to converge based on joint convergence. */
    bool allow_primary_normalization;  /**< Allow the solver to normalize primary motion. */
    bool allow_auxiliary_nomalization; /**< Allow the solver to normalize auxiliary motion. */
    bool limit_primary_motion;         /**< Allow the solver to limit the primary motion. */
    bool limit_auxiliary_motion;       /**< Allow the solver to limit the auxiliary motion. */
    bool limit_auxiliary_interations;  /**< Allow the solver to limit the number of auxiliary interation. */
    int solver_max_iterations;         /**< Solver's maximum number of iterations allowed. */
    int solver_min_iterations;         /**< Solver's minimum number of iterations required. */
    int auxiliary_max_iterations;      /**< Auxiliary's maximum number of iteration allowed. */
    double primary_max_motion;         /**< Primary's maximum motion allowed. */
    double auxiliary_max_motion;       /**< Auxiliary's maximum motion allowed. */
    double primary_norm;               /**< Normalize primary motion to set value. */
    double auxiliary_norm;             /**< Normalize auxiliary motion to set value. */
    double primary_gain;               /**< Solver's primary motion update gain. */
    double auxiliary_gain;             /**< Solver's auxiliary motion update gain. */
    double joint_convergence_tol;      /**< Solver's joint convergence tolerance. */
  };

  /**
   * @brief Convert ConstrainedIKDynamicReconfigureConfig to ConstrainedIKConfiguration
   * @param config The ConstrainedIKDynamicReconfigureConfig object
   * @return A ConstrainedIKConfiguration object
   */
  ConstrainedIKConfiguration convertToConstrainedIKConfiguration(CLIKDynamicConfig &config);

  template<typename T>
  void validateConstrainedIKConfiguration(T &config)
  {
    if (config.limit_auxiliary_motion)
    {
      if (config.auxiliary_norm > config.auxiliary_max_motion)
      {
        config.auxiliary_norm = config.auxiliary_max_motion;
      }
      else if (config.auxiliary_norm < config.auxiliary_max_motion)
      {
        unsigned int divisor = floor(config.auxiliary_max_motion/config.auxiliary_norm) + 1;
        config.auxiliary_norm = config.auxiliary_max_motion/divisor;
      }
    }
  }

  /**
   * @brief Get parameter from XmlRPCValue as double
   * @param config available parameters
   * @param key name of parameter to find
   * @param value populate results found in config
   * @return True if parameter was found in config, otherwise false
   */
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& value);

  /**
   * @brief Get parameter from XmlRPCValue as a vector of double's
   * @param config available parameters
   * @param key name of parameter to find
   * @param double_array populate results found in config
   * @return True if parameter was found in config, otherwise false
   */
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<double>& double_array);

  /**
   * @brief Get parameter from XmlRPCValue as Eigen::VectorXd
   * @param config available parameters
   * @param key name of parameter to find
   * @param eigen_vector populate results found in config
   * @return True if parameter was found in config, otherwise false
   */
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, Eigen::VectorXd& eigen_vector);

  /**
   * @brief Get parameter from XmlRPCValue as bool
   * @param config available parameters
   * @param key name of parameter to find
   * @param value populate results found in config
   * @return True if parameter was found in config, otherwise false
   */
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value);

  /**
   * @brief Get parameter from XmlRPCValue as vector of string's
   * @param config available parameters
   * @param key name of parameter to find
   * @param string_array populate results found in config
   * @return True if parameter was found in config, otherwise false
   */
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<std::string>& string_array);

}
#endif // CONSTRAINED_IK_UTILS_H

