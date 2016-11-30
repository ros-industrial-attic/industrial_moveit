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
namespace constrained_ik
{

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

