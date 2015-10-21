/**
* @file constrained_ik_utils.h
* @brief Utility package
* @author Levi Armstrong
* @date October 21, 2015
* @version TODO
* @bug No known bugs
*
* @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef CONSTRAINED_IK_UTILS_H
#define CONSTRAINED_IK_UTILS_H

#include <XmlRpc.h>
#include <eigen3/Eigen/Core>
namespace constrained_ik
{

  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& value);
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<double>& double_array);
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, Eigen::VectorXd& eigen_vector);
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value);
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<std::string>& string_array);

}
#endif // CONSTRAINED_IK_UTILS_H

