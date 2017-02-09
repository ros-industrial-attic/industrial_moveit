/**
 * @file constrained_ik_utils.cpp
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
#include <constrained_ik/constrained_ik_utils.h>
#include <ros/ros.h>

namespace constrained_ik
{
  ConstrainedIKConfiguration convertToConstrainedIKConfiguration(CLIKDynamicConfig &config)
  {
    ConstrainedIKConfiguration c;
    c.debug_mode = config.debug_mode;
    c.allow_joint_convergence = config.allow_joint_convergence;
    c.allow_primary_normalization = config.allow_primary_normalization;
    c.allow_auxiliary_nomalization = config.allow_auxiliary_nomalization;
    c.limit_primary_motion = config.limit_primary_motion;
    c.limit_auxiliary_motion = config.limit_auxiliary_motion;
    c.limit_auxiliary_interations = config.limit_auxiliary_interations;
    c.solver_max_iterations = config.solver_max_iterations;
    c.solver_min_iterations = config.solver_min_iterations;
    c.auxiliary_max_iterations = config.auxiliary_max_iterations;
    c.primary_max_motion = config.primary_max_motion;
    c.auxiliary_max_motion = config.auxiliary_max_motion;
    c.primary_norm = config.primary_norm;
    c.auxiliary_norm = config.auxiliary_norm;
    c.primary_gain = config.primary_gain;
    c.auxiliary_gain = config.auxiliary_gain;
    c.joint_convergence_tol = config.joint_convergence_tol;
    return c;
  }

  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& value)
  {
    if (!config.hasMember(key))
    {
      ROS_ERROR("XmlRpcValue does not contain key %s.", key.c_str());
      return false;
    }
    XmlRpc::XmlRpcValue param = config[key];
    if (param.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        param.getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      return false;
    }
    value = param;
    return true;
  }

  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<double>& double_array)
  {
    if (!config.hasMember(key))
    {
      ROS_ERROR("XmlRpcValue does not contain key %s.", key.c_str());
      return false;
    }

    XmlRpc::XmlRpcValue d_array_xml = config[key];

    if (d_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("XmlRpcValue is not of type array.");
      return false;
    }

    double_array.clear();
    for (int i=0; i<d_array_xml.size(); ++i)
    {
      if (d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
          d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
        return false;
      }
      double value = 0.0;
      if (d_array_xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        value = static_cast<double>(static_cast<int>(d_array_xml[i]));
      }
      else
      {
        value = static_cast<double>(d_array_xml[i]);
      }
      double_array.push_back(value);
    }

    return true;
  }

  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, Eigen::VectorXd& eigen_vector)
  {
    std::vector<double> double_array;

    if(!getParam(config, key, double_array))
      return false;

    eigen_vector = Eigen::VectorXd::Map(double_array.data(), double_array.size());
    return true;
  }

  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value)
  {
    if (!config.hasMember(key))
    {
      ROS_ERROR("XmlRpcValue does not contain key %s.", key.c_str());
      return false;
    }

    XmlRpc::XmlRpcValue param = config[key];
    if (param.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
      return false;

    value = param;
    return true;
  }

  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<std::string>& string_array)
  {
    if (!config.hasMember(key))
    {
      ROS_ERROR("XmlRpcValue does not contain key %s.", key.c_str());
      return false;
    }

    XmlRpc::XmlRpcValue s_array_xml = config[key];

    if (s_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("XmlRpcValue is not of type array.");
      return false;
    }

    string_array.clear();
    for (int i=0; i<s_array_xml.size(); ++i)
    {
      if (s_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR("XmlRpcValue is not a string array.");
        return false;
      }
      else
      {
        string_array.push_back(s_array_xml[i]);
      }
    }

    return true;
  }
}
