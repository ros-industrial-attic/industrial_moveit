/**
* @file constrained_ik_utils.cpp
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
#include <constrained_ik/constrained_ik_utils.h>
#include <ros/ros.h>

namespace constrained_ik
{
  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& value)
  {
    if (!config.hasMember(key))
    {
      return false;
    }
    XmlRpc::XmlRpcValue param = config[key];
    if (param.getType() != XmlRpc::XmlRpcValue::TypeDouble)
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
    if(getParam(config, key, double_array))
    {
      eigen_vector = Eigen::VectorXd::Map(double_array.data(), double_array.size());
    }
    else
    {
      return false;
    }
  }

  bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value)
  {
    if (!config.hasMember(key))
      {
        return false;
      }
    XmlRpc::XmlRpcValue param = config[key];
    if (param.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
      return false;
    }
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
